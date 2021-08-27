#pragma once
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include "geometry.h"
#include <cmath>
#include <vector>
#include <limits>
#include <memory>
#include "shapes.h"

//TODO: Add basic textures - checkerboard/Perlin
// remove virtual functions - use homogenous vectors
// analytic intersection and norm for spheres/planes
// bumpmap
// reflection
// refraction
class PointLight
{
public:
    PointLight(const Vec3f& p, const Vec3f& c, const float& i) : pos(p), col(c), intensity(i) {}
    Vec3f pos;
    Vec3f col;
    float intensity;
};

class Scene
{
public:
    Scene()
    {
        mObjects.push_back(std::make_shared<Plane<TextureCheck>>(Vec3f(0, 1, 0), Vec3f(0, -2, 0)));
        // mObjects.push_back(std::make_shared<Cube>(Vec3f(1.5)));
        mObjects.push_back(std::make_shared<Torus>(Vec3f(2, 0.5, 0.0), 2, 0.65));
        mObjects.push_back(std::make_shared<Sphere<TextureCheck>>(Vec3f(-3, 0.0, 0.0), 2));
        /*
        mObjects.push_back(std::make_shared<Blend>(
            std::make_shared<Cube>(Vec3f(1.5)),
            std::make_shared<Torus>(2, 0.65), 5));
        mObjects.push_back(std::make_shared<Mix<Cube, Sphere>>(
            Cube(Vec3f(1)),
            Sphere(Vec3f(0), 1), 0.5));
            */
        //mObjects.push_back(std::make_shared<SoftObject>());

        mLights.push_back(std::make_unique<PointLight>(Vec3f( 20, 10,  20), Vec3f(1.0, 0.2, 0.2), 10000));
        mLights.push_back(std::make_unique<PointLight>(Vec3f( 0, 10, 20), Vec3f(0.2, 1, 0.2), 5000));
        mLights.push_back(std::make_unique<PointLight>(Vec3f( -20, 10,  20), Vec3f(0.2, 0.2, 1.0), 10000));
        mLights.push_back(std::make_unique<PointLight>(Vec3f( 0, 10,  -20), Vec3f(1, 1, 1), 10000));

        lookAt(Vec3f(0, 1, 9), 0);
    };

    void update(const float time)
    {
        lookAt(Vec3f(0, 1, 2+time/10.0f), 0);
    }

    std::vector<std::shared_ptr<ImplicitShape>> mObjects;
    std::vector<std::unique_ptr<PointLight>> mLights;
    Matrix44f mCamToWorld; // normalized vectors for x,y,z orientation

    void setCameraOrientation(const Vec3f& z, Vec3f camUp = Vec3f(0, 1, 0))
    {
        Vec3f x = camUp.normalize().crossProduct(z).normalize();
        Vec3f y = z.crossProduct(x);

        // set x-axis
        mCamToWorld[0][0] = x[0];
        mCamToWorld[0][1] = x[1];
        mCamToWorld[0][2] = x[2];

        // set y-axis
        mCamToWorld[1][0] = y[0];
        mCamToWorld[1][1] = y[1];
        mCamToWorld[1][2] = y[2];

        // set z-axis
        mCamToWorld[2][0] = z[0];
        mCamToWorld[2][1] = z[1];
        mCamToWorld[2][2] = z[2];
    }

private:
    void lookAt(const Vec3f& camPos, const Vec3f& to, Vec3f camUp = Vec3f(0, 1, 0))
    {
        // because the forward axis in a right hand coordinate system points backward we compute -(to - camPos)
        Vec3f z = (camPos - to).normalize();  // vector in direction that camera points
        setCameraOrientation(z, camUp);

        // set position
        mCamToWorld[3][0] = camPos[0];
        mCamToWorld[3][1] = camPos[1];
        mCamToWorld[3][2] = camPos[2];
    }
};


class RayTrace
{
public:
    float sphereTraceShadow(const Vec3f& rayOrigin,
                           const Vec3f& rayDirection, 
                           const float& maxDistance,
                           const ImplicitShape* exclude)
    { 
        float distance;
        for (const auto& shape : mScene.mObjects) {
            if (shape.get() != exclude) {
                [[likely]];
                if ((shape->intersect(rayOrigin, rayDirection, distance)) && (distance < maxDistance)) {
                    [[unlikely]];
                    return 1.0f;
                }
            }
        }
     
        return 0.0f; 
    }

    Vec3f sphereTrace(const Vec3f& rayOrigin, const Vec3f& rayDirection)
    {
        ImplicitShape* isectShape = nullptr;
        float minDistance = std::numeric_limits<float>::max();

        // check all shapes for first intersection
        for (const auto& shape : mScene.mObjects) {
            float distance;
            if (shape->intersect(rayOrigin, rayDirection, distance) &&
               (distance < minDistance))
            {
                [[unlikely]];
                isectShape = shape.get();
                minDistance = distance;
            }
        }
        if (minDistance != std::numeric_limits<float>::max())
        {
            [[unlikely]];
            Vec3f surfaceNorm;
            Vec3f color;
            Vec3f isectPoint = rayOrigin + minDistance * rayDirection;
            isectShape->getSurfaceData(isectPoint, surfaceNorm, color);
         
            // loop over all lights in the scene and add their contribution to P's brightness
            Vec3f R{0.5, 0.5, 0.5};
            constexpr float decayScale = 8.0f;
            for (const auto& light: mScene.mLights) {
                Vec3f lightDir = light->pos - isectPoint;
                const float dist = lightDir.makeNormed(); 
                const float cosAngle = lightDir.dotProduct(surfaceNorm);
                if (cosAngle > 0) {
                    [[unlikely]];
                    // sphere trace the light source, so if we are illuminated or in shadow
                    const float shadow = 1.0f - sphereTraceShadow(isectPoint, lightDir, dist, isectShape);
                    R += shadow * cosAngle * light->col * light->intensity / (decayScale * dist); 
                }
            }
            return R * color / (decayScale * minDistance);
        }

        return 0; 
    }

    void draw(auto& image)
    {
        //mScene.update(mTime++);
        constexpr int16_t width = image.GRID_SIZE, height = image.GRID_SIZE;
        constexpr float ratio = width / static_cast<float>(height);
        float angle = tan((mFov * 0.5) / 180.f * M_PI);

        Vec3f rayOrigin = mScene.mCamToWorld.multVecMatrix(Vec3f(0));
        for (uint32_t j = 0; j < height; ++j) {
            for (uint32_t i = 0; i < width; ++i) {
                float x = (2 * i / static_cast<float>(width) - 1) * ratio * angle;
                float y = (1 - j / static_cast<float>(height) * 2) * angle;
                Vec3f rayDirection = mScene.mCamToWorld.multDirMatrix(Vec3f(x, y, -1).normalize());
                Vec3f pixelColor = sphereTrace(rayOrigin, rayDirection);
                image.image[image.POS(i, j)] = Pixel{std::min(1.0f, pixelColor[0]), std::min(1.0f, pixelColor[1]), std::min(1.0f, pixelColor[2])};
            }
        }
    }

    void rotateCamera(const float moveX, const float moveY)
    {
        //std::cout << moveX << " " << moveY << std::endl;
        // x move on the screen rotates the camera z to the right
        const float fovRad = mFov * M_PI / 180.0f;
        // set z-axis
        Vec3f z{mScene.mCamToWorld[2][0],
                mScene.mCamToWorld[2][1],
                mScene.mCamToWorld[2][2]};
        const float cosX = cos(moveX * fovRad);
        const float sinX = sin(moveX * fovRad);
        z.x = z.x * cosX - z.z * sinX;
        z.z = z.x * sinX + z.z * cosX;

        [[maybe_unused]] const float cosY = cos(-moveY * fovRad);
        [[maybe_unused]] const float sinY = sin(-moveY * fovRad);
        z.y = z.y * cosY - z.z * sinY;
        z.z = z.y * sinY + z.z * cosY;
        mScene.setCameraOrientation(z.normalize());
    }

    void moveCamera(const float x, const float y, const float z)
    {
        mScene.mCamToWorld[3][0] += x * mScene.mCamToWorld[0][0];
        mScene.mCamToWorld[3][1] += x * mScene.mCamToWorld[0][1];
        mScene.mCamToWorld[3][2] += x * mScene.mCamToWorld[0][2];

        mScene.mCamToWorld[3][0] += y * mScene.mCamToWorld[1][0];
        mScene.mCamToWorld[3][1] += y * mScene.mCamToWorld[1][1];
        mScene.mCamToWorld[3][2] += y * mScene.mCamToWorld[1][2];

        mScene.mCamToWorld[3][0] += z * mScene.mCamToWorld[2][0];
        mScene.mCamToWorld[3][1] += z * mScene.mCamToWorld[2][1];
        mScene.mCamToWorld[3][2] += z * mScene.mCamToWorld[2][2];
    }

private:
    Scene mScene;
    float mTime{};
    float mFov{60};
};
