#pragma once
#include <cstdlib>
#include <cstdio>
#include <fstream>
// #include "geometry.h"
#include <cmath>
#include <vector>
#include <limits>
#include <memory>
#include "shapes.h"
#include <Eigen/Dense>

using Vec3d = Eigen::Vector3d;
using Matrix43d = Eigen::Matrix<double,4,3>;

//TODO: Add wavelet noise textures
// torus analytical intersect
// cube analytical intersect
// remove virtual functions - use homogenous vectors
// analytic intersection and norm for spheres/planes
// bumpmap
// reflection
// refraction
class PointLight
{
public:
    PointLight(const Vec3d& p, const Vec3d& c, const double& i) : pos(p), col(c), intensity(i) {}
    Vec3d pos;
    Vec3d col;
    double intensity;
};

class Scene
{
public:
    Scene()
    {
        mObjects.push_back(std::make_shared<Plane<TextureCheck>>(Vec3d(0, 1, 0), Vec3d(0, -2, 0)));
        // mObjects.push_back(std::make_shared<Cube>(Vec3d(1.5)));
        // mObjects.push_back(std::make_shared<Torus>(Vec3d(2, 0.5, 0.0), 2, 0.65));
        mObjects.push_back(std::make_shared<Sphere<TextureCheck>>(Vec3d(-3, 0.0, 0.0), 2));
        mObjects.push_back(std::make_shared<Sphere<TextureCheck>>(Vec3d(3, 0.0, 0.0), 1));
        /*
        mObjects.push_back(std::make_shared<Blend>(
            std::make_shared<Cube>(Vec3d(1.5)),
            std::make_shared<Torus>(2, 0.65), 5));
        mObjects.push_back(std::make_shared<Mix<Cube, Sphere>>(
            Cube(Vec3d(1)),
            Sphere(Vec3d(0), 1), 0.5));
            */
        //mObjects.push_back(std::make_shared<SoftObject>());

        mLights.push_back(std::make_unique<PointLight>(Vec3d( 20, 10,  20), Vec3d(1.0, 0.2, 0.2), 10000));
        mLights.push_back(std::make_unique<PointLight>(Vec3d( 0, 10, 20), Vec3d(0.2, 1, 0.2), 5000));
        mLights.push_back(std::make_unique<PointLight>(Vec3d( -20, 10,  20), Vec3d(0.2, 0.2, 1.0), 10000));
        mLights.push_back(std::make_unique<PointLight>(Vec3d( 0, 10,  -20), Vec3d(1, 1, 1), 10000));

        lookAt(Vec3d(0, 1, 9), Vec3d(0,0,0));
    };

    void update(const double time)
    {
        lookAt(Vec3d(0, 1, 2+time/10.0f), Vec3d(0,0,0));
    }

    std::vector<std::shared_ptr<ImplicitShape>> mObjects;
    std::vector<std::unique_ptr<PointLight>> mLights;

    Matrix44f mCamToWorld; // normalized vectors for x,y,z orientation

    void setCameraOrientation(const Vec3d& z, Vec3d camUp = Vec3d(0, 1, 0))
    {
        mCamToWorld.row(0) = (camUp.crossProduct(z)).normalize();
        mCamToWorld.row(1) = z.crossProduct(x);
        mCamToWorld.row(2) = z;
    }

private:
    void lookAt(const Vec3d& camPos, const Vec3d& to, Vec3d camUp = Vec3d(0, 1, 0))
    {
        // because the forward axis in a right hand coordinate system points backward we compute -(to - camPos)
        Vec3d z = (camPos - to).normalized(); // vector in direction that camera points
        setCameraOrientation(z, camUp);

        // set position
        mCamToWorld.row(3) = camPos[0];
    }
};


class RayTrace
{
public:
    float traceShadow(const Vec3d& rayOrigin,
                           const Vec3d& rayDirection, 
                           const float& maxDistance,
                           const ImplicitShape* exclude)
    { 
        double distance;
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

    Vec3d traceRay(const Vec3d& rayOrigin, const Vec3d& rayDirection)
    {
        ImplicitShape* isectShape = nullptr;
        double minDistance = std::numeric_limits<double>::max();

        // check all shapes for first intersection
        for (const auto& shape : mScene.mObjects) {
            double distance;
            if (shape->intersect(rayOrigin, rayDirection, distance) &&
               (distance < minDistance))
            {
                [[unlikely]];
                isectShape = shape.get();
                minDistance = distance;
            }
        }
        if (minDistance != std::numeric_limits<double>::max())
        {
            [[unlikely]];
            Vec3d surfaceNorm;
            Vec3d color;
            Vec3d isectPoint = rayOrigin + minDistance * rayDirection;
            isectShape->getSurfaceData(isectPoint, surfaceNorm, color);
         
            // loop over all lights in the scene and add their contribution to P's brightness
            Vec3d R{0.5, 0.5, 0.5};
            constexpr float decayScale = 8.0f;
            for (const auto& light: mScene.mLights) {
                Vec3d lightDir = light->pos - isectPoint;
                const float dist = lightDir.norm(); 
		lightDir /= dist;

                // Lambertian shading. The light per unit area depends on incidence angle vs normal
                const float cosAngle = lightDir.dotProduct(surfaceNorm);
                if (cosAngle > 0) {
                    [[unlikely]];
                    // sphere trace the light source, so if we are illuminated or in shadow
                    const float shadow = 1.0f - traceShadow(isectPoint, lightDir, dist, isectShape);
                    R += shadow * cosAngle * light->col * light->intensity / (decayScale * dist); 
                }

                //albedo parameter is reflected light/incident light

                // diffuse light is independent of light incident angle
            }
            return R.cwiseProduct(color) / (decayScale * minDistance);
        }

        return Vec3d(0,0,0);
    }

    void draw(auto& image)
    {
        //mScene.update(mTime++);
        constexpr int16_t width = image.GRID_SIZE, height = image.GRID_SIZE;
        constexpr double ratio = width / static_cast<double>(height);
        double angle = tan((mFov * 0.5) / 180.f * M_PI);

        Vec3d src{0,0,0};
        Eigen::Vector4d tmp = mScene.mCamToWorld * src;
        Vec3d rayOrigin{tmp[0]/tmp[3], tmp[1]/tmp[3], tmp[2]/tmp[3]};

        // Vec3d rayOrigin = mScene.mCamToWorld.multVecMatrix(Vec3d(0));
        for (uint32_t j = 0; j < height; ++j) {
            for (uint32_t i = 0; i < width; ++i) {
                double x = (2 * i / static_cast<double>(width) - 1) * ratio * angle;
                double y = (1 - j / static_cast<double>(height) * 2) * angle;
                // Vec3d rayDirection = mScene.mCamToWorld.multDirMatrix(Vec3d(x, y, -1).normalized());
                Vec3d rayDirection = (mScene.mCamToWorld * (Vec3d(x, y, -1).normalized())).head(3);

                Vec3d pixelColor = sphereTrace(rayOrigin, rayDirection);
                image.image[image.POS(i, j)] = Pixel{std::min<float>(1.0, pixelColor[0]),
                                                    std::min<float>(1.0, pixelColor[1]),
                                                    std::min<float>(1.0, pixelColor[2])};
            }
        }
    }

    void rotateCamera(const float moveX, const float moveY)
    {
        //std::cout << moveX << " " << moveY << std::endl;
        // x move on the screen rotates the camera z to the right
        const float fovRad = mFov * M_PI / 180.0f;
        // set z-axis
        Vec3d z{mScene.mCamToWorld[2][0],
                mScene.mCamToWorld[2][1],
                mScene.mCamToWorld[2][2]};
        const float cosX = cos(moveX * fovRad);
        const float sinX = sin(moveX * fovRad);
        z.x = z.x * cosX - z.z * sinX;
        z.z = z.x * sinX + z.z * cosX;

        const float cosY = cos(-moveY * fovRad);
        const float sinY = sin(-moveY * fovRad);
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
