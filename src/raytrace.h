#pragma once
#include "shapes.h"
#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <vector>

using Vec3d = Eigen::Vector3d;
using Matrix43d = Eigen::Matrix<double, 4, 3>;

// TODO: Add wavelet noise textures
//  torus analytical intersect
//  cube analytical intersect
//  remove virtual functions - use homogenous vectors
//  analytic intersection and norm for spheres/planes
//  bumpmap
//  reflection
//  refraction
class PointLight {
  public:
    PointLight(const Vec3d &p, const Vec3d &c, const double &i) : pos(p), col(c), intensity(i) {}
    Vec3d pos;
    Vec3d col;
    double intensity;
};

class Scene {
  public:
    Scene() {
        mObjects.push_back(std::make_shared<Plane<TextureCheck>>(Vec3d(0, 1, 0), Vec3d(0, -2, 0)));
        // mObjects.push_back(std::make_shared<Cube>(Vec3d(1.5, 1.5, 1.5)));
        // mObjects.push_back(std::make_shared<Torus>(Vec3d(2, 0.5, 0.0), 2, 0.65));
        //
        mObjects.push_back(std::make_shared<Sphere<TextureCheck>>(Vec3d(-3, 0.0, 0.0), 2));
        mObjects.push_back(std::make_shared<Sphere<TextureCheck>>(Vec3d(3, 0.0, 0.0), 1));

        // mObjects.push_back(std::make_shared<Blend>(
        // std::make_shared<Cube>(Vec3d(1.5)),
        // std::make_shared<Torus>(2, 0.65), 5));

        // mObjects.push_back(std::make_shared<Mix<Cube, Sphere<TextureCheck>>>(
        // Cube(Vec3d(1,1,1)),
        // Sphere<TextureCheck>(Vec3d(0,0,0), 1), 0.5));

        // mObjects.push_back(std::make_shared<SoftObject>());

        mLights.push_back(std::make_unique<PointLight>(Vec3d(20, 10, 20), Vec3d(1.0, 0.2, 0.2), 10000));
        mLights.push_back(std::make_unique<PointLight>(Vec3d(0, 10, 20), Vec3d(0.2, 1, 0.2), 5000));
        mLights.push_back(std::make_unique<PointLight>(Vec3d(-20, 10, 20), Vec3d(0.2, 0.2, 1.0), 10000));
        mLights.push_back(std::make_unique<PointLight>(Vec3d(0, 10, -20), Vec3d(1, 1, 1), 10000));

        lookAt(Vec3d(0, 1, 9), Vec3d(0, 0, 0));
    };

    void update(const double time) { lookAt(Vec3d(0, 1, 2 + time / 10.0f), Vec3d(0, 0, 0)); }

    std::vector<std::shared_ptr<ImplicitShape>> mObjects;
    std::vector<std::unique_ptr<PointLight>> mLights;

    Matrix43d mCamToWorld; // normalized vectors for x,y,z orientation

    void setCameraOrientation(const Vec3d &z, Vec3d camUp = Vec3d(0, 1, 0)) {
        mCamToWorld.row(0) = (camUp.cross(z)).normalized();
        mCamToWorld.row(1) = z.cross(mCamToWorld.row(0));
        mCamToWorld.row(2) = z;
    }

  private:
    void lookAt(const Vec3d &camPos, const Vec3d &to, Vec3d camUp = Vec3d(0, 1, 0)) {
        // because the forward axis in a right hand coordinate system points
        // backward we compute -(to - camPos)
        Vec3d z = (camPos - to).normalized(); // vector in direction that camera points
        setCameraOrientation(z, camUp);

        // set position
        mCamToWorld.row(3) = camPos;
    }
};

class RayTrace {
  public:
    inline float traceShadow(const Vec3d &rayOrigin, const Vec3d &rayDirection, const float &maxDistance,
                             const ImplicitShape *exclude) {
        double distance;
        for (const auto &shape : mScene.mObjects) {
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

    inline Vec3d traceRay(const Vec3d &rayOrigin, const Vec3d &rayDirection) {
        // std::cout << "trace from: " << rayOrigin.transpose() << " dir: " <<
        // rayDirection.transpose() << std::endl;
        ImplicitShape *isectShape = nullptr;
        double minDistance = std::numeric_limits<double>::max();

        // check all shapes for first intersection
        for (const auto &shape : mScene.mObjects) {
            double distance;
            if (shape->intersect(rayOrigin, rayDirection, distance) && (distance < minDistance)) {
                [[unlikely]];
                // std::cout << "Intersect at: " << distance << std::endl;
                isectShape = shape.get();
                minDistance = distance;
            }
        }
        if (minDistance != std::numeric_limits<double>::max()) {
            [[unlikely]];
            Vec3d surfaceNorm;
            Vec3d color;
            Vec3d isectPoint = rayOrigin + minDistance * rayDirection;
            isectShape->getSurfaceData(isectPoint, surfaceNorm, color);

            // loop over all lights in the scene and add their contribution to P's
            // brightness
            Vec3d R{0.5, 0.5, 0.5};
            constexpr float decayScale = 8.0f;
            for (const auto &light : mScene.mLights) {
                Vec3d lightDir = light->pos - isectPoint;
                const float dist = lightDir.norm();
                lightDir /= dist;

                // Lambertian shading. The light per unit area depends on incidence
                // angle vs normal
                const float cosAngle = lightDir.dot(surfaceNorm);
                if (cosAngle > 0) {
                    [[unlikely]];
                    // sphere trace the light source, so if we are illuminated or in
                    // shadow
                    const float shadow = 1.0f - traceShadow(isectPoint, lightDir, dist, isectShape);
                    R += shadow * cosAngle * light->col * light->intensity / (decayScale * dist);
                }

                // albedo parameter is reflected light/incident light
                //  diffuse light is independent of light incident angle
            }
            Vec3d finalCol = R.cwiseProduct(color) / (decayScale * minDistance);
            //    std::cout << "Final color: " << finalCol.transpose() << " minDist: "
            //    << minDistance << std::endl;
            return finalCol;
        }

        return Vec3d(0, 0, 0);
    }

    template <typename IMAGE_T> void draw(IMAGE_T &image) {
        // mScene.update(mTime++);
        constexpr double width = image.GRID_SIZE, height = image.GRID_SIZE;
        constexpr double ratio = width / height;
        double angle = tan((mFov * 0.5) / 180.f * M_PI);

        Vec3d const &rayOrigin = mScene.mCamToWorld.row(3);
        Vec3d pointFromCam(0, 0, -1);
        Eigen::Matrix<double, 3, 3> camRotate = mScene.mCamToWorld.topRows(3).transpose();

        constexpr double invHeight = 1.0 / height;
        constexpr double invWidth = 1.0 / width;
        for (uint32_t j = 0; j < height; ++j) {
            pointFromCam[1] = (1 - j * invHeight * 2) * angle;
            for (uint32_t i = 0; i < width; ++i) {
                pointFromCam[0] = (2 * i * invWidth - 1) * ratio * angle;
                // pointFromCam /= fastSqrt(pointFromCam.squaredNorm(), 0.01);
                double norm = fastSqrt(pointFromCam.squaredNorm(), 0.05);
                pointFromCam /= norm;

                Vec3d pixelColor = traceRay(rayOrigin, camRotate * pointFromCam);
                image.image[image.POS(i, j)] =
                    Pixel{std::min<float>(1.0, pixelColor(0)), std::min<float>(1.0, pixelColor(1)),
                          std::min<float>(1.0, pixelColor(2))};
            }
        }
    }

    void rotateCamera(const float moveX, const float moveY) {
        // x move on the screen rotates the camera z to the right
        const float fovRad = mFov * M_PI / 180.0f;
        // set z-axis
        Vec3d z = mScene.mCamToWorld.row(2);
        const float cosX = cos(moveX * fovRad);
        const float sinX = sin(moveX * fovRad);
        z[0] = z[0] * cosX - z[2] * sinX;
        z[2] = z[0] * sinX + z[2] * cosX;

        const float cosY = cos(-moveY * fovRad);
        const float sinY = sin(-moveY * fovRad);
        z[1] = z[1] * cosY - z[2] * sinY;
        z[2] = z[1] * sinY + z[2] * cosY;
        mScene.setCameraOrientation(z.normalized());
    }

    void moveCamera(const float x, const float y, const float z) {
        mScene.mCamToWorld.row(3) += mScene.mCamToWorld.topRows(3) * Vec3d(x, y, z);
    }

  private:
    Scene mScene;
    float mTime{};
    float mFov{60};
};
