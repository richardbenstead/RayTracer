#pragma once
#include <iostream>

static constexpr float INTERSECT_TH = 1e-2; 
static constexpr float MAX_DISTANCE = 50;

struct TextureCheck
{
    float getVal(const float x, const float y, const float scale=20) const {
        return ((int8_t(x * scale) ^ int8_t(y * scale)) & 1);
    }
};

// ImplicitShape base class
class ImplicitShape
{
public:
    virtual ~ImplicitShape() {}

    // default intersection using sphere-trace
    virtual bool intersect(const Vec3f& orig, const Vec3f& dir, float &t) const = 0;

    // Compute the surface data such as normal and texture coordinates at the intersection point.
    virtual void getSurfaceData([[maybe_unused]] const Vec3f& point, Vec3f& norm, Vec3f& col) const = 0;

    Vec3f color{1,1,1};
};

// Implicit sphere surface
template<typename TEX_T=TextureCheck>
class Sphere : public ImplicitShape
{
public:
    Sphere(const Vec3f& c, const float& r) : center{c}, radius{r}, radius2{r*r} {}

    bool intersect(const Vec3f& orig, const Vec3f& dir, float &t) const
    {
        const Vec3f L = center - orig; 
        const float tca = L.dotProduct(dir);    // project L along dir to get distance to point tangental to center
        const float d2 = L.norm() - tca * tca;  // square of distance from tangent to center via pythagoras
        if ((d2 > radius2) || (tca < 0)) {
            [[likely]];
            return false; 
        }
        const float thc = sqrt(radius2 - d2); 
        t = tca - thc;  // first intersect
        return true;
    }

    void getSurfaceData([[maybe_unused]] const Vec3f& point, Vec3f& norm, Vec3f& col) const
    {
        norm = (point - center).normalize();
        col = color * texture.getVal((1 + atan2(norm.z, norm.x) / M_PI) * 0.5,
                                      acosf(norm.y) / M_PI);
    }
    const TEX_T texture{};
    const Vec3f center;
    const float radius, radius2;
};

// Implicit plane surface
template<typename TEX_T=TextureCheck>
class Plane : public ImplicitShape
{
public:
    Plane(const Vec3f& _normal = Vec3f(0, 1, 0),
          const Vec3f& pp = Vec3f(0)) : normal(_normal), pointOnPlane(pp) {}

    bool intersect(const Vec3f& orig, const Vec3f& dir, float &t) const
    {
        // assuming vectors are all normalized
        // (intersect_point (p) - pointOnPlane (p)).dot(normal) = 0
        // intersect_point = (orig + dir * t) 
        // => t = (intersect point - orig).dot(normal) / dir.dot(normal)
        const float denom = dir.dotProduct(normal);
        if (denom < -1e-6) { 
            Vec3f p0l0 = pointOnPlane - orig; 
            t = p0l0.dotProduct(normal) / denom; 

            return (t > 0); 
        } 
     
        return false;
    }

    void getSurfaceData([[maybe_unused]] const Vec3f& point, Vec3f& norm, Vec3f& col) const
    {
        norm = normal;
        const Vec3f tex = point.crossProduct(normal);
        col = color * texture.getVal(tex.x, tex.z, 2);
    }

    const TEX_T texture{};
    const Vec3f normal;
    const Vec3f pointOnPlane;
};

// Implicit torus surface
class Torus : public ImplicitShape
{
public:
    Torus(const Vec3f& _center, const float& _r0, const float& _r1) : center(_center), r0(_r0), r1(_r1) {}
    float getDistance(const Vec3f& from) const 
    {
        Vec3f pointObjectSpace = from - center;
        // reduce 3D point to 2D point
        const float tmpx = sqrtf(pointObjectSpace.x * pointObjectSpace.x + pointObjectSpace.z * pointObjectSpace.z) - r0;
        const float tmpy = pointObjectSpace.y;
        
        // distance to cicle
        return sqrtf(tmpx * tmpx + tmpy * tmpy) - r1;
    }
    bool intersect(const Vec3f& orig, const Vec3f& dir, float &t) const
    {
        float distance;
        for(t=0; t<MAX_DISTANCE; t+=distance) {
            Vec3f point = orig + t * dir;
            distance = getDistance(point);

            if (distance <= INTERSECT_TH * t) {
                [[unlikely]];
                return true;
            }
        }
        return false;
    }
    // Method to compute the surface data such as normal and texture coordinates at the intersection point.
    // See method implementation in children class for details
    void getSurfaceData([[maybe_unused]] const Vec3f& point, Vec3f& norm, Vec3f& col) const
    {
        constexpr float delta = 10e-5; 
        const float base = getDistance(point);
        norm = Vec3f(getDistance(point + Vec3f(delta, 0, 0)) - base,
                     getDistance(point + Vec3f(0, delta, 0)) - base,
                     getDistance(point + Vec3f(0, 0, delta)) - base).normalize();
        col = color;
    }
    const Vec3f center;
    const float r0, r1;
};

// Implicit cube surface
class Cube : public ImplicitShape
{
public:
    Cube(const Vec3f &_corner) : corner(_corner) {}
    inline float getDistance(const Vec3f& point) const 
    {
        constexpr float scale = 1;

        Vec3f pointObjectSpace = point * 1.f / scale;
        pointObjectSpace.x = std::fabs(pointObjectSpace.x);
        pointObjectSpace.y = std::fabs(pointObjectSpace.y);
        pointObjectSpace.z = std::fabs(pointObjectSpace.z);
        
        // now compute the distance from the point to the surface of the object
        Vec3f d = pointObjectSpace - corner;
        
        float dmaxLen = sqrt(d.x * d.x * (d.x > 0) + 
                             d.y * d.y * (d.y > 0) + 
                             d.z * d.z * (d.z > 0));
        
        // don't forget to apply the scale back
        return scale * (std::min(std::max(d.x, std::max(d.y, d.z)), 0.f) + dmaxLen);
    }
    bool intersect(const Vec3f& orig, const Vec3f& dir, float &t) const
    {
        float distance;
        for(t=0; t<MAX_DISTANCE; t+=distance) {
            Vec3f point = orig + t * dir;
            distance = getDistance(point);

            if (distance <= INTERSECT_TH * t) {
                [[unlikely]];
                return true;
            }
        }
        return false;
    }
    // Method to compute the surface data such as normal and texture coordinates at the intersection point.
    // See method implementation in children class for details
    void getSurfaceData([[maybe_unused]] const Vec3f& point, Vec3f& norm, Vec3f& col) const
    {
        constexpr float delta = 10e-5; 
        const float base = getDistance(point);
        norm = Vec3f(getDistance(point + Vec3f(delta, 0, 0)) - base,
                     getDistance(point + Vec3f(0, delta, 0)) - base,
                     getDistance(point + Vec3f(0, 0, delta)) - base).normalize();
        col = color;
    }
    const Vec3f corner{};
};

struct unionFunc
{
    float operator() (float a, float b) const { return std::min(a, b); }
};

struct subtractFunc
{
    float operator() (float a, float b) const { return std::max(-a, b); }
};

struct intersectionFunc
{
    float operator() (float a, float b) const { return std::max(a, b); }
};

struct blendFunc
{
    blendFunc(const float &_k) : k(_k) {}
    float operator() (float a, float b) const
    {
        float res = exp(-k * a) + exp(-k * b);
        return -log(std::max(0.0001f, res)) / k;
    }
    float k;
};

struct mixFunc
{
    mixFunc(const float &_t) : t(_t) {}
    float operator() (float a, float b) const
    {
        return a * (1 -t) + b * t;
    }
    float t;
};

// Combining implict shapes using CSG
template<typename Op, typename S1, typename S2, typename ... Args>
class CSG : public ImplicitShape
{
public:
    CSG(
        const S1& s1,
        const S2& s2,
        Args&& ... args) : op(std::forward<Args>(args) ...), shape1(s1), shape2(s2)
    {}
    float getDistance(const Vec3f& from) const
    {
        return op(shape1.getDistance(from), shape2.getDistance(from));
    }
    Op op;
    const S1 shape1;
    const S2 shape2;
};

// Blobbies
class SoftObject : public ImplicitShape
{
    struct Blob
    {
        float R; // radius
        Vec3f c; // blob center
    };
public:
    SoftObject()
    {
#if 0
        blobbies.push_back({2.0, Vec3f(-1, 0, 0)});
        blobbies.push_back({1.5, Vec3f( 1, 0, 0)});
#else
        for (size_t i = 0; i < 20; ++i) {
            float radius = (0.3 + drand48() * 1.3);
            Vec3f c((0.5 - drand48()) * 3, (0.5 - drand48()) * 3, (0.5 - drand48()) * 3);
            blobbies.push_back({radius, c});
        }
#endif
    }
    float getDistance(const Vec3f& from) const
    {
        float sumDensity = 0;
        float sumRi = 0;
        float minDistance = std::numeric_limits<float>::max();
        for (const auto& blob: blobbies) {
            float r = (blob.c - from).length();
            if (r <=  blob.R) {
                // this can be factored for speed if you want
                sumDensity += 2 * (r * r * r) / (blob.R * blob.R * blob.R) -
                    3 * (r * r) / (blob.R * blob.R) + 1;
            }
            minDistance = std::min(minDistance, r - blob.R);
            sumRi += blob.R;
        }

        return std::max(minDistance, (magic - sumDensity) / ( 3 / 2.f * sumRi));
    }
    float magic{ 0.2 };
    std::vector<Blob> blobbies;
};

template<typename S1, typename S2>
using Union = CSG<unionFunc, S1, S2>;

template<typename S1, typename S2>
using Subtract = CSG<subtractFunc, S1, S2>;

template<typename S1, typename S2>
using Intersect = CSG<intersectionFunc, S1, S2>;

template<typename S1, typename S2>
using Blend = CSG<blendFunc, S1, S2, float>;

template<typename S1, typename S2>
using Mix = CSG<mixFunc, S1, S2, float>;

