#pragma once
#include "utils.h"
#include <Eigen/Dense>
#include <iostream>
#include <vector>

static constexpr double INTERSECT_TH = 1e-2;
static constexpr double MAX_DISTANCE = 50;

using Vec3d = Eigen::Vector3d;

struct TextureCheck {
  double getVal(const double x, const double y, const double scale = 20) const {
    return ((int8_t(x * scale) ^ int8_t(y * scale)) & 1);
  }
};

// ImplicitShape base class
class ImplicitShape {
public:
  ImplicitShape(const Vec3d &col) : color(col) {}
  virtual ~ImplicitShape() {}

  // default intersection using sphere-trace
  virtual bool intersect(const Vec3d &orig, const Vec3d &dir,
                         double &t) const = 0;

  // Compute the surface data such as normal and texture coordinates at the
  // intersection point.
  virtual void getSurfaceData([[maybe_unused]] const Vec3d &point,
                              Vec3d &normal, Vec3d &col) const = 0;

  // only needed by marching cubes
  virtual double getDistance(const Vec3d &from) const = 0;

  Vec3d color{0.9, 0.2, 0.2};
};

// Implicit sphere surface
template <typename TEX_T = TextureCheck> class Sphere : public ImplicitShape {
public:
  Sphere(const Vec3d &c, const double &r, const Vec3d &col = Vec3d(1, 1, 1))
      : center{c}, radius{r}, radius2{r * r}, ImplicitShape(col) {}

  double getDistance(const Vec3d &from) const {
    return (from - center).norm() - radius;
  }

  bool intersect(const Vec3d &orig, const Vec3d &dir, double &t) const {
    const Vec3d L = center - orig;

    // project L along dir to get distance to point tangental to center
    const double tca = dir.dot(L);
    if (tca >= 0) {
      // square of distance from tangent to center via pythagoras
      const double d2 = L.squaredNorm() - tca * tca;
      if (d2 <= radius2) {
        const double thc = fastSqrt(radius2 - d2);
        t = tca - thc; // first intersect
        return true;
      }
    }
    return false;
  }

  double nearestDistance(const Vec3d &orig, const Vec3d &dir) const {
    const Vec3d L = center - orig;

    // project L along dir to get distance to point tangental to center
    const double tca = dir.dot(L);
    if (tca >= 0) {
      // square of distance from tangent to center via pythagoras
      const double d2 = L.squaredNorm() - tca * tca;
      return std::max(0.0, d2 - radius2);
    }
    return 1e9;
  }

  void getSurfaceData([[maybe_unused]] const Vec3d &point, Vec3d &normal,
                      Vec3d &col) const {
    normal = (point - center).normalized();
    col = color * texture.getVal((1 + atan2(normal[2], normal[0]) / M_PI) * 0.5,
                                 acosf(normal[1]) / M_PI);
  }
  const TEX_T texture{};
  const Vec3d center;
  const double radius, radius2;
};

// Implicit plane surface
template <typename TEX_T = TextureCheck> class Plane : public ImplicitShape {
public:
  Plane(const Vec3d &_normal = Vec3d(0, 1, 0), const Vec3d &pp = Vec3d(0),
        const Vec3d &col = Vec3d(1, 1, 1))
      : ImplicitShape(col), normal(_normal), pointOnPlane(pp) {}

  bool intersect(const Vec3d &orig, const Vec3d &dir, double &t) const {
    // assuming vectors are all normalized
    // (intersect_point (p) - pointOnPlane (p)).dot(normal) = 0
    // intersect_point = (orig + dir * t)
    // => t = (intersect point - orig).dot(normal) / dir.dot(normal)
    const double denom = dir.dot(normal);
    // std::cout << "check plane, denom: " << denom << std::endl;
    if (denom < -1e-6) {
      Vec3d p0l0 = pointOnPlane - orig;
      t = p0l0.dot(normal) / denom;
      return (t > 0);
    }

    return false;
  }

  void getSurfaceData([[maybe_unused]] const Vec3d &point, Vec3d &norm,
                      Vec3d &col) const {
    norm = normal;
    const Vec3d tex = point.cross(normal);
    col = color * texture.getVal(tex[0], tex[2], 2);
  }

  double getDistance(const Vec3d &) const { return 0; }

  const TEX_T texture{};
  const Vec3d normal;
  const Vec3d pointOnPlane;
};

class MarchingCubeImplicitShape : public ImplicitShape {
public:
  MarchingCubeImplicitShape(const Vec3d &col) : ImplicitShape(col) {}
  virtual bool intersect(const Vec3d &orig, const Vec3d &dir, double &t) const {
    double distance = 0;
    for (t = 0; t < MAX_DISTANCE; t += distance) {
      Vec3d point = orig + t * dir;
      distance = getDistance(point);

      if (distance <= INTERSECT_TH * t) {
        [[unlikely]];
        return true;
      }
    }
    return false;
  }

  // Method to compute the surface data such as normal and texture coordinates
  // at the intersection point. See method implementation in children class for
  // details
  virtual void getSurfaceData([[maybe_unused]] const Vec3d &point,
                              Vec3d &normal, Vec3d &col) const {
    constexpr double delta = 10e-5;
    const double base = getDistance(point);
    normal = Vec3d(getDistance(point + Vec3d(delta, 0, 0)) - base,
                   getDistance(point + Vec3d(0, delta, 0)) - base,
                   getDistance(point + Vec3d(0, 0, delta)) - base)
                 .normalized();
    col = color;
  }
};

// Implicit torus surface
class Torus : public MarchingCubeImplicitShape {
public:
  Torus(const Vec3d &_center, const double &_r0, const double &_r1,
        const Vec3d &col = Vec3d(1, 1, 1))
      : MarchingCubeImplicitShape(col), center(_center), r0(_r0), r1(_r1) {}
  virtual double getDistance(const Vec3d &from) const {
    Vec3d pointObjectSpace = from - center;
    // reduce 3D point to 2D point
    const double tmpx = fastSqrt(pointObjectSpace[0] * pointObjectSpace[0] +
                                 pointObjectSpace[2] * pointObjectSpace[2]) -
                        r0;
    const double tmpy = pointObjectSpace[1];

    // distance to cicle
    return fastSqrt(tmpx * tmpx + tmpy * tmpy) - r1;
  }

  const Vec3d center;
  const double r0, r1;
};

// Implicit cube surface
class Cube : public MarchingCubeImplicitShape {
public:
  Cube(const Vec3d &_corner, Vec3d col = Vec3d(1, 1, 1))
      : MarchingCubeImplicitShape(col), corner(_corner) {}
  virtual double getDistance(const Vec3d &point) const {
    constexpr double scale = 1.0;
    constexpr double invScale = 1.0 / scale;

    Vec3d const pointObjectSpace = (point * 1.0 * invScale).cwiseAbs();

    // now compute the distance from the point to the surface of the object
    Vec3d d = pointObjectSpace - corner;

    double dmaxLen =
        fastSqrt(d[0] * d[0] * (d[0] > 0) + d[1] * d[1] * (d[1] > 0) +
                 d[2] * d[2] * (d[2] > 0));

    // don't forget to apply the scale back
    return scale *
           (std::min(std::max(d[0], std::max(d[1], d[2])), 0.0) + dmaxLen);
  }

  const Vec3d corner{};
};

struct unionFunc {
  double operator()(double a, double b) const { return std::min(a, b); }
};

struct subtractFunc {
  double operator()(double a, double b) const { return std::max(-a, b); }
};

struct intersectionFunc {
  double operator()(double a, double b) const { return std::max(a, b); }
};

struct blendFunc {
  blendFunc(const double &_k) : k(_k) {}
  double operator()(double a, double b) const {
    double res = exp(-k * a) + exp(-k * b);
    return -log(std::max(0.0001, res)) / k;
  }
  double k;
};

struct mixFunc {
  mixFunc(const double &_t) : t(_t) {}
  double operator()(double a, double b) const { return a * (1 - t) + b * t; }
  double t;
};

// Combining implict shapes using computational solid geometry
template <typename Op, typename S1, typename S2, typename... Args>
class CSG : public MarchingCubeImplicitShape {
public:
  CSG(const S1 &s1, const S2 &s2, Args &&...args)
      : op(std::forward<Args>(args)...), shape1(s1), shape2(s2) {}
  double getDistance(const Vec3d &from) const {
    return op(shape1.getDistance(from), shape2.getDistance(from));
  }
  Op op;
  const S1 shape1;
  const S2 shape2;
};

// softmin, return smin and blend factor
std::pair<double, double> smin(double a, double b, double k) {
  double h = std::max(k - abs(a - b), 0.0) / k;
  double m = h * h * 0.5;
  double s = m * k * (1.0 / 2.0);
  return (a < b) ? std::pair<double, double>(a - s, m)
                 : std::pair<double, double>(b - s, 1.0 - m);
}

// Blobbies
class SoftObject : public ImplicitShape {
public:
  using SphereT = Sphere<TextureCheck>;
  SoftObject() : ImplicitShape(Vec3d(1, 1, 1)) {
#if 1
    blobbies.push_back(new SphereT(Vec3d(-1, 0, 0), 2.0, Vec3d(1, 0.2, 0.2)));
    blobbies.push_back(new SphereT(Vec3d(1, 0, 0), 1.5, Vec3d(0.2, 1.0, 0.8)));
#else
    for (size_t i = 0; i < 5; ++i) {
      double radius = (0.3 + drand48() * 1.3);
      Vec3d c((0.5 - drand48()) * 3, (0.5 - drand48()) * 3,
              (0.5 - drand48()) * 3);
      blobbies.push_back(new SphereT(c, radius));
    }
#endif
  }
  double getDistance(const Vec3d &) const { return 0; }

  // default intersection using sphere-trace
  bool intersect(const Vec3d &orig, const Vec3d &dir, double &t) const {
    t = std::numeric_limits<double>::max();
    for (const SphereT *blob : blobbies) {
      if (blob->intersect(orig, dir, t)) {
        return true;
      }
      // auto [dist, mix] = smin(t, blob->nearestDistance(orig, dir), blend);
      // t = dist;
    }
    return false;
  }

  // Compute the surface data such as normal and texture coordinates at the
  // intersection point.
  virtual void getSurfaceData([[maybe_unused]] const Vec3d &point,
                              Vec3d &normal, Vec3d &col) const {
    // blobbies[0]->getSurfaceData(point, normal, col);
    // std::cout << col.transpose() << " " << normal.transpose() << std::endl;
    // return;

    Vec3d sumNorm = Vec3d(0, 0, 0);
    Vec3d sumCol = Vec3d(0, 0, 0);
    double totalD2 = 0;
    for (const SphereT *blob : blobbies) {
      Vec3d tmpNorm, tmpCol;
      double d = std::min(1.0, blob->getDistance(point));

      blob->getSurfaceData(point, tmpNorm, tmpCol);
      double wgt = 1.0 / (d * d);
      sumNorm = sumNorm + tmpNorm * wgt;
      sumCol = sumCol + tmpCol * wgt;
      totalD2 += wgt;

      // x2 + y2 + z2 = r2
      //
    }
    normal = sumNorm.normalized();
    col = sumCol.array() / totalD2;
    // std::cout << col.transpose() << " " << normal.transpose() << std::endl;
  }

  double blend = 0.2;
  double magic{0.2};
  std::vector<SphereT *> blobbies;
};

template <typename S1, typename S2> using Union = CSG<unionFunc, S1, S2>;

template <typename S1, typename S2> using Subtract = CSG<subtractFunc, S1, S2>;

template <typename S1, typename S2>
using Intersect = CSG<intersectionFunc, S1, S2>;

template <typename S1, typename S2>
using Blend = CSG<blendFunc, S1, S2, double>;

template <typename S1, typename S2> using Mix = CSG<mixFunc, S1, S2, double>;
