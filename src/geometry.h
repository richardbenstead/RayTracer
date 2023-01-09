#include <cmath>
#include <iostream>

template <typename T> class Vec2 {
public:
  Vec2() : x(0), y(0) {}
  Vec2(T xx) : x(xx), y(xx) {}
  Vec2(T xx, T yy) : x(xx), y(yy) {}
  Vec2 operator+(const Vec2 &v) const { return Vec2(x + v.x, y + v.y); }
  Vec2 operator/(const T &r) const { return Vec2(x / r, y / r); }
  Vec2 operator*(const T &r) const { return Vec2(x * r, y * r); }
  Vec2 &operator/=(const T &r) {
    x /= r, y /= r;
    return *this;
  }
  Vec2 &operator*=(const T &r) {
    x *= r, y *= r;
    return *this;
  }
  friend std::ostream &operator<<(std::ostream &s, const Vec2<T> &v) {
    return s << '[' << v.x << ' ' << v.y << ']';
  }
  friend Vec2 operator*(const T &r, const Vec2<T> &v) {
    return Vec2(v.x * r, v.y * r);
  }
  T x, y;
};

typedef Vec2<float> Vec2f;
typedef Vec2<int> Vec2i;

// Implementation of a generic vector class - it will be used to deal with 3D
// points, vectors and normals. The class is implemented as a template. While it
// may complicate the code a bit, it gives us the flexibility later, to
// specialize the type of the coordinates into anything we want. For example:
// Vec3f if we want the coordinates to be floats or Vec3i if we want the
// coordinates to be integers.
//
// Vec3 is a standard/common way of naming vectors, points, etc. The OpenEXR and
// Autodesk libraries use this convention for instance.
template <typename T> class Vec3 {
public:
  Vec3() : x(T(0)), y(T(0)), z(T(0)) {}
  Vec3(T xx) : x(xx), y(xx), z(xx) {}
  Vec3(T xx, T yy, T zz) : x(xx), y(yy), z(zz) {}

  // const operators
  Vec3 operator+(const Vec3 &v) const {
    return Vec3(x + v.x, y + v.y, z + v.z);
  }
  Vec3 operator-(const Vec3 &v) const {
    return Vec3(x - v.x, y - v.y, z - v.z);
  }
  Vec3 operator-() const { return Vec3(-x, -y, -z); }
  Vec3 operator*(const T &r) const { return Vec3(x * r, y * r, z * r); }
  Vec3 operator*(const Vec3 &v) const {
    return Vec3(x * v.x, y * v.y, z * v.z);
  }
  T dotProduct(const Vec3<T> &v) const { return x * v.x + y * v.y + z * v.z; }
  Vec3 operator/(const T &r) const { return Vec3(x / r, y / r, z / r); }

  Vec3 &operator/=(const T &r) {
    x /= r, y /= r, z /= r;
    return *this;
  }
  Vec3 &operator*=(const T &r) {
    x *= r, y *= r, z *= r;
    return *this;
  }
  Vec3 &operator+=(const Vec3 &v) {
    x += v.x, y += v.y, z += v.z;
    return *this;
  }

  Vec3 crossProduct(const Vec3<T> &v) const {
    return Vec3<T>(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
  }
  inline T norm() const { return x * x + y * y + z * z; }
  inline T length() const { return sqrt(norm()); }

  const T &operator[](uint8_t i) const { return (&x)[i]; }
  T &operator[](uint8_t i) { return (&x)[i]; }
  Vec3 &normalize() {
    const T n = norm();
    const T dist = sqrt(n + 1e-20);
    const T factor = 1 / dist;
    x *= factor, y *= factor, z *= factor;
    return *this;
  }

  T makeNormed() {
    const T n = norm();
    const T dist = sqrt(n + 1e-20);
    const T factor = 1 / dist;
    x *= factor, y *= factor, z *= factor;
    return dist;
  }

  friend Vec3 operator*(const T &r, const Vec3 &v) {
    return Vec3<T>(v.x * r, v.y * r, v.z * r);
  }
  friend Vec3 operator/(const T &r, const Vec3 &v) {
    return Vec3<T>(r / v.x, r / v.y, r / v.z);
  }

  friend std::ostream &operator<<(std::ostream &s, const Vec3<T> &v) {
    return s << '[' << v.x << ' ' << v.y << ' ' << v.z << ']';
  }

  T x, y, z;
};

// Now you can specialize the class. We are just showing two examples here. In
// your code you can declare a vector either that way: Vec3<float> a, or that
// way: Vec3f a
typedef Vec3<float> Vec3f;
typedef Vec3<int> Vec3i;

// Implementation of a generic 4x4 Matrix class - Same thing here than with the
// Vec3 class. It uses a template which is maybe less useful than with vectors
// but it can be used to define the coefficients of the matrix to be either
// floats (the most case) or doubles depending on our needs.
template <typename T> class Matrix44 {
public:
  T x[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
  static const Matrix44 kIdentity;

  Matrix44() {}

  Matrix44(T a, T b, T c, T d, T e, T f, T g, T h, T i, T j, T k, T l, T m, T n,
           T o, T p) {
    x[0][0] = a;
    x[0][1] = b;
    x[0][2] = c;
    x[0][3] = d;
    x[1][0] = e;
    x[1][1] = f;
    x[1][2] = g;
    x[1][3] = h;
    x[2][0] = i;
    x[2][1] = j;
    x[2][2] = k;
    x[2][3] = l;
    x[3][0] = m;
    x[3][1] = n;
    x[3][2] = o;
    x[3][3] = p;
  }

  const T *operator[](uint8_t i) const { return x[i]; }
  T *operator[](uint8_t i) { return x[i]; }

  // \brief return a transposed copy of the current matrix as a new matrix
  Matrix44 transposed() const {
    return Matrix44(x[0][0], x[1][0], x[2][0], x[3][0], x[0][1], x[1][1],
                    x[2][1], x[3][1], x[0][2], x[1][2], x[2][2], x[3][2],
                    x[0][3], x[1][3], x[2][3], x[3][3]);
  }

  // \brief transpose itself
  Matrix44 &transpose() {
    Matrix44 tmp(x[0][0], x[1][0], x[2][0], x[3][0], x[0][1], x[1][1], x[2][1],
                 x[3][1], x[0][2], x[1][2], x[2][2], x[3][2], x[0][3], x[1][3],
                 x[2][3], x[3][3]);
    *this = tmp;

    return *this;
  }

  // This method needs to be used for point-matrix multiplication. Keep in mind
  // we don't make the distinction between points and vectors at least from
  // a programming point of view, as both (as well as normals) are declared as
  // Vec3. However, mathematically they need to be treated differently. Points
  // can be translated when translation for vectors is meaningless. Furthermore,
  // points are implicitly be considered as having homogeneous coordinates. Thus
  // the w coordinates needs to be computed and to convert the coordinates from
  // homogeneous back to Cartesian coordinates, we need to divided x, y z by w.
  //
  // The coordinate w is more often than not equals to 1, but it can be
  // different than 1 especially when the matrix is projective matrix
  // (perspective projection matrix).
  template <typename S> inline Vec3<S> multVecMatrix(const Vec3<S> &src) const {
    S a, b, c, w;

    a = src[0] * x[0][0] + src[1] * x[1][0] + src[2] * x[2][0] + x[3][0];
    b = src[0] * x[0][1] + src[1] * x[1][1] + src[2] * x[2][1] + x[3][1];
    c = src[0] * x[0][2] + src[1] * x[1][2] + src[2] * x[2][2] + x[3][2];
    w = src[0] * x[0][3] + src[1] * x[1][3] + src[2] * x[2][3] + x[3][3];

    return Vec3<S>{a / w, b / w, c / w};
  }

  // This method needs to be used for vector-matrix multiplication. Look at the
  // differences with the previous method (to compute a point-matrix
  // multiplication). We don't use the coefficients in the matrix that account
  // for translation (x[3][0], x[3][1], x[3][2]) and we don't compute w.
  template <typename S> inline Vec3<S> multDirMatrix(const Vec3<S> &src) const {
    return Vec3<S>{src[0] * x[0][0] + src[1] * x[1][0] + src[2] * x[2][0],
                   src[0] * x[0][1] + src[1] * x[1][1] + src[2] * x[2][1],
                   src[0] * x[0][2] + src[1] * x[1][2] + src[2] * x[2][2]};
  }
};

typedef Matrix44<float> Matrix44f;
