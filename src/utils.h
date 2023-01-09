#pragma once
#include "tuple"
#include <array>
#include <cmath>

struct XYPair {
  XYPair operator*(const float f) const { return XYPair{x * f, y * f}; }
  XYPair operator-(const XYPair &xy) const {
    return XYPair{x - xy.x, y - xy.y};
  }
  XYPair operator+(const XYPair &xy) const {
    return XYPair{x + xy.x, y + xy.y};
  }
  XYPair operator+=(const XYPair &xy) {
    x += xy.x;
    y += xy.y;
    return *this;
  }
  float norm() const { return sqrt(x * x + y * y); }

  float x{}, y{};
};

struct Pixel {
  Pixel operator*(const float f) const { return Pixel{r * f, g * f, b * f}; }
  Pixel operator+(const Pixel &pix) const {
    return Pixel{r + pix.r, g + pix.g, b + pix.b};
  }
  Pixel operator+=(const Pixel &pix) {
    r += pix.r;
    g += pix.g;
    b += pix.b;
    return *this;
  }
  float r{}, g{}, b{};
};

template <int16_t GS> class Image {
public:
  static constexpr int16_t GRID_SIZE{GS};
  static constexpr int32_t ARR_SIZE{GS * GS};

  constexpr static int32_t POS(int16_t i, int16_t j) {
    return i + GRID_SIZE * j;
  };
  std::array<Pixel, ARR_SIZE> image{};
};

template <typename Image> class Frame {
public:
  static constexpr int16_t VIEW_WIDTH{Image::GRID_SIZE};
  static constexpr int16_t VIEW_HEIGHT{Image::GRID_SIZE};

  void reset() {
    mCentre = XYPair{0, 0};
    mScale = XYPair(5, 5);
  }

  XYPair imageToWorld(int16_t x, int16_t y) {
    double wx =
        (static_cast<double>(x) / static_cast<double>(VIEW_WIDTH) - 0.5) *
            mScale.x +
        mCentre.x;
    double wy =
        (static_cast<double>(y) / static_cast<double>(VIEW_WIDTH) - 0.5) *
            mScale.y +
        mCentre.y;
    return XYPair(wx, wy);
  }

  XYPair mCentre{};
  XYPair mScale{5, 5};
};
