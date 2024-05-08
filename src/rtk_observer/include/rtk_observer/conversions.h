#pragma once

#include <array>

namespace utils {

float g2m_s2(float g);

float deg2rad(float deg);

struct Quaternion {
  float w;
  float x;
  float y;
  float z;
};

struct RPY {
  float roll;
  float pitch;
  float yaw;
};

RPY quaternion2Euler(const Quaternion& q);

struct LLA {
  LLA() {
    lat = lon = alt = 0;
  }
  LLA(double lat_, double lon_, double alt_) {
    lat = lat_;
    lon = lon_;
    alt = alt_;
  }
  double lat;
  double lon;
  double alt;
};

struct ENU {
  ENU() {x = y = z = 0;}
  ENU(float x_, float y_, float z_) {
    x = x_;
    y = y_;
    z = z_;
  }
  float x;
  float y;
  float z;
};

struct LocalFrame {
  void fixReferenceCoord(const LLA& reference_point);

  ENU operator()(const LLA& global) const;

 private:
  std::array<double, 3> ecef0{};
  std::array<double, 9> R_ecef2enu{};
};

}  // namespace utils
