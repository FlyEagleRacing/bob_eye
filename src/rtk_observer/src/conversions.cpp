#include "rtk_observer/conversions.h"

#include <cmath>

namespace utils {

float g2m_s2(float g) { return g * 9.81f; }

float deg2rad(float deg) { return static_cast<float>(deg * M_PI / 180); }

RPY quaternion2Euler(const Quaternion& q) {
  RPY angles;

  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  angles.roll = static_cast<float>(std::atan2(sinr_cosp, cosr_cosp));

  double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
  double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
  angles.pitch = static_cast<float>(2 * std::atan2(sinp, cosp) - M_PI / 2);

  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  angles.yaw = static_cast<float>(std::atan2(siny_cosp, cosy_cosp));

  return angles;
}

void LocalFrame::fixReferenceCoord(const LLA& reference_point) {
  static constexpr double a = 6'378'137.0;
  static constexpr double e2 = 6.69437999014 * 1e-3;

  const double clat = std::cos(reference_point.lat * M_PI / 180.0);
  const double clon = std::cos(reference_point.lon * M_PI / 180.0);
  const double slat = std::sin(reference_point.lat * M_PI / 180.0);
  const double slon = std::sin(reference_point.lon * M_PI / 180.0);

  const double N = a / std::sqrt(1.0 - e2 * (slat * slat));

  ecef0 = {
      (reference_point.alt + N) * clat * clon,
      (reference_point.alt + N) * clat * slon,
      (reference_point.alt + N * (1.0 - e2)) * slat,
  };

  R_ecef2enu = {-slon, clon,        0.0,         -slat * clon, -slat * slon,
                clat,  clat * clon, clat * slon, slat};
}

ENU LocalFrame::operator()(const LLA& global) const {
  static constexpr double a = 6'378'137.0;
  static constexpr double e2 = 6.69437999014 * 1e-3;

  const double clat = std::cos(global.lat * M_PI / 180.0);
  const double clon = std::cos(global.lon * M_PI / 180.0);
  const double slat = std::sin(global.lat * M_PI / 180.0);
  const double slon = std::sin(global.lon * M_PI / 180.0);

  const double N = a / std::sqrt(1.0 - e2 * (slat * slat));

  std::array<double, 3> ecef = {
      (global.alt + N) * clat * clon,
      (global.alt + N) * clat * slon,
      (global.alt + N * (1.0 - e2)) * slat,
  };

  const double ecef_delta_X = ecef[0] - ecef0[0];
  const double ecef_delta_Y = ecef[1] - ecef0[1];
  const double ecef_delta_Z = ecef[2] - ecef0[2];

  return ENU(static_cast<float>(R_ecef2enu[0] * ecef_delta_X +
                            R_ecef2enu[1] * ecef_delta_Y +
                            R_ecef2enu[2] * ecef_delta_Z),
             static_cast<float>(R_ecef2enu[3] * ecef_delta_X +
                            R_ecef2enu[4] * ecef_delta_Y +
                            R_ecef2enu[5] * ecef_delta_Z),
             static_cast<float>(R_ecef2enu[6] * ecef_delta_X +
                            R_ecef2enu[7] * ecef_delta_Y +
                            R_ecef2enu[8] * ecef_delta_Z));
}

}  // namespace utils
