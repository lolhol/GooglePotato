//
// Created by ubuntu on 3/22/24.
//

#include "DataTransform.h"
#include "Position.h"
#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/transform/transform.h"
#include <cartographer/common/time.h>
#include <cmath>

double fromXYZWToYaw(double x, double y, double z, double w) {
  return std::atan2(2. * (w * z + x * y), 1. - 2. * (y * y + z * z));
}

double fromXYZWToPitch(double x, double y, double z, double w) {
  return std::asin(2. * (w * y - z * x));
}

double fromXYZWToRoll(double x, double y, double z, double w) {
  return std::atan2(2. * (w * x + y * z), 1. - 2. * (x * x + y * y));
}

long fromCartoTime(cartographer::common::Time time) {
  return (cartographer::common::ToUniversal(time) / 10);
}

std::vector<double> fromRPYToXYZW(double roll, double pitch, double yaw) {
  std::vector<double> result(4);
  result[0] = std::cos(yaw) * std::cos(pitch);
  result[1] = std::cos(yaw) * std::sin(pitch) * std::sin(roll) -
              std::sin(yaw) * std::cos(roll);
  result[2] = std::cos(yaw) * std::sin(pitch) * std::cos(roll) +
              std::sin(yaw) * std::sin(roll);
  result[3] = std::sin(yaw) * std::cos(pitch);
  return result;
}

cartographer::common::Time toCartoTime(double timestampMs) {
  return cartographer::common::FromUniversal(123) + cartographer::common::FromMilliseconds(timestampMs);
}

double fromHzToSec(double hz) { return (1000.0 / hz); }