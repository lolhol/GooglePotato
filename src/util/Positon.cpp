#include "DataTransform.h"
#include "Position.h"
#include <algorithm>
#include <cmath>
#include <vector>

Position::Position() : Position(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) {}

Position::Position(long time_us, double x_val, double y_val, double theta_val)
    : timestamp(time_us), x(x_val), y(y_val), z(0.0), pitch(0.0),
      yaw(normalizeTheta(theta_val)), roll(0.0) {}

Position::Position(long time_us, double x_val, double y_val, double z_val,
                   double pitch_val, double yaw_val, double roll_val)
    : timestamp(time_us), x(x_val), y(y_val), z(z_val), pitch(pitch_val),
      yaw(normalizeTheta(yaw_val)), roll(roll_val) {}

Position::Position(long time_us, const cartographer::transform::Rigid3d &pose)
    : timestamp(time_us) {
  Eigen::Matrix<double, 3, 1> trans = pose.translation();
  Eigen::Matrix3d rotation_matrix = pose.rotation().matrix();

  this->x = trans.x();
  this->y = trans.y();
  this->z = trans.z();

  double x_rot = rotation_matrix(0);
  double y_rot = rotation_matrix(1);
  double z_rot = rotation_matrix(2);
  double w_rot = rotation_matrix(3);

  this->pitch = fromXYZWToPitch(x_rot, y_rot, z_rot, w_rot);
  this->yaw = fromXYZWToYaw(x_rot, y_rot, z_rot, w_rot);
  this->roll = fromXYZWToRoll(x_rot, y_rot, z_rot, w_rot);
}

// Conversion operator to Rigid3d
Position::operator cartographer::transform::Rigid3d() const {
  Eigen::Matrix<double, 3, 1> trans(x, y, z);
  std::vector<double> xyzw = fromRPYToXYZW(yaw, pitch, roll);
  Eigen::Quaterniond quaternion(xyzw.at(3), xyzw.at(0), xyzw.at(1), xyzw.at(2));
  return cartographer::transform::Rigid3d{trans, quaternion};
}

// Normalize theta (yaw) to [0, 2*PI]
double Position::normalizeTheta(double theta_val) {
  while (theta_val < 0) {
    theta_val += 2 * M_PI;
  }
  while (theta_val > 2 * M_PI) {
    theta_val -= 2 * M_PI;
  }
  return theta_val;
}

// Operator overloads
Position operator-(const Position &a, const Position &b) {
  return {std::max(a.timestamp, b.timestamp),
          a.x - b.x,
          a.y - b.y,
          a.z - b.z,
          a.pitch - b.pitch,
          a.yaw - b.yaw,
          a.roll - b.roll};
}

Position operator+(const Position &a, const Position &b) {
  return {std::max(a.timestamp, b.timestamp),
          a.x + b.x,
          a.y + b.y,
          a.z + b.z,
          a.pitch + b.pitch,
          a.yaw + b.yaw,
          a.roll + b.roll};
}

Position operator*(const Position &a, const Position &b) {
  long timestamp = std::max(a.timestamp, b.timestamp);

  double cos_yaw_a = std::cos(a.yaw);
  double sin_yaw_a = std::sin(a.yaw);
  double cos_pitch_a = std::cos(a.pitch);
  double sin_pitch_a = std::sin(a.pitch);
  double cos_roll_a = std::cos(a.roll);
  double sin_roll_a = std::sin(a.roll);

  double x = b.x * (cos_yaw_a * cos_pitch_a) - b.y * (sin_yaw_a * cos_pitch_a) +
             b.z * sin_pitch_a + a.x;
  double y =
      b.x * (cos_yaw_a * sin_pitch_a * sin_roll_a - sin_yaw_a * cos_roll_a) +
      b.y * (cos_yaw_a * cos_roll_a + sin_yaw_a * sin_pitch_a * sin_roll_a) +
      b.z * (-sin_yaw_a * sin_roll_a + cos_yaw_a * sin_pitch_a) + a.y;
  double z =
      b.x * (sin_yaw_a * sin_pitch_a * cos_roll_a + cos_yaw_a * sin_roll_a) +
      b.y * (sin_yaw_a * sin_pitch_a * sin_roll_a - cos_yaw_a * cos_roll_a) +
      b.z * (cos_yaw_a * cos_pitch_a) + a.z;

  return {timestamp,      x, y, z, a.pitch + b.pitch, a.yaw + b.yaw,
          a.roll + b.roll};
}

Position operator/(const Position &p, const Position &a) {
  return {p.timestamp,       p.x / a.x,     p.y / a.y,      p.z / a.z,
          p.pitch / a.pitch, p.yaw / a.yaw, p.roll / a.roll};
}

bool operator==(const Position &a, const Position &b) {
  return a.x == b.x && a.y == b.y && a.z == b.z && a.pitch == b.pitch &&
         a.yaw == b.yaw && a.roll == b.roll;
}