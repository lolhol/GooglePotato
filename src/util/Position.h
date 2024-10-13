#ifndef POSITION_H
#define POSITION_H

#include "cartographer/transform/rigid_transform.h"
#include <Eigen/Core>
#include <vector>

class Position {
public:
  Position();
  Position(long time_us, double x_val, double y_val, double theta_val);
  Position(long time_us, double x_val, double y_val, double z_val,
           double pitch_val, double yaw_val, double roll_val);
  Position(long time_us, const cartographer::transform::Rigid3d &pose);

  long timestamp;
  double x;
  double y;
  double z;
  double pitch;
  double yaw;
  double roll;

  operator cartographer::transform::Rigid3d() const;

private:
  static double normalizeTheta(double theta_val);
};

Position operator-(const Position &a, const Position &b);
Position operator+(const Position &a, const Position &b);
Position operator*(const Position &a, const Position &b);
Position operator/(const Position &p, const Position &a);
bool operator==(const Position &a, const Position &b);

#endif // POSITION_H