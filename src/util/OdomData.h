#ifndef ODOM_DATA_H
#define ODOM_DATA_H

#include "SensorIdentity.h"
#include "cartographer/sensor/odometry_data.h"
#include <Eigen/Core>

class OdomData {
public:
  // Constructor for full 3D position and quaternion
  OdomData(SensorIdentity sensorIdentity, double x, double y, double z,
           float q[]);

  // Constructor for 2D position (z = 0) and quaternion
  OdomData(SensorIdentity sensorIdentity, double x, double y, float q[]);

  // Function to convert OdomData to Cartographer OdometryData
  cartographer::sensor::OdometryData toCarto();

  SensorIdentity sensorIdentity;

private:
  double x;
  double y;
  double z;

  float q_x;
  float q_y;
  float q_z;
  float q_w;
};

#endif // ODOM_DATA_H