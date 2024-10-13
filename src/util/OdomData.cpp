#include "OdomData.h"

#include "./DataTransform.h"
#include "cartographer/sensor/odometry_data.h"

// Constructor for full 3D position and quaternion
OdomData::OdomData(SensorIdentity sensorIdentity, double x, double y, double z,
                   float q[])
    : sensorIdentity(sensorIdentity), x(x), y(y), z(z), q_x(q[0]), q_y(q[1]),
      q_z(q[2]), q_w(q[3]) {}

// Constructor for 2D position (z = 0) and quaternion
OdomData::OdomData(SensorIdentity sensorIdentity, double x, double y, float q[])
    : OdomData(sensorIdentity, x, y, 0.0, q) {}

// Function to convert OdomData to Cartographer OdometryData format
cartographer::sensor::OdometryData OdomData::toCarto() {
  cartographer::transform::Rigid3d pose = cartographer::transform::Rigid3d(
      Eigen::Vector3d(x, y, z), Eigen::Quaterniond(q_x, q_y, q_z, q_w));
  return {toCartoTime(sensorIdentity.timeUS), pose};
}