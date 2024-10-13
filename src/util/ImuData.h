#ifndef IMU_DATA_H
#define IMU_DATA_H

#include "SensorIdentity.h"
#include "cartographer/sensor/imu_data.h"
#include <Eigen/Core>
#include <vector>

class ImuData {
public:
  ImuData(SensorIdentity sensorIdentity, double linear_acceleration_x,
          double linear_acceleration_y, double linear_acceleration_z,
          double angular_velocity_x, double angular_velocity_y,
          double angular_velocity_z);

  ImuData(SensorIdentity sensorIdentity, float linear_accelerations[],
          float quaternion[], float angular_velocities[]);

  cartographer::sensor::ImuData toCartoImu(int64_t startTime);

  SensorIdentity sensorIdentity;

private:
  // Linear acceleration
  float linear_acceleration_x;
  float linear_acceleration_y;
  float linear_acceleration_z;

  // Angular velocity
  float angular_velocity_x;
  float angular_velocity_y;
  float angular_velocity_z;
};

#endif // IMU_DATA_H