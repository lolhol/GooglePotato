#include "ImuData.h"

#include "DataTransform.h"
#include "cartographer/sensor/imu_data.h"
#include <Eigen/Core>
#include <vector>

ImuData::ImuData(SensorIdentity sensorIdentity, double linear_acceleration_x,
                 double linear_acceleration_y, double linear_acceleration_z,
                 double angular_velocity_x, double angular_velocity_y,
                 double angular_velocity_z)
    : sensorIdentity(sensorIdentity),
      linear_acceleration_x(linear_acceleration_x),
      linear_acceleration_y(linear_acceleration_y),
      linear_acceleration_z(linear_acceleration_z),
      angular_velocity_x(angular_velocity_x),
      angular_velocity_y(angular_velocity_y),
      angular_velocity_z(angular_velocity_z) {}

// Constructor for arrays
ImuData::ImuData(SensorIdentity sensorIdentity, float linear_accelerations[],
                 float quaternion[], float angular_velocities[])
    : ImuData(sensorIdentity, static_cast<double>(linear_accelerations[0]),
              static_cast<double>(linear_accelerations[1]),
              static_cast<double>(linear_accelerations[2]),
              static_cast<double>(angular_velocities[0]),
              static_cast<double>(angular_velocities[1]),
              static_cast<double>(angular_velocities[2])) {}

cartographer::sensor::ImuData ImuData::toCartoImu(int64_t startTime)
{
    Eigen::Vector3d linear_acceleration(
        linear_acceleration_x, linear_acceleration_y, linear_acceleration_z);

    Eigen::Vector3d angular_velocity(angular_velocity_x, angular_velocity_y,
                                     angular_velocity_z);

    return cartographer::sensor::ImuData{
        toCartoTime(sensorIdentity.timeUS),
        linear_acceleration, angular_velocity};
}