#ifndef DATA_TRANSFORM_H
#define DATA_TRANSFORM_H

#include "cartographer/sensor/imu_data.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "cartographer/transform/transform.h"
#include <cmath>
#include <vector>

// Function to convert quaternion (x, y, z, w) to yaw (Z-axis rotation)
double fromXYZWToYaw(double x, double y, double z, double w);

// Function to convert quaternion (x, y, z, w) to pitch (Y-axis rotation)
double fromXYZWToPitch(double x, double y, double z, double w);

// Function to convert quaternion (x, y, z, w) to roll (X-axis rotation)
double fromXYZWToRoll(double x, double y, double z, double w);

// Function to convert roll, pitch, and yaw to quaternion (x, y, z, w)
std::vector<double> fromRPYToXYZW(double roll, double pitch, double yaw);

// Function to convert universal time in microseconds to Cartographer time
cartographer::common::Time toCartoTime(double timestamp_us);

long fromCartoTime(cartographer::common::Time time);

// Function to convert frequency (Hz) to time in milliseconds
double fromHzToSec(double hz);

#endif // DATA_TRANSFORM_H