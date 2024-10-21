#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include "SensorIdentity.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "sensor/LidarSensor.h"
#include <Eigen/Core>
#include <vector>

class PointCloud {
public:
  PointCloud(SensorIdentity sensorIdentity);
  PointCloud(int size, SensorIdentity sensorIdentity);
  PointCloud(std::vector<Eigen::Vector3d> points, SensorIdentity sensorIdentity,
             std::vector<float> intensities);

  void add_point(double x, double y, double z);
  void add_point(double x, double y);
  void add_point(double x, double y, double z, double intensity);

  cartographer::sensor::TimedPointCloudData
  toTimedPointCloudData(int64_t startTime, LidarSensor sensor);

  SensorIdentity sensorIdentity;
  std::vector<Eigen::Vector3d> points;
  std::vector<float> intensities;
};

#endif // POINT_CLOUD_H