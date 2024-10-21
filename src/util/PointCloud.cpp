#include "PointCloud.h"

#include "SensorIdentity.h"
#include "cartographer/sensor/timed_point_cloud_data.h"
#include "sensor/ImuSensor.h"
#include "sensor/LidarSensor.h"
#include <Eigen/Core>
#include <cartographer/common/time.h>
#include <vector>

PointCloud::PointCloud(SensorIdentity sensorIdentity)
    : sensorIdentity(sensorIdentity) {}

PointCloud::PointCloud(int size, SensorIdentity sensorIdentity)
    : sensorIdentity(sensorIdentity) {
  points.reserve(size);
  intensities.reserve(size);
}

PointCloud::PointCloud(std::vector<Eigen::Vector3d> points,
                       SensorIdentity sensorIdentity,
                       std::vector<float> intensities)
    : sensorIdentity(sensorIdentity), points(points), intensities(intensities) {
}

void PointCloud::add_point(double x, double y, double z) {
  this->add_point(x, y, z, 0.25);
}

void PointCloud::add_point(double x, double y) { this->add_point(x, y, 0.0); }

void PointCloud::add_point(double x, double y, double z, double intensity) {
  points.push_back(Eigen::Vector3d(x, y, z));
  intensities.push_back(static_cast<float>(intensity));
}

cartographer::sensor::TimedPointCloudData
PointCloud::toTimedPointCloudData(int64_t startTime, LidarSensor sensor) {
  double timeSec = sensorIdentity.timeUS + startTime;

  cartographer::common::Time time =
      cartographer::common::FromUniversal(1) +
      cartographer::common::FromMilliseconds(timeSec);

  cartographer::sensor::TimedPointCloud carto_data;
  int size = points.size();
  carto_data.reserve(size);

  // Calculate the timestep between points, assuming scanTimeHz is in Hz (scans
  // per second)
  double timestep = 1.0 / sensor.scanTimeHz; // Time between each scan
  float total_time = 0;

  // Convert each point into a TimedRangefinderPoint and add it to
  // Cartographer's TimedPointCloud
  for (int i = 0; i < size; i++) {
    cartographer::sensor::TimedRangefinderPoint tmp_point{
        Eigen::Vector3f(points.at(i)(0), points.at(i)(1), points.at(i)(2)),
        0.0F};

    carto_data.push_back(tmp_point);
    total_time += timestep;
  }

  // Return TimedPointCloudData including the Lidar position and intensities
  return cartographer::sensor::TimedPointCloudData{
      time, Eigen::Vector3f(sensor.x, sensor.y, sensor.z), carto_data,
      intensities};
}