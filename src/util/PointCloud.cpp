#include "PointCloud.h"

#include "cartographer/sensor/timed_point_cloud_data.h"
#include "sensor/ImuSensor.h"
#include "sensor/LidarSensor.h"
#include <Eigen/Core>
#include <vector>

PointCloud::PointCloud(SensorIdentity sensorIdentity)
    : sensorIdentity(sensorIdentity) {}

PointCloud::PointCloud(int size, SensorIdentity sensorIdentity)
    : sensorIdentity(sensorIdentity) {
  points.reserve(size);
  intensities.reserve(size);
}

void PointCloud::add_point(double x, double y, double z) {
  this->add_point(x, y, z, 0.25);
}

void PointCloud::add_point(double x, double y, double z, double intensity) {
  points.push_back(Eigen::Vector3d(x, y, z));
  intensities.push_back(static_cast<float>(intensity));
}

cartographer::sensor::TimedPointCloudData
PointCloud::toTimedPointCloudData(int64_t startTime, LidarSensor sensor) {
  double timeSec =
      static_cast<double>((sensorIdentity.timeUS - startTime)) / 1000.;

  cartographer::common::Time time = cartographer::common::FromUniversal(123) +
                                    cartographer::common::FromSeconds(timeSec);

  cartographer::sensor::TimedPointCloud carto_data;
  int size = points.size();
  carto_data.reserve(size);
  double timestep = sensor.scanTimeHz / size;
  float total_time = 0;

  for (int i = 0; i < size; i++) {
    cartographer::sensor::TimedRangefinderPoint tmp_point{
        Eigen::Vector3f(points.at(i)(0), points.at(i)(1), points.at(i)(2)),
        total_time};

    carto_data.push_back(tmp_point);
    total_time += timestep;
  }

  return cartographer::sensor::TimedPointCloudData{
      time, Eigen::Vector3f(sensor.x, sensor.y, sensor.z), carto_data,
      intensities};
}