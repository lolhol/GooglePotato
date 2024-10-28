//
// Created by Denis Koterov on 10/12/24.
//

#ifndef GOOGLE_POTATO_H
#define GOOGLE_POTATO_H

#include "util/ImuData.h"
#include "util/OdomData.h"
#include "util/PointCloud.h"
#include "util/Position.h"
#include "util/map/Map.h"
#include "util/sensor/ImuSensor.h"
#include "util/sensor/LidarSensor.h"
#include "util/sensor/Odom.h"
#include <Eigen/src/Core/Matrix.h>
#include <cartographer/common/fixed_ratio_sampler.h>
#include <cartographer/common/time.h>
#include <cartographer/io/submap_painter.h>
#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/pose_extrapolator.h>
#include <functional>
#include <string>

#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <vector>

using PoseUpdateCallback = std::function<void(const Position &)>;

class GooglePotato {
public:
  GooglePotato(std::string configDir, std::string mainConfigFile,
               PoseUpdateCallback callback, std::vector<LidarSensor> lidars,
               std::vector<Odom> odom, std::vector<ImuSensor> imuSensors);

  int handleLidarData(PointCloud data);
  int handleImuData(ImuData data);
  int handleOdomData(OdomData data);

  std::vector<std::vector<float>> getMapPointsLowRes();
  std::vector<std::vector<float>> getMapPointsHighRes();
  std::vector<std::vector<float>> getMapPointsGravityAligned();

  std::tuple<float, std::vector<std::vector<float>>>
  getObstructedWallPoints(double maximumIntensity, double minimumAlpha);

  void stopAndOptimize();

private:
  std::vector<LidarSensor> lidars;
  std::vector<Odom> odom;
  std::vector<ImuSensor> imuSensors;

  PoseUpdateCallback poseUpdateCallback;

  double radarScanTime = 1000000;
  long latestSensorTimestamp = -1;

  int64_t startTime = 0;

  pthread_mutex_t sensorMutex;
  pthread_mutex_t mutex;

  std::unique_ptr<cartographer::mapping::MapBuilderInterface> mapBuilder;
  cartographer::mapping::TrajectoryBuilderInterface *trajectoryBuilder;
  cartographer::mapping::proto::MapBuilderOptions mapBuilderOptions;
  cartographer::mapping::proto::TrajectoryBuilderOptions
      trajectoryBuilderOptions;
  int trajectoryId;
};

#endif // GOOGLE_POTATO_H
