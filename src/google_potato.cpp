//
// Created by Denis Koterov on 10/12/24.
//

#include "google_potato.h"

#include "util/DataTransform.h"
#include "util/ImuData.h"
#include "util/OdomData.h"
#include "util/PointCloud.h"
#include "util/Position.h"
#include "util/sensor/ImuSensor.h"
#include "util/sensor/LidarSensor.h"
#include "util/sensor/Odom.h"
#include <cartographer/common/configuration_file_resolver.h>
#include <cartographer/common/lua_parameter_dictionary.h>
#include <string>

GooglePotato::GooglePotato(std::string configDir, std::string mainConfigFile,
                           PoseUpdateCallback callback,
                           std::vector<LidarSensor> lidars,
                           std::vector<Odom> odom,
                           std::vector<ImuSensor> imuSensors)
    : poseUpdateCallback(poseUpdateCallback),
      startTime(std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count()),
      lidars(lidars), odom(odom), imuSensors(imuSensors) {
  auto file_resolver =
      absl::make_unique<cartographer::common::ConfigurationFileResolver>(
          std::vector<std::string>{configDir});
  const std::string code = file_resolver->GetFileContentOrDie(mainConfigFile);

  cartographer::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));
  mapBuilderOptions = cartographer::mapping::CreateMapBuilderOptions(
      lua_parameter_dictionary.GetDictionary("map_builder").get());
  trajectoryBuilderOptions =
      cartographer::mapping::CreateTrajectoryBuilderOptions(
          lua_parameter_dictionary.GetDictionary("trajectory_builder").get());

  mapBuilder =
      absl::make_unique<cartographer::mapping::MapBuilder>(mapBuilderOptions);

  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  using SensorType = SensorId::SensorType;
  std::set<SensorId> sensor_ids;

  for (const auto &lidar : lidars) {
    sensor_ids.insert(SensorId{SensorType::RANGE, lidar.name});
  }

  for (const auto &imu : imuSensors) {
    sensor_ids.insert(SensorId{SensorType::IMU, imu.name});
  }

  for (const auto &odom : odom) {
    sensor_ids.insert(SensorId{SensorType::ODOMETRY, odom.name});
  }

  trajectoryId = mapBuilder->AddTrajectoryBuilder(
      sensor_ids, trajectoryBuilderOptions,
      [this](auto id, auto time, auto local_pose, auto range_data_in_local,
             std::unique_ptr<const cartographer::mapping::
                                 TrajectoryBuilderInterface::InsertionResult>
                 res) {
        mapBuilder->pose_graph()->GetTrajectoryNodes();
        cartographer::transform::Rigid3d local2global =
            mapBuilder->pose_graph()->GetLocalToGlobalTransform(trajectoryId);
        cartographer::transform::Rigid3d pose3d = local2global * local_pose;
        this->poseUpdateCallback(Position(fromCartoTime(time), pose3d));
      });

  trajectoryBuilder = mapBuilder->GetTrajectoryBuilder(trajectoryId);
  if (!trajectoryBuilder) {
    std::cout << "Get Trajectory Builder Failed" << std::endl;
    LOG(ERROR) << "Get Trajectory Builder Failed";
  }

  pthread_mutex_init(&mutex, nullptr);
  pthread_mutex_init(&sensorMutex, nullptr);
}

void GooglePotato::stopAndOptimize() {
  pthread_mutex_lock(&mutex);
  if (trajectoryId < 0) {
    pthread_mutex_unlock(&mutex);
    return;
  }

  mapBuilder->FinishTrajectory(trajectoryId);
  trajectoryId = -1;
  usleep(1000000);
  mapBuilder->pose_graph()->RunFinalOptimization();
  pthread_mutex_unlock(&mutex);
}

int GooglePotato::handleImuData(ImuData data) {
  auto has = false;
  for (auto cur : imuSensors) {
    if (cur.name == data.sensorIdentity.name) {
      has = true;
    }
  }

  if (!has) {
    return 1;
  }

  pthread_mutex_lock(&mutex);
  if (trajectoryId < 0) {
    pthread_mutex_unlock(&mutex);
    return 1;
  }

  pthread_mutex_lock(&sensorMutex);
  if (latestSensorTimestamp < 0 ||
      latestSensorTimestamp < data.sensorIdentity.timeUS) {
    trajectoryBuilder->AddSensorData(data.sensorIdentity.name,
                                     data.toCartoImu(startTime));
    latestSensorTimestamp = data.sensorIdentity.timeUS;
  }
  pthread_mutex_unlock(&sensorMutex);
  pthread_mutex_unlock(&mutex);
  return 0;
}

int GooglePotato::handleLidarData(PointCloud data) {
  pthread_mutex_lock(&mutex);
  if (trajectoryId < 0) {
    pthread_mutex_unlock(&mutex);
    return 1;
  }

  auto it =
      std::find_if(lidars.begin(), lidars.end(), [&](const LidarSensor &cur) {
        return cur.name == data.sensorIdentity.name;
      });

  if (it == lidars.end()) {
    pthread_mutex_unlock(&mutex);
    return 1;
  }

  LidarSensor sensor = *it;

  // LOG(INFO)<<"Add range data at timestamp " << data.timestamp << " point
  // count " << data.points.size(); radar_scan_time, startTime
  pthread_mutex_lock(&sensorMutex);
  if (latestSensorTimestamp < 0 ||
      latestSensorTimestamp < data.sensorIdentity.timeUS) {
    auto cartoDat = data.toTimedPointCloudData(sensor.scanTimeHz, sensor);
    trajectoryBuilder->AddSensorData(data.sensorIdentity.name, cartoDat);
    latestSensorTimestamp = data.sensorIdentity.timeUS;
  }

  pthread_mutex_unlock(&sensorMutex);
  pthread_mutex_unlock(&mutex);
  return 0;
}

int GooglePotato::handleOdomData(OdomData data) {

  pthread_mutex_lock(&mutex);
  if (trajectoryId < 0) {
    pthread_mutex_unlock(&mutex);
    return 1;
  }

  auto it = std::find_if(odom.begin(), odom.end(), [&](const Odom &cur) {
    return cur.name == data.sensorIdentity.name;
  });

  if (it == odom.end()) {
    pthread_mutex_unlock(&mutex);
    return 1;
  }

  Odom odom = *it;

  pthread_mutex_lock(&sensorMutex);
  if (latestSensorTimestamp < 0 ||
      latestSensorTimestamp < data.sensorIdentity.timeUS) {
    trajectoryBuilder->AddSensorData(data.sensorIdentity.name, data.toCarto());
    latestSensorTimestamp = data.sensorIdentity.timeUS;
  }
  pthread_mutex_unlock(&sensorMutex);
  pthread_mutex_unlock(&mutex);
  return 0;
}