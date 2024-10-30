//
// Created by Denis Koterov on 10/12/24.
//

#include "google_potato.h"

#include "util/DataTransform.h"
#include "util/ImuData.h"
#include "util/OdomData.h"
#include "util/PointCloud.h"
#include "util/Position.h"
#include "util/map/Map.h"
#include "util/map/Pixel.h"
#include "util/sensor/ImuSensor.h"
#include "util/sensor/LidarSensor.h"
#include "util/sensor/Odom.h"
#include <Eigen/src/Core/Matrix.h>
#include <boost/iostreams/filter/gzip.hpp>
#include <cartographer/common/configuration_file_resolver.h>
#include <cartographer/common/lua_parameter_dictionary.h>
#include <cartographer/io/submap_painter.h>
#include <cartographer/mapping/2d/submap_2d.h>
#include <cartographer/mapping/3d/submap_3d.h>
#include <cartographer/mapping/id.h>
#include <cartographer/mapping/proto/pose_graph.pb.h>
#include <cartographer/mapping/proto/submap.pb.h>
#include <cartographer/mapping/submaps.h>
#include <cartographer/transform/transform.h>
#include <map>
#include <pthread.h>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

GooglePotato::GooglePotato(std::string configDir, std::string mainConfigFile,
                           PoseUpdateCallback callback,
                           std::vector<LidarSensor> lidars,
                           std::vector<Odom> odom,
                           std::vector<ImuSensor> imuSensors, bool loggingEnabled)
    : poseUpdateCallback(callback),
      startTime(std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch())
                    .count()),
      lidars(lidars), odom(odom), imuSensors(imuSensors), loggingEnabled(loggingEnabled)
{
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

  for (const auto &lidar : lidars)
  {
    sensor_ids.insert(SensorId{SensorType::RANGE, lidar.name});

    if (loggingEnabled) {
      std::cout << "Found " << lidar.name << " lidar." << std::endl;
      LOG(INFO) << "Found " << lidar.name << " lidar.";
    }
  }

  for (const auto &imu : imuSensors)
  {
    sensor_ids.insert(SensorId{SensorType::IMU, imu.name});

    if (loggingEnabled) {
      std::cout << "Found " << imu.name << " imu." << std::endl;
      LOG(INFO) << "Found " << imu.name << " imu.";
    }
  }

  for (const auto &odom : odom)
  {
    sensor_ids.insert(SensorId{SensorType::ODOMETRY, odom.name});

    if (loggingEnabled) {
      std::cout << "Found " << odom.name << " odom." << std::endl;
      LOG(INFO) << "Found " << odom.name << " odom.";
    }
  }

  trajectoryId = mapBuilder->AddTrajectoryBuilder(
      sensor_ids, trajectoryBuilderOptions,
      [this](auto id, auto time, auto local_pose, auto range_data_in_local,
             std::unique_ptr<const cartographer::mapping::
                                 TrajectoryBuilderInterface::InsertionResult>
                 res)
      {
        mapBuilder->pose_graph()->GetTrajectoryNodes();
        cartographer::transform::Rigid3d local2global =
            mapBuilder->pose_graph()->GetLocalToGlobalTransform(trajectoryId);
        cartographer::transform::Rigid3d pose3d = local2global * local_pose;
        this->poseUpdateCallback(Position(fromCartoTime(time), pose3d));
      });

  trajectoryBuilder = mapBuilder->GetTrajectoryBuilder(trajectoryId);
  if (!trajectoryBuilder)
  {
    std::cout << "Get Trajectory Builder Failed" << std::endl;
    LOG(ERROR) << "Get Trajectory Builder Failed";
  }

  pthread_mutex_init(&mutex, nullptr);
  pthread_mutex_init(&sensorMutex, nullptr);
}

void GooglePotato::stopAndOptimize()
{
  pthread_mutex_lock(&mutex);
  if (trajectoryId < 0)
  {
    if (loggingEnabled) {
      std::cout << "Stopping failed as trajectoryId is undefined" << std::endl;
      LOG(ERROR) << "Stopping failed as trajectoryId is undefined";
    }

    pthread_mutex_unlock(&mutex);
    return;
  }

  mapBuilder->FinishTrajectory(trajectoryId);
  trajectoryId = -1;
  usleep(1000000);
  mapBuilder->pose_graph()->RunFinalOptimization();
  pthread_mutex_unlock(&mutex);
}

int GooglePotato::handleImuData(ImuData data)
{
  auto has = false;
  for (auto cur : imuSensors)
  {
    if (cur.name == data.sensorIdentity.name)
    {
      if (loggingEnabled) {
        std::cout << "Found imu sensor " << cur.name << " when handling imu data." << std::endl;
        LOG(ERROR) << "Found imu sensor " << cur.name << " when handling imu data.";
      }

      has = true;
    }
  }

  if (!has)
  {
    return 1;
  }

  pthread_mutex_lock(&mutex);
  if (trajectoryId < 0)
  {
    pthread_mutex_unlock(&mutex);
    return 1;
  }

  pthread_mutex_lock(&sensorMutex);
  if (latestSensorTimestamp < 0 ||
      latestSensorTimestamp < data.sensorIdentity.timeUS)
  {
    trajectoryBuilder->AddSensorData(data.sensorIdentity.name,
                                     data.toCartoImu(startTime));
    latestSensorTimestamp = data.sensorIdentity.timeUS;
  }

  pthread_mutex_unlock(&sensorMutex);
  pthread_mutex_unlock(&mutex);
  return 0;
}

int GooglePotato::handleLidarData(PointCloud data)
{
  pthread_mutex_lock(&mutex);
  if (trajectoryId < 0)
  {
    pthread_mutex_unlock(&mutex);
    return 1;
  }

  auto it =
      std::find_if(lidars.begin(), lidars.end(), [&](const LidarSensor &cur)
                   { return cur.name == data.sensorIdentity.name; });

  if (it == lidars.end())
  {
    pthread_mutex_unlock(&mutex);
    return 1;
  }

  LidarSensor sensor = *it;

  // LOG(INFO)<<"Add range data at timestamp " << data.timestamp << " point
  // count " << data.points.size(); radar_scan_time, startTime
  pthread_mutex_lock(&sensorMutex);
  if (latestSensorTimestamp < 0 ||
      latestSensorTimestamp < data.sensorIdentity.timeUS)
  {
    auto cartoDat = data.toTimedPointCloudData(sensor.scanTimeHz, sensor);
    trajectoryBuilder->AddSensorData(data.sensorIdentity.name, cartoDat);
    latestSensorTimestamp = data.sensorIdentity.timeUS;
  }

  pthread_mutex_unlock(&sensorMutex);
  pthread_mutex_unlock(&mutex);
  return 0;
}

int GooglePotato::handleOdomData(OdomData data)
{

  pthread_mutex_lock(&mutex);
  if (trajectoryId < 0)
  {
    pthread_mutex_unlock(&mutex);
    return 1;
  }

  auto it = std::find_if(odom.begin(), odom.end(), [&](const Odom &cur)
                         { return cur.name == data.sensorIdentity.name; });

  if (it == odom.end())
  {
    pthread_mutex_unlock(&mutex);
    return 1;
  }

  Odom odom = *it;

  pthread_mutex_lock(&sensorMutex);
  if (latestSensorTimestamp < 0 ||
      latestSensorTimestamp < data.sensorIdentity.timeUS)
  {
    trajectoryBuilder->AddSensorData(data.sensorIdentity.name, data.toCarto());
    latestSensorTimestamp = data.sensorIdentity.timeUS;
  }
  pthread_mutex_unlock(&sensorMutex);
  pthread_mutex_unlock(&mutex);
  return 0;
}

std::vector<std::vector<float>> GooglePotato::getMapPointsHighRes()
{
  pthread_mutex_lock(&mutex);

  std::vector<std::vector<float>> pointCloud;

  for (const auto &tn : mapBuilder->pose_graph()->GetTrajectoryNodes())
  {
    int count = 0;
    const auto intensities =
        tn.data.constant_data->high_resolution_point_cloud.intensities();

    for (const auto &point :
         tn.data.constant_data->high_resolution_point_cloud.points())
    {
      pointCloud.push_back(
          {point.position.x(), point.position.y(), point.position.z(),
           intensities.size() < count ? intensities[count] : 0});

      count++;
    }
  }

  pthread_mutex_unlock(&mutex);
  return pointCloud;
}

std::vector<std::vector<float>> GooglePotato::getMapPointsLowRes()
{
  pthread_mutex_lock(&mutex);

  std::vector<std::vector<float>> pointCloud;

  for (const auto &tn : mapBuilder->pose_graph()->GetTrajectoryNodes())
  {
    int count = 0;
    const auto intensities =
        tn.data.constant_data->low_resolution_point_cloud.intensities();
    const auto points =
        tn.data.constant_data->low_resolution_point_cloud.points();

    for (const auto &point : points)
    {
      pointCloud.push_back(
          {point.position.x(), point.position.y(), point.position.z(),
           intensities.size() < count ? intensities[count] : 0});

      count++;
    }
  }

  pthread_mutex_unlock(&mutex);
  return pointCloud;
}

std::vector<std::vector<float>> GooglePotato::getMapPointsGravityAligned()
{
  pthread_mutex_lock(&mutex);

  std::vector<std::vector<float>> pointCloud;

  for (const auto &tn : mapBuilder->pose_graph()->GetTrajectoryNodes())
  {
    int count = 0;
    const auto intensities =
        tn.data.constant_data->low_resolution_point_cloud.intensities();
    const auto points =
        tn.data.constant_data->filtered_gravity_aligned_point_cloud.points();

    for (const auto &point : points)
    {
      pointCloud.push_back(
          {point.position.x(), point.position.y(), point.position.z(),
           intensities.size() < count ? intensities[count] : 0});

      count++;
    }
  }

  pthread_mutex_unlock(&mutex);
  return pointCloud;
}

/**
   * Intensity values often represent the occupancy probability (how likely a
   cell is to be an obstacle).

   * Alpha values denote the certainty of that information by indicating whether
   the cell was observed.
 */
std::tuple<float, std::vector<std::vector<float>>>
GooglePotato::getObstructedWallPoints(double maximumIntensity,
                                      double minimumAlpha)
{
  pthread_mutex_lock(&mutex);

  std::vector<std::vector<float>> obstructed_points;
  float resolution;
  for (const auto &sub : mapBuilder->pose_graph()->GetAllSubmapPoses())
  {
    cartographer::mapping::SubmapId submapId = sub.id;
    cartographer::transform::Rigid3d pose = sub.data.pose;

    // Retrieve submap texture data.
    cartographer::mapping::proto::SubmapQuery::Response responseProto;
    const std::string error =
        mapBuilder->SubmapToProto(submapId, &responseProto);
    if (!error.empty() || responseProto.textures_size() == 0)
    {
      continue;
    }

    // Extract the first texture for simplicity.
    const auto &first_texture = responseProto.textures(0);
    resolution = first_texture.resolution();
    int width = first_texture.width();
    int height = first_texture.height();
    auto slice_pose =
        cartographer::transform::ToRigid3(first_texture.slice_pose());

    // Unpack the intensity and alpha channels from the texture data.
    auto pixels = cartographer::io::UnpackTextureData(first_texture.cells(),
                                                      width, height);

    // Process each pixel directly to filter obstructed wall points.
    for (int y = 0; y < height; ++y)
    {
      for (int x = 0; x < width; ++x)
      {
        const int index = y * width + x;
        const uint8_t intensityValue = pixels.intensity[index];
        const uint8_t alphaValue = pixels.alpha[index];

        if (alphaValue > minimumAlpha && intensityValue < maximumIntensity)
        {
          Eigen::Vector2d global_position =
              pose.translation().head<2>() +
              slice_pose.translation().head<2>() +
              Eigen::Vector2d(x * resolution, y * resolution);

          obstructed_points.push_back({
              static_cast<float>(global_position.x()),
              static_cast<float>(global_position.y()),
              0.0f,
              static_cast<float>(intensityValue),
          });
        }
      }
    }
  }

  pthread_mutex_unlock(&mutex);
  return std::make_tuple(resolution, obstructed_points);
}

void GooglePotato::setLogging(bool state) {
  this->loggingEnabled = state;
}