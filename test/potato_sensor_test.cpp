#include "../src/google_potato.h"
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <gtest/gtest.h>
#include <iostream>
#include <mutex>
#include <ostream>
#include <thread>
#include <unistd.h>
#include <vector>

std::mutex dataMutex;
std::condition_variable dataCv;
std::vector<PointCloud> lidarDataQueue;
bool dataAvailable = false;
bool stopThread = false;

void onPoseUpdate(const Position &position) {
  std::cout << "Pos" << position.x << std::endl;
}

static std::vector<PointCloud> getFakeLidarData(std::string name, int64_t time, int64_t step) {
  std::vector<PointCloud> lidarData;
  const double radius = 10.0; // Radius of the circle
  auto totalTime = time;

  for (int i = 0; i < 20; ++i) {
    PointCloud cloud = PointCloud(SensorIdentity(totalTime, name));

    for (int j = 0; j < 500; ++j) {
      double angle = 2 * M_PI * j / 500; // Divide circle into 500 points
      double x = radius * cos(angle);    // X coordinate on the circle
      double y = radius * sin(angle);    // Y coordinate on the circle
      cloud.add_point(x, y, 1);          // Add point to the cloud (2D point)
    }

    lidarData.push_back(cloud);
    totalTime += step;
  }

  return lidarData;
}

static std::vector<ImuData> getFakeImuData(std::string name, int64_t time, int64_t step) {
  auto totalTime = time;
  std::vector<ImuData> imuFake;
  for (int i = 0; i < 40; i++) {
    imuFake.push_back(ImuData(SensorIdentity(totalTime, name), 0.0, 0.0, 9.8, 0.0, 0.0, 0.0));
    totalTime += step;
  }

  return imuFake;
}

static int64_t getCurrentTimeInMilliseconds() {
  auto now = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch());
  return duration.count();
}

TEST(SensorTests, TestLidarDataAddition) {
  auto potato = GooglePotato(
      "../configuration", "mapping.lua", onPoseUpdate,
      std::vector<LidarSensor>{LidarSensor(0.0, 0.0, 0.0, 6.0, "range")},
      std::vector<Odom>{}, std::vector<ImuSensor>{}, true);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  auto fakeData = getFakeLidarData("range", getCurrentTimeInMilliseconds(), 50);

  int i = 0;
  for (auto c : fakeData) {
    potato.handleLidarData(c);

    std::cout << i << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    i++;
  }

  auto pointsHighRes = potato.getMapPointsHighRes();
  std::cout << "!!!" << std::endl;
  auto pointsLowRes = potato.getMapPointsLowRes();
  std::cout << "!!!" << std::endl;
  // auto pointsGravityRes = potato.getMapPointsGravityAligned();
  // std::cout << "!!!" << std::endl;

  std::cout << " points: " << potato.getMapPointsHighRes().size()
            << potato.getMapPointsLowRes().size() << std::endl;
  // ASSERT_GT(points.size(), 0);

  potato.stopAndOptimize();
  // auto map = potato.paint2DMap();
  // std::cout << map.map.size() << "!!!!" << std::endl;
  // paintToPNG("out.png", map);

  auto map = potato.getObstructedWallPoints(120, 0);
}

TEST(SensorTests, TestImuDataAddition) {
  auto potato = GooglePotato(
      "../configuration", "mapping.lua", onPoseUpdate,
      std::vector<LidarSensor>{LidarSensor(0.0, 0.0, 0.0, 6.0, "range")},
      std::vector<Odom>{}, std::vector<ImuSensor>{}, true);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  auto fakeDataRange = getFakeLidarData("range", getCurrentTimeInMilliseconds(), 50);
  auto fakeDataImu = getFakeImuData("imu0", getCurrentTimeInMilliseconds(), 25);

  int i = 0;
  for (auto c : fakeDataImu) {
    if (i % 2 == 0) {
      potato.handleLidarData(fakeDataRange.at(i / 2));
    }

    potato.handleImuData(c);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    i++;
  }

  potato.stopAndOptimize();

  auto map = potato.getObstructedWallPoints(147, 0);

  std::cout << static_cast<std::vector<std::vector<float>>>(std::get<1>(map)).size() << std::endl;
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}