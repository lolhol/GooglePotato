#include "../google_potato.h"
#include <condition_variable>
#include <cstdint>
#include <ctime>
#include <gtest/gtest.h>
#include <mutex>
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

static std::vector<PointCloud> getFakeLidarData(std::string name, int64_t time,
                                                int64_t step) {
  std::vector<PointCloud> lidarData;
  int64_t totalStep = 0;
  const double radius = 10.0;
  const double height = 2.0;

  for (int i = 0; i < 20; ++i) {
    PointCloud cloud = PointCloud(SensorIdentity(time + totalStep, name));

    for (int j = 0; j < 500; ++j) {
      double angle = 2 * M_PI * j / 500;
      double x = radius * cos(angle);
      double y = radius * sin(angle);
      cloud.add_point(x, y);
    }

    lidarData.push_back(cloud);
    totalStep += step;
  }

  return lidarData;
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
      std::vector<Odom>{}, std::vector<ImuSensor>{});

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  auto fakeData = getFakeLidarData("range", getCurrentTimeInMilliseconds(), 50);

  int i = 0;
  for (auto c : fakeData) {
    potato.handleLidarData(c);

    std::cout << i << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    i++;
  }

  potato.stopAndOptimize();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}