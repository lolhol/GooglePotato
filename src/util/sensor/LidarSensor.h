#ifndef LIDAR_SENSOR
#define LIDAR_SENSOR

#include <cstdlib>
#include <string>

class LidarSensor {
public:
  LidarSensor(float x, float y, float z, float scanTimeHz, std::string name) {
    this->x = x;
    this->y = y;
    this->z = z;
    this->scanTimeHz = scanTimeHz;
    this->name = name;
  }

  LidarSensor(float pos[3], float scanTimeHz, std::string name)
      : LidarSensor(pos[0], pos[1], pos[2], scanTimeHz, name) {}

  LidarSensor(float pos[3], float scanTimeHz)
      : LidarSensor(pos[0], pos[1], pos[2], scanTimeHz,
                    std::to_string(random())) {}

  float x;
  float y;
  float z;
  float scanTimeHz;
  std::string name;
};

#endif // LIDAR_SENSOR
