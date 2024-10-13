#ifndef IMU_SENSOR
#define IMU_SENSOR

#include <cstdlib>
#include <string>

class ImuSensor {
public:
  ImuSensor(std::string name) { this->name = name; }

  std::string name;
};

#endif // IMU_SENSOR