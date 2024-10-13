#ifndef SENSOR_IDENTITY
#define SENSOR_IDENTITY

#include <string>

class SensorIdentity {
public:
  SensorIdentity(long timeUS, std::string name) {
    this->timeUS = timeUS;
    this->name = name;
  }

  long timeUS;
  std::string name;
};

#endif // SENSOR_IDENTITY