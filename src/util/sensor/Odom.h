#ifndef ODOM_SENSOR
#define ODOM_SENSOR

#include <cstdlib>
#include <string>

class Odom {
public:
  Odom(std::string name) { this->name = name; }

  std::string name;
};

#endif // ODOM_SENSOR