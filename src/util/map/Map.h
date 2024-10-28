#ifndef MAP_H
#define MAP_H

#include "Pixel.h"
#include <vector>
class Map {
public:
  Map(int height, int width, double resolution, double originX, double originY,
      std::vector<Pixel> map)
      : height(height), width(width), resolution(resolution), originX(originX),
        originY(originY), map(map) {}

  const int height;
  const int width;
  const double resolution;

  const double originX;
  const double originY;

  const std::vector<Pixel> map;
};

#endif // ODOM_DATA_H