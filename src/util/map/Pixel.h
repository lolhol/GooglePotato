#ifndef PIXEL_H
#define PIXEL_H

#include <vector>

class Pixel {
public:
  Pixel(double r, double g, double b) : r(r), g(g), b(b) {}

  double getR() const { return r; }
  double getG() const { return g; }
  double getB() const { return b; }

private:
  const double r, g, b;
};

#endif // ODOM_DATA_H