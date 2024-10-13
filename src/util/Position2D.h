
#include <cmath>

#ifndef POSITION_H
#define POSITION_H

class Position2D {
public:
  // Default constructor
  Position2D() : Position2D(0, 0.0, 0.0, 0.0) {}

  // Parameterized constructor with normalization of theta
  Position2D(long time_us, double x_val, double y_val, double theta_val)
      : timestamp(time_us), x(x_val), y(y_val),
        theta(normalizeTheta(theta_val)) {}

  long timestamp;
  double x;
  double y;
  double theta;

private:
  // Normalize the theta value to be within 0 to 2*PI
  static double normalizeTheta(double theta_val) {
    while (theta_val < 0) {
      theta_val += 2 * M_PI;
    }
    while (theta_val > 2 * M_PI) {
      theta_val -= 2 * M_PI;
    }
    return theta_val;
  }
};

// Operator overloads for Position2D class
inline Position2D operator-(const Position2D &a, const Position2D &b) {
  return Position2D(a.timestamp - b.timestamp, a.x - b.x, a.y - b.y,
                    a.theta - b.theta);
}

inline Position2D operator+(const Position2D &a, const Position2D &b) {
  return Position2D(a.timestamp + b.timestamp, a.x + b.x, a.y + b.y,
                    a.theta + b.theta);
}

inline Position2D operator*(const Position2D &a, const Position2D &b) {
  return Position2D(a.timestamp * b.timestamp, a.x * b.x, a.y * b.y,
                    a.theta * b.theta);
}

inline Position2D operator/(const Position2D &p, const Position2D &a) {
  return Position2D(p.timestamp / a.timestamp, p.x / a.x, p.y / a.y,
                    p.theta / a.theta);
}

inline bool operator==(const Position2D &a, const Position2D &b) {
  return a.timestamp == b.timestamp && a.x == b.x && a.y == b.y &&
         a.theta == b.theta;
}

#endif // POSITION_H