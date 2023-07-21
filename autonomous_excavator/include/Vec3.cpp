#include "Vec3.h"

#include <cmath>

Vec3::Vec3(){};

void Vec3::Make2D() { this->z = 0; }

double Vec3::Squared_XY_Distance(Vec3* to) {
  double dx = x - to->x;
  double dy = y - to->y;
  return dx * dx + dy * dy;
}

double Vec3::XY_Distance(Vec3* to) {
  double dx = x - to->x;
  double dy = y - to->y;
  return std::sqrt(dx * dx + dy * dy);
}

double Vec3::Dot2D(Vec3* to) { return x * to->y - to->x * y; }
