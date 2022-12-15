#include "Vec3.h"

Vec3::Vec3(){};

float Vec3::Squared_XY_Distance(Vec3* to)
{
    float dx = x - to->x;
    float dy = y - to->y;
    return dx * dx + dy * dy;
}

float Vec3::XY_Distance(Vec3* to)
{
    float dx = x - to->x;
    float dy = y - to->y;
    return std::sqrt(dx * dx + dy * dy);
}

float Vec3::Dot2D(Vec3* to) { return x * to->y - to->x * y; }

