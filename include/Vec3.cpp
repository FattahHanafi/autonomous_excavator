#include "Vec3.h"

Vec3::Vec3()
{
    x = 0;
    y = 0;
    z = 0;
}

float Vec3::XY_Distance(Vec3* to)
{
    float dx = x - to->x;
    float dy = y - to->y;
    return dx * dx + dy * dy;
}
