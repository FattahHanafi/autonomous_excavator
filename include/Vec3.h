#pragma once

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Vec3();

  Vec3 operator+(Vec3 const &second_vec3) {
    Vec3 res;
    res.x = x + second_vec3.x;
    res.y = y + second_vec3.y;
    res.z = z + second_vec3.z;
    return res;
  }

  Vec3 operator-(Vec3 const &second_vec3) {
    Vec3 res;
    res.x = x - second_vec3.x;
    res.y = y - second_vec3.y;
    res.z = z - second_vec3.z;
    return res;
  }

  void Make2D();

  double XY_Distance(Vec3 *);
  double Squared_XY_Distance(Vec3 *);
  double Dot2D(Vec3 *);
};
