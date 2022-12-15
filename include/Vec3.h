#pragma once
#include <cmath>

struct Vec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vec3();

    Vec3 operator+(Vec3 const &second_vec3)
    {
        Vec3 res;
        res.x = x + second_vec3.x;
        res.y = y + second_vec3.y;
        res.z = z + second_vec3.z;
        return res;
    }

    Vec3 operator-(Vec3 const &second_vec3)
    {
        Vec3 res;
        res.x = x - second_vec3.x;
        res.y = y - second_vec3.y;
        res.z = z - second_vec3.z;
        return res;
    }

    void Make2D() { this->z = 0; }

    float XY_Distance(Vec3 *);
    float Squared_XY_Distance(Vec3 *);
    float Dot2D(Vec3 *);
};
