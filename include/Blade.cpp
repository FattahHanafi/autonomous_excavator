#include "Blade.h"

Blade::Blade(){};

Blade::Blade(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
    VerticesCount = msg->points.size();
    OldFace->points.resize(VerticesCount);
    NewFace->points.resize(VerticesCount);

    for (uint32_t i = 0; i < VerticesCount; ++i) {
        OldFace->points[i].x = msg->points[i].x;
        OldFace->points[i].y = msg->points[i].y;
        OldFace->points[i].z = msg->points[i].z;

        NewFace->points[i].x = msg->points[i].x;
        NewFace->points[i].y = msg->points[i].y;
        NewFace->points[i].z = msg->points[i].z;
    }
}

void Blade::Update(geometry_msgs::msg::Polygon* msg)
{
    for (uint32_t i = 0; i < VerticesCount; ++i) {
        OldFace->points[i].x = NewFace->points[i].x;
        OldFace->points[i].y = NewFace->points[i].y;
        OldFace->points[i].z = NewFace->points[i].z;

        NewFace->points[i].x = msg->points[i].x;
        NewFace->points[i].y = msg->points[i].y;
        NewFace->points[i].z = msg->points[i].z;
    }
};

