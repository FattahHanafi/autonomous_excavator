#pragma once
#include <geometry_msgs/msg/polygon.hpp>
#include <iostream>

class Blade {
  public:
    Blade();

    Blade(const geometry_msgs::msg::Polygon::SharedPtr msg);

    void Update(geometry_msgs::msg::Polygon* msg);

    uint32_t VerticesCount = 0;
    geometry_msgs::msg::Polygon::SharedPtr OldFace;
    geometry_msgs::msg::Polygon::SharedPtr NewFace;
};
