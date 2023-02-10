#pragma once
#include <geometry_msgs/msg/polygon.hpp>

class Blade {
 public:
  Blade();

  Blade(const geometry_msgs::msg::Polygon::SharedPtr msg);

  void Update(geometry_msgs::msg::Polygon::SharedPtr msg);

  geometry_msgs::msg::Polygon OldFace;
  geometry_msgs::msg::Polygon NewFace;

 private:
  const uint32_t m_VerticesCount = 0;
};
