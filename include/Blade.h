#pragma once
#include <geometry_msgs/msg/polygon.hpp>
#include <iostream>

class Blade {
  public:
    Blade();

    Blade(const geometry_msgs::msg::Polygon::SharedPtr msg);

    void Update(const geometry_msgs::msg::Polygon::SharedPtr msg);

  private:
    uint32_t m_VerticesCount = 0;
    geometry_msgs::msg::Polygon::SharedPtr m_OldFace;
    geometry_msgs::msg::Polygon::SharedPtr m_NewFace;
};
