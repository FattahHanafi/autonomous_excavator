#include "Blade.h"

Blade::Blade(const geometry_msgs::msg::Polygon::SharedPtr msg) : m_VerticesCount(msg->points.size()) {
  OldFace.points.resize(m_VerticesCount);
  NewFace.points.resize(m_VerticesCount);

  OldFace.points = msg->points;
  NewFace.points = msg->points;
}

void Blade::Update(geometry_msgs::msg::Polygon::SharedPtr msg) {
  OldFace.points = NewFace.points;
  NewFace.points = msg->points;
};
