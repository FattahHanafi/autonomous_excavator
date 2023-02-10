#include "MarchingCubes.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

MarchingCubes::MarchingCubes(){};

MarchingCubes::MarchingCubes(const double x_step, const double y_step, const double z_step, const double size) {
  m_Step.x = x_step;
  m_Step.y = y_step;
  m_Step.z = z_step;
  m_Size.x = size;
  m_Size.y = size;
  m_Size.z = size;

  m_Vertices.resize(m_Step.x + 1);
  for (uint32_t i = 0; i < m_Step.x + 1; ++i) {
    m_Vertices.at(i).resize(m_Step.y + 1);
    for (uint32_t j = 0; j < m_Step.y + 1; ++j) m_Vertices.at(i).at(j).resize(m_Step.z + 1, 0);
  }

  m_Cubes.resize(m_Step.x);
  for (uint32_t i = 0; i < m_Cubes.size(); ++i) {
    m_Cubes.at(i).resize(m_Step.y);
    for (uint32_t j = 0; j < m_Cubes.at(i).size(); ++j) m_Cubes.at(i).at(j).resize(m_Step.z, 0);
  }

  m_Nodes.at(0).x = 0.5 * m_Size.x;
  m_Nodes.at(0).y = 0.0 * m_Size.y;
  m_Nodes.at(0).z = 0.0 * m_Size.z;

  m_Nodes.at(1).x = 1.0 * m_Size.x;
  m_Nodes.at(1).y = 0.5 * m_Size.y;
  m_Nodes.at(1).z = 0.0 * m_Size.z;

  m_Nodes.at(2).x = 0.5 * m_Size.x;
  m_Nodes.at(2).y = 1.0 * m_Size.y;
  m_Nodes.at(2).z = 0.0 * m_Size.z;

  m_Nodes.at(3).x = 0.0 * m_Size.x;
  m_Nodes.at(3).y = 0.5 * m_Size.y;
  m_Nodes.at(3).z = 0.0 * m_Size.z;

  m_Nodes.at(4).x = 0.0 * m_Size.x;
  m_Nodes.at(4).y = 0.0 * m_Size.y;
  m_Nodes.at(4).z = 0.5 * m_Size.z;

  m_Nodes.at(5).x = 1.0 * m_Size.x;
  m_Nodes.at(5).y = 0.0 * m_Size.y;
  m_Nodes.at(5).z = 0.5 * m_Size.z;

  m_Nodes.at(6).x = 1.0 * m_Size.x;
  m_Nodes.at(6).y = 1.0 * m_Size.y;
  m_Nodes.at(6).z = 0.5 * m_Size.z;

  m_Nodes.at(7).x = 0.0 * m_Size.x;
  m_Nodes.at(7).y = 1.0 * m_Size.y;
  m_Nodes.at(7).z = 0.5 * m_Size.z;

  m_Nodes.at(8).x = 0.5 * m_Size.x;
  m_Nodes.at(8).y = 0.0 * m_Size.y;
  m_Nodes.at(8).z = 1.0 * m_Size.z;

  m_Nodes.at(9).x = 1.0 * m_Size.x;
  m_Nodes.at(9).y = 0.5 * m_Size.y;
  m_Nodes.at(9).z = 1.0 * m_Size.z;

  m_Nodes.at(10).x = 0.5 * m_Size.x;
  m_Nodes.at(10).y = 1.0 * m_Size.y;
  m_Nodes.at(10).z = 1.0 * m_Size.z;

  m_Nodes.at(11).x = 0.0 * m_Size.x;
  m_Nodes.at(11).y = 0.5 * m_Size.y;
  m_Nodes.at(11).z = 1.0 * m_Size.z;
}

void SumPoints(geometry_msgs::msg::Point* a, geometry_msgs::msg::Point* b) {
  a->x += b->x;
  a->y += b->y;
  a->z += b->z;
}

void MarchingCubes::CreateMessage(visualization_msgs::msg::Marker* msg) {
  msg->points.clear();
  geometry_msgs::msg::Point vertex;
  for (uint32_t i = 0; i < m_Step.x; ++i)
    for (uint32_t j = 0; j < m_Step.y; ++j)
      for (uint32_t k = 0; k < m_Step.z; ++k) {
        vertex.x = i * m_Size.x;
        vertex.y = j * m_Size.y;
        vertex.z = k * m_Size.z;
        uint32_t l = 0;
        uint8_t idx = 0;
        while (())
          SumPoints(&vertex, &m_Nodes.at(l));
        ++l;
      }
}
}
}

void MarchingCubes::SetVertex(const uint32_t i, const uint32_t j, const uint32_t k, const bool value) {
  m_Vertices[i][j][k] = value;
}

void MarchingCubes::SetVertices(const uint32_t i, const uint32_t j, const uint32_t length) {
  for (uint32_t k = 0; k < length; ++k)
    m_Vertices[i][j][k] = true;
  for (uint32_t k = length; k < m_Step.z; ++k)
    m_Vertices[i][j][k] = false;
}

void MarchingCubes::RebuildCubes() {
  for (uint32_t i = 0; i < m_Step.x; ++i)
    for (uint32_t j = 0; j < m_Step.y; ++j)
      for (uint32_t k = 0; k < m_Step.z; ++k) {
        // if (!m_UpdateFlag[i][j][k]) continue;
        // m_UpdateFlag[i][j][k] = 0;
        m_Cubes[i][j][k] = m_Vertices[i + 0][j + 0][k + 0];
        m_Cubes[i][j][k] |= m_Vertices[i + 1][j + 0][k + 0] << 1;
        m_Cubes[i][j][k] |= m_Vertices[i + 1][j + 1][k + 0] << 2;
        m_Cubes[i][j][k] |= m_Vertices[i + 0][j + 1][k + 0] << 3;
        m_Cubes[i][j][k] |= m_Vertices[i + 0][j + 0][k + 1] << 4;
        m_Cubes[i][j][k] |= m_Vertices[i + 1][j + 0][k + 1] << 5;
        m_Cubes[i][j][k] |= m_Vertices[i + 1][j + 1][k + 1] << 6;
        m_Cubes[i][j][k] |= m_Vertices[i + 0][j + 1][k + 1] << 7;
      }
}

double MarchingCubes::CalculateVolume() {
  double volume = 0;
  for (uint32_t i = 0; i < m_Step.x; ++i)
    for (uint32_t j = 0; j < m_Step.y; ++j)
      for (uint32_t k = 0; k < m_Step.z; ++k) volume += m_Volumes[m_Cubes[i][j][k]];

  return volume;
}

void MarchingCubes::SetBlade(geometry_msgs::msg::Polygon* msg) {
  m_Blade.Update(msg);
  m_Blade.Update(msg);
}

void MarchingCubes::SetUpdateCubeList(const uint32_t i, const uint32_t j, const uint32_t k) {
  std::list<int32_t> X = {-1, 0};
  std::list<int32_t> Y = {-1, 0};
  std::list<int32_t> Z = {-1, 0};

  if (i == 0) X.pop_front();
  if (i == (m_Step.x)) X.pop_back();

  if (j == 0) Y.pop_front();
  if (j == (m_Step.y)) Y.pop_back();

  if (k == 0) Z.pop_front();
  if (k == (m_Step.z)) Z.pop_back();

  for (int32_t u : X)
    for (int32_t v : Y)
      for (int32_t w : Z) m_UpdateFlag[i + u][j + v][k + w] = true;
}

void MarchingCubes::CutBlade() {
  double x_range[] = {1e9, -1e9};
  double y_range[] = {1e9, -1e9};
  double z_range[] = {1e9, -1e9};
  for (auto p : blade.m_NewFace->points) {
    x_range[0] = std::min(x_range[0], p.x);
    y_range[0] = std::min(y_range[0], p.y);
    z_range[0] = std::min(z_range[0], p.z);
    x_range[1] = std::max(x_range[1], p.x);
    y_range[1] = std::max(y_range[1], p.y);
    z_range[1] = std::max(z_range[1], p.z);
  }
  for (auto p : blade.m_OldFace->points) {
    x_range[0] = std::min(x_range[0], p.x);
    y_range[0] = std::min(y_range[0], p.y);
    z_range[0] = std::min(z_range[0], p.z);
    x_range[1] = std::max(x_range[1], p.x);
    y_range[1] = std::max(y_range[1], p.y);
    z_range[1] = std::max(z_range[1], p.z);
  }

  uint32_t i_range[] = {uint32_t(std::floor(x_range[0] / m_Size.x)), uint32_t(std::ceil(x_range[1] / m_Size.x))};
  uint32_t j_range[] = {uint32_t(std::floor(y_range[0] / m_Size.y)), uint32_t(std::ceil(y_range[1] / m_Size.y))};
  uint32_t k_range[] = {uint32_t(std::floor(z_range[0] / m_Size.z)), uint32_t(std::ceil(z_range[1] / m_Size.z))};

  geometry_msgs::msg::Point32 p;
  for (uint32_t i = i_range[0]; i <= i_range[1]; ++i)
    for (uint32_t j = j_range[0]; j <= j_range[1]; ++j)
      for (uint32_t k = k_range[0]; k <= k_range[1]; ++k) {
        p.x = i * m_Size.x;
        p.y = j * m_Size.y;
        p.z = k * m_Size.z;
        SetVertex(i, j, k, !IsInside(&m_Blade, &p));
      }
}

bool MarchingCubes::IsInside(Blade* blade, geometry_msgs::msg::Point32* P) {
  bool s0, s1, s2, s3, s4, s5;
  s0 = sideSign(&blade->m_OldFace->points[0], &blade->m_OldFace->points[1], &blade->m_OldFace->points[2], P);
  s1 = sideSign(&blade->m_NewFace->points[0], &blade->m_NewFace->points[1], &blade->m_NewFace->points[2], P);
  if (s0 == s1) return 0;

  s2 = sideSign(&blade->m_OldFace->points[0], &blade->m_NewFace->points[0], &blade->m_NewFace->points[1], P);
  s3 = sideSign(&blade->m_OldFace->points[3], &blade->m_NewFace->points[3], &blade->m_NewFace->points[2], P);
  if (s2 == s3) return 0;

  s4 = sideSign(&blade->m_OldFace->points[0], &blade->m_NewFace->points[0], &blade->m_NewFace->points[3], P);
  s5 = sideSign(&blade->m_OldFace->points[1], &blade->m_NewFace->points[1], &blade->m_NewFace->points[2], P);
  if (s4 == s5) return 0;

  return 1;
}

bool MarchingCubes::sideSign(geometry_msgs::msg::Point32* A, geometry_msgs::msg::Point32* B, geometry_msgs::msg::Point32* C,
                             geometry_msgs::msg::Point32* P) {
  geometry_msgs::msg::Point32 B_;
  geometry_msgs::msg::Point32 C_;
  geometry_msgs::msg::Point32 P_;
  B_.x = B->x - A->x;
  B_.y = B->y - A->y;
  B_.z = B->z - A->z;

  C_.x = C->x - A->x;
  C_.y = C->y - A->y;
  C_.z = C->z - A->z;

  P_.x = P->x - A->x;
  P_.y = P->y - A->y;
  P_.z = P->z - A->z;
  // Bx*Cy*Pz - Bx*Cz*Py - By*Cx*Pz + By*Cz*Px + Bz*Cx*Py - Bz*Cy*Px
  double det = +B_.x * C_.y * P_.z;
  det += -B_.x * C_.z * P_.y;
  det += -B_.y * C_.x * P_.z;
  det += +B_.y * C_.z * P_.x;
  det += +B_.z * C_.x * P_.y;
  det += -B_.z * C_.y * P_.x;

  return (det > 0);
}
