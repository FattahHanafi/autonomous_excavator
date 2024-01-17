#include "MarchingCubes.hpp"

MarchingCubes::MarchingCubes(const uint32_t x_count, const uint32_t y_count,
                             const uint32_t z_count, const float size) {
  X(m_count) = x_count;
  Y(m_count) = y_count;
  Z(m_count) = z_count;
  X(m_size) = size;
  Y(m_size) = size;
  Z(m_size) = size;
};

void MarchingCubes::setPosition(const vec3f position) { m_position = position; }
const vec3f MarchingCubes::getPosition() const { return m_position; }

void MarchingCubes::setCount(const vec3u count) { m_position = count; }
const vec3u MarchingCubes::getCount() const { return m_count; }

void MarchingCubes::setSize(const vec3f size) { m_size = size; }
const vec3f MarchingCubes::getSize() const { return m_size; }

void MarchingCubes::setOrientation(const quat orientation) {
  m_orientation = orientation;
}
const quat MarchingCubes::getOrientation() const { return m_orientation; }

std::string MarchingCubes::print() const {
  std::string res = "\nPosition : " + std::to_string(X(m_position)) + " , " +
                    std::to_string(Y(m_position)) + " , " +
                    std::to_string(Z(m_position));
  res += "\nCount : " + std::to_string(X(m_count)) + " , " +
         std::to_string(Y(m_count)) + " , " + std::to_string(Z(m_count));
  res += "\nSize : " + std::to_string(X(m_size)) + " , " +
         std::to_string(Y(m_size)) + " , " + std::to_string(Z(m_size));
  return res;
}
