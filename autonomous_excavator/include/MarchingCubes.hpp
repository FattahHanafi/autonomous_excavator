#pragma once

#include <boost/qvm/all.hpp>
#include <string>

typedef boost::qvm::vec<float, 3> vec3f;
typedef boost::qvm::vec<uint32_t, 3> vec3u;
typedef boost::qvm::quat<float> quat;

class MarchingCubes {
 public:
  MarchingCubes(const uint32_t, const uint32_t, const uint32_t, const float);
  void setPosition(const vec3f);
  const vec3f getPosition() const;
  void setCount(const vec3u);
  const vec3u getCount() const;
  void setSize(const vec3f);
  const vec3f getSize() const;
  void setOrientation(const quat);
  const quat getOrientation() const;
  std::string print() const;

 private:
  vec3f m_position = boost::qvm::zero_vec<float, 3>();
  quat m_orientation = boost::qvm::identity_quat<float>();
  vec3u m_count = boost::qvm::zero_vec<uint32_t, 3>();
  vec3f m_size = boost::qvm::zero_vec<float, 3>();
};
