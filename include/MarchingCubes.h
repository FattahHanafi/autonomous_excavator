#include <geometry_msgs/msg/polygon.hpp>
#include <iostream>
#include <list>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

#include "Blade.h"
#include "Vec3.h"

class MarchingCubes {
  public:
    MarchingCubes();

    MarchingCubes(const uint32_t x_steps, const uint32_t y_steps, const uint32_t z_steps, const float x_size = 0.01f, const float y_size = 0.01f,
                  const float z_size = 0.0f, const float x_origin = 0.0f, const float y_origin = 0.0f, const float z_origin = 0.0f);

    void SetVertex(const uint32_t i, const uint32_t j, const uint32_t k, const bool value);

    void RebuildCubes();

    void SetBlade(const geometry_msgs::msg::Polygon::SharedPtr msg);

    void RebuildSurface(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  private:
    Vec3 m_Step;
    Vec3 m_Size;
    Vec3 m_Origin;

    Blade blade;

    std::vector<std::vector<std::vector<bool>>> m_Vertices;
    std::vector<std::vector<std::vector<uint8_t>>> m_Cubes;
    std::vector<std::vector<std::vector<bool>>> m_UpdateFlag;

    void SetUpdateCubeList(const uint32_t i, const uint32_t j, const uint32_t k);
};
