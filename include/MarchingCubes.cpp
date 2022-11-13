#include "MarchingCubes.h"

MarchingCubes::MarchingCubes(){};

MarchingCubes::MarchingCubes(const uint32_t x_steps, const uint32_t y_steps, const uint32_t z_steps, const float x_size, const float y_size,
                             const float z_size, const float x_origin, const float y_origin, const float z_origin)
{
    m_Step.x = x_steps;
    m_Step.y = y_steps;
    m_Step.z = z_steps;
    m_Size.x = x_size;
    m_Size.y = y_size;
    m_Size.z = z_size;
    m_Origin.x = x_origin;
    m_Origin.y = y_origin;
    m_Origin.z = z_origin;

    m_Vertices.resize(m_Step.x + 1);
    for (uint32_t i = 0; i < m_Vertices.size(); ++i) {
        m_Vertices[i].resize(m_Step.y + 1);
        for (uint32_t j = 0; j < m_Vertices[i].size(); ++j) m_Vertices[i][j].resize(m_Step.z + 1, 0);
    }

    m_Cubes.resize(m_Step.x);
    for (uint32_t i = 0; i < m_Cubes.size(); ++i) {
        m_Cubes[i].resize(m_Step.y);
        for (uint32_t j = 0; j < m_Cubes[i].size(); ++j) m_Cubes[i][j].resize(m_Step.z, 0);
    }

    m_UpdateFlag.resize(m_Step.x);
    for (uint32_t i = 0; i < m_UpdateFlag.size(); ++i) {
        m_UpdateFlag[i].resize(m_Step.y);
        for (uint32_t j = 0; j < m_UpdateFlag[i].size(); ++j) m_UpdateFlag[i][j].resize(m_Step.z, 0);
    }
}

void MarchingCubes::SetVertex(const uint32_t i, const uint32_t j, const uint32_t k, const bool value)
{
    m_Vertices[i][j][k] = value;
    SetUpdateCubeList(i, j, k);
}

void MarchingCubes::RebuildCubes()
{
    for (uint32_t i = 0; i < m_Step.x; ++i)
        for (uint32_t j = 0; j < m_Step.y; ++j)
            for (uint32_t k = 0; k < m_Step.z; ++k) {
                if (!m_UpdateFlag[i][j][k]) continue;
                m_UpdateFlag[i][j][k] = 0;
                m_Cubes[i][j][k] = 0;
                m_Cubes[i][j][k] |= m_Vertices[i + 0][j + 0][k + 0] << 0;
                m_Cubes[i][j][k] |= m_Vertices[i + 1][j + 0][k + 0] << 1;
                m_Cubes[i][j][k] |= m_Vertices[i + 1][j + 1][k + 0] << 2;
                m_Cubes[i][j][k] |= m_Vertices[i + 0][j + 1][k + 0] << 3;
                m_Cubes[i][j][k] |= m_Vertices[i + 0][j + 0][k + 1] << 4;
                m_Cubes[i][j][k] |= m_Vertices[i + 1][j + 0][k + 1] << 5;
                m_Cubes[i][j][k] |= m_Vertices[i + 1][j + 1][k + 1] << 6;
                m_Cubes[i][j][k] |= m_Vertices[i + 0][j + 1][k + 1] << 7;
            }
}

void MarchingCubes::SetBlade(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
    blade.Update(msg);
    blade.Update(msg);
}

void MarchingCubes::RebuildSurface(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    Vec3 mc_point;
    Vec3 pc_point;
    for (uint32_t i = 0; i < m_Step.x + 1; ++i) {
        mc_point.x = m_Origin.x + i * m_Size.x;
        for (uint32_t j = 0; j < m_Step.y + 1; ++j) {
            mc_point.y = m_Origin.y + j * m_Size.y;
            float dis = 1e6;
            for (uint32_t idx = 0; idx < msg->width * msg->height * msg->point_step; idx += msg->point_step) {
                pc_point.x = *(float*)&msg->data[idx + msg->fields[0].offset];
                pc_point.y = *(float*)&msg->data[idx + msg->fields[1].offset];
                pc_point.z = *(float*)&msg->data[idx + msg->fields[2].offset];

                float dis2 = mc_point.XY_Distance(&pc_point);
                if (dis2 < dis) {
                    mc_point.z = pc_point.z;
                    dis = dis2;
                }
            }
            for (uint32_t k = 0; k < m_Step.z + 1; ++k) {
                SetVertex(i, j, k, mc_point.z > (m_Origin.z + k * m_Size.z));
                m_UpdateFlag[i][j][k] = true;
            }
        }
    }
}

void MarchingCubes::SetUpdateCubeList(const uint32_t i, const uint32_t j, const uint32_t k)
{
    std::list<int32_t> X = {-1, 0};
    std::list<int32_t> Y = {-1, 0};
    std::list<int32_t> Z = {-1, 0};

    if (i == 0) X.pop_front();
    if (i == (m_Step.x + 1)) X.pop_back();

    if (j == 0) Y.pop_front();
    if (j == (m_Step.y + 1)) Y.pop_back();

    if (k == 0) Z.pop_front();
    if (k == (m_Step.z + 1)) Z.pop_back();

    for (int32_t u : X)
        for (int32_t v : Y)
            for (int32_t w : Z) m_UpdateFlag[i + u][j + v][k + w] = true;
}
