#include "MarchingCubes.h"

MarchingCubes::MarchingCubes(){};

MarchingCubes::MarchingCubes(const float x_steps, const float y_steps, const float z_steps, const float x_size, const float y_size,
                             const float z_size)
{
    m_Step.x = x_steps;
    m_Step.y = y_steps;
    m_Step.z = z_steps;
    m_Size.x = x_size;
    m_Size.y = y_size;
    m_Size.z = z_size;

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

void MarchingCubes::CreateMessage(visualization_msgs::msg::Marker* msg)
{
    msg->points.clear();
    geometry_msgs::msg::Point vertex;
    for (float i = 0; i < m_Step.x; ++i)
        for (float j = 0; j < m_Step.y; ++j)
            for (float k = 0; k < m_Step.z; ++k) {
                if (!m_UpdateFlag[i][j][k]) continue;
                for (int32_t l = 15; l > -1; --l) {
                    switch (triangles[m_Cubes[i][j][k]][l]) {
                        case 0:
                            vertex.x = (i + 0.5f) * m_Size.x;
                            vertex.y = (j + 0.0f) * m_Size.y;
                            vertex.z = (k + 0.0f) * m_Size.z;
                            msg->points.push_back(vertex);
                            break;
                        case 1:
                            vertex.x = (i + 1.0f) * m_Size.x;
                            vertex.y = (j + 0.5f) * m_Size.y;
                            vertex.z = (k + 0.0f) * m_Size.z;
                            msg->points.push_back(vertex);
                            break;
                        case 2:
                            vertex.x = (i + 0.5f) * m_Size.x;
                            vertex.y = (j + 1.0f) * m_Size.y;
                            vertex.z = (k + 0.0f) * m_Size.z;
                            msg->points.push_back(vertex);
                            break;
                        case 3:
                            vertex.x = (i + 0.0f) * m_Size.x;
                            vertex.y = (j + 0.5f) * m_Size.y;
                            vertex.z = (k + 0.0f) * m_Size.z;
                            msg->points.push_back(vertex);
                            break;
                        case 4:
                            vertex.x = (i + 0.5f) * m_Size.x;
                            vertex.y = (j + 0.0f) * m_Size.y;
                            vertex.z = (k + 1.0f) * m_Size.z;
                            msg->points.push_back(vertex);
                            break;
                        case 5:
                            vertex.x = (i + 1.0f) * m_Size.x;
                            vertex.y = (j + 0.5f) * m_Size.y;
                            vertex.z = (k + 1.0f) * m_Size.z;
                            msg->points.push_back(vertex);
                            break;
                        case 6:
                            vertex.x = (i + 0.5f) * m_Size.x;
                            vertex.y = (j + 1.0f) * m_Size.y;
                            vertex.z = (k + 1.0f) * m_Size.z;
                            msg->points.push_back(vertex);
                            break;
                        case 7:
                            vertex.x = (i + 0.0f) * m_Size.x;
                            vertex.y = (j + 0.5f) * m_Size.y;
                            vertex.z = (k + 1.0f) * m_Size.z;
                            msg->points.push_back(vertex);
                            break;
                        case 8:
                            vertex.x = (i + 0.0f) * m_Size.x;
                            vertex.y = (j + 0.0f) * m_Size.y;
                            vertex.z = (k + 0.5f) * m_Size.z;
                            msg->points.push_back(vertex);
                            break;
                        case 9:
                            vertex.x = (i + 1.0f) * m_Size.x;
                            vertex.y = (j + 0.0f) * m_Size.y;
                            vertex.z = (k + 0.5f) * m_Size.z;
                            msg->points.push_back(vertex);
                            break;
                        case 10:
                            vertex.x = (i + 1.0f) * m_Size.x;
                            vertex.y = (j + 1.0f) * m_Size.y;
                            vertex.z = (k + 0.5f) * m_Size.z;
                            msg->points.push_back(vertex);
                            break;
                        case 11:
                            vertex.x = (i + 0.0f) * m_Size.x;
                            vertex.y = (j + 1.0f) * m_Size.y;
                            vertex.z = (k + 0.5f) * m_Size.z;
                            msg->points.push_back(vertex);
                            break;
                        default:
                            break;
                    }
                }
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

void MarchingCubes::SetUpdateCubeList(const uint32_t i, const uint32_t j, const uint32_t k)
{
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
