#include "Blade.h"

Blade::Blade(){};

Blade::Blade(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
    m_VerticesCount = msg->points.size();
    m_OldFace->points.resize(m_VerticesCount);
    m_NewFace->points.resize(m_VerticesCount);

    for (uint32_t i = 0; i < m_VerticesCount; ++i) {
        m_OldFace->points[i].x = msg->points[i].x;
        m_OldFace->points[i].y = msg->points[i].y;
        m_OldFace->points[i].z = msg->points[i].z;

        m_NewFace->points[i].x = msg->points[i].x;
        m_NewFace->points[i].y = msg->points[i].y;
        m_NewFace->points[i].z = msg->points[i].z;
    }
}

void Blade::Update(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
    for (uint32_t i = 0; i < m_VerticesCount; ++i) {
        m_OldFace->points[i].x = m_NewFace->points[i].x;
        m_OldFace->points[i].y = m_NewFace->points[i].y;
        m_OldFace->points[i].z = m_NewFace->points[i].z;

        m_NewFace->points[i].x = msg->points[i].x;
        m_NewFace->points[i].y = msg->points[i].y;
        m_NewFace->points[i].z = msg->points[i].z;
    }
};

