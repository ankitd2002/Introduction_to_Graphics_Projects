#pragma once

#include <Eigen/Core>

class VertexAttributes
{
public:
    VertexAttributes(float x = 0, float y = 0, float z = 0, float w = 1)
    {
        position << x, y, z, w;
        
        
    }

    // Interpolates the vertex attributes
    static VertexAttributes interpolate(
        const VertexAttributes &a,
        const VertexAttributes &b,
        const VertexAttributes &c,
        const float alpha,
        const float beta,
        const float gamma)
    {
        VertexAttributes r;
        r.position = alpha * a.position + beta * b.position + gamma * c.position;
        r.color= alpha * a.color + beta * b.color + gamma * c.color;
        return r;
    }

    Eigen::Vector4f position;
    Eigen::Vector3d N;
    Eigen::Vector3d color;
    Eigen::Vector3d tri_normal;
    Eigen::Matrix3d v_normal;
};

class FragmentAttributes
{
public:
    FragmentAttributes(float r = 0, float g = 0, float b = 0, float a = 1,float x = 0,float y = 0, float z = 0, float w=1)
    {
        color << r, g, b, a;
        position<<x,y,-z,w;
    }

    Eigen::Vector4f color;
    Eigen::Vector4f position;
};

class FrameBufferAttributes
{
public:
    FrameBufferAttributes(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255)
    {
        color << r, g, b, a;
        depth = 2;
    }

    Eigen::Matrix<uint8_t, 4, 1> color;
    float depth;
};

class UniformAttributes
{
public:
    Eigen::Matrix4f P;
    Eigen::Matrix4f camera_transformation1;
    Eigen::Vector4f color;
    Eigen::Matrix4f view;
    std::vector<Eigen::Vector3d> light_positions;
    std::vector<Eigen::Vector3d> light_colors;
    Eigen::Vector3d obj_diffuse_color;
    Eigen::Vector3d obj_specular_color;
    double obj_specular_exponent;
    Eigen::Vector3d camera_position;
    Eigen::Matrix4f trafo;
};