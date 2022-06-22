// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "raster.h"

#include <gif.h>
#include <fstream>

#include <Eigen/Geometry>
// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;
using namespace Eigen;

//Image height
const int H = 480;

//Camera settings
const double near_plane = -1.5; //AKA focal length
const double far_plane = near_plane * 100;
const double field_of_view = 0.7854; //45 degrees
const double aspect_ratio = 1.5;
const bool is_perspective = false;
const Vector3d camera_position(0, 0, 3);
const Vector3d camera_gaze(0, 0, -1);
const Vector3d camera_top(0, 1, 0);

//Object
const std::string data_dir = DATA_DIR;
const std::string mesh_filename(data_dir + "bunny.off");
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)

//Material for the object
const Vector3d obj_diffuse_color(0.5, 0.5, 0.5);
const Vector3d obj_specular_color(0.2, 0.2, 0.2);
const double obj_specular_exponent = 256.0;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector3d> light_colors;
//Ambient light
const Vector3d ambient_light(0.3, 0.3, 0.3);

//Fills the different arrays
void setup_scene()
{
    //Loads file
    std::ifstream in(mesh_filename);
    if (!in.good())
    {
        std::cerr << "Invalid file " << mesh_filename << std::endl;
        exit(1);
    }
    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    vertices.resize(nv, 3);
    facets.resize(nf, 3);
    for (int i = 0; i < nv; ++i)
    {
        in >> vertices(i, 0) >> vertices(i, 1) >> vertices(i, 2);
    }
    for (int i = 0; i < nf; ++i)
    {
        int s;
        in >> s >> facets(i, 0) >> facets(i, 1) >> facets(i, 2);
        assert(s == 3);
    }

    //Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16);
}

void build_uniform(UniformAttributes &uniform)
{
    //TODO: setup uniform

    //TODO: setup camera, compute w, u, v
    const Vector3d w = - (camera_gaze).normalized();
    const Vector3d u = camera_top.cross(w).normalized();
    const Vector3d v = w.cross(u).normalized();

    //TODO: compute the camera transformation

    Matrix4f camera;
    camera << u(0), v(0), w(0), camera_position(0),
        u(1), v(1), w(1), camera_position(1),
        u(2), v(2), w(2), camera_position(2),
        0, 0, 0, 1;
    
    Matrix4f camera_transformation = camera.inverse();
    // std::cout<<camera_transformation<<std::endl;

    double h1 = -near_plane * tan(field_of_view / 2);
    double image_top = h1; // top 
    double image_bottom = -image_top;

    double image_right = aspect_ratio * image_top; // right
    double image_left = -image_right;

    //to set up the orthographic projection //

    //TODO: setup projection matrix

    double a1 = 2/(image_right-image_left);
    double a2 = - ((image_right+image_left)/(image_right - image_left));
    double a3 = 2/ (image_top - image_bottom);
    double a4 = 2 / (near_plane - far_plane);
    double a5 = - ((image_top + image_bottom)/ (image_top -image_bottom));
    double a6 = - ((near_plane + far_plane) / (near_plane - far_plane));

    double a7 = near_plane + far_plane;
    double a8 = - (near_plane * far_plane);

    Matrix4f orth;
    Matrix4f pers;
    Matrix4f p;

    orth<< a1,0,0,a2,
        0,a3,0,a5,
        0,0,a4,a6,
        0,0,0,1;
    if (is_perspective)
    {
        //TODO setup prespective camera
        pers<<near_plane,0,0,0,
           0,near_plane,0,0,
           0,0,a7,a8,
           0,0,1,0;
        p = orth * pers;

        
    }
    else
    {
        p = orth;

    }

    uniform.P = p;
    uniform.camera_transformation = camera_transformation;
}

void simple_render(Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        VertexAttributes out;
        out.position = uniform.P * uniform.camera_transformation * va.position;
        return out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: fill the shader
        return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };

    std::vector<VertexAttributes> vertex_attributes;
    //TODO: build the vertex attributes from vertices and facets
    vector<VertexAttributes> vertex;
    for (int i = 0; i < facets.rows(); i++)
    {
        int v1 = facets(i, 0);
        int v2 = facets(i, 1);
        int v3 = facets(i, 2);

        Vector3d p1 = vertices.row(v1);
        Vector3d p2 = vertices.row(v2);
        Vector3d p3 = vertices.row(v3);

        vertex.push_back(VertexAttributes(p1[0],p1[1],p1[2]));
        vertex.push_back(VertexAttributes(p2[0],p2[1],p2[2]));
        vertex.push_back(VertexAttributes(p3[0],p3[1],p3[2]));
    }
        
    rasterize_triangles(program, uniform, vertex, frameBuffer);
}


Matrix4f compute_rotation(const double alpha)
{
    //TODO: Compute the rotation matrix of angle alpha on the y axis 
    //around the object barycenter
    Matrix4f res;

    Matrix4f trans;

    double cx = 0.0;
    double cy = 0.0;
    double cz = 0.0;

    

    for(int i=0; i<vertices.rows(); i++){

        cx += vertices(i,0);
        cy += vertices(i,1);
        cz += vertices(i,2);

    }

    cx = cx/ vertices.rows();
    cy = cy / vertices.rows();
    cz = cz / vertices.rows();

    Matrix4f tc;
    tc << 1,0,0,cx,
          0,1,0,cy,
          0,0,1,cz,
          0,0,0,1;
             


    Matrix4f tcminus;
    tcminus << 1, 0, 0, -cx,
        0, 1, 0, -cy,
        0, 0, 1, -cz,
        0, 0, 0, 1;

    res << cos(alpha),0, sin(alpha),0,
            0,1,0,0,
            -sin(alpha),0,cos(alpha),0,
            0,0,0,1;
           
    
    trans = tc * res * tcminus;
    return trans;
}

void wireframe_render(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
   
    
    Matrix4f trafo = compute_rotation(alpha);
    uniform.trafo = trafo;


    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        VertexAttributes out;
        out.position = uniform.P * uniform.camera_transformation *  uniform.trafo * va.position;

        return out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: fill the shader
        return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };

    std::vector<VertexAttributes> vertex_attributes;

    //TODO: generate the vertex attributes for the edges and rasterize the lines
    //TODO: use the transformation matrix
    vector<VertexAttributes>  vertex;
    for (int i = 0; i < facets.rows(); i++)
    {
        int v1 = facets(i, 0);
        int v2 = facets(i, 1);
        int v3 = facets(i, 2);

        Vector3d p1 = vertices.row(v1);
        Vector3d p2 = vertices.row(v2);
        Vector3d p3 = vertices.row(v3);

        vertex.push_back(VertexAttributes(p1[0], p1[1], p1[2]));
        vertex.push_back(VertexAttributes(p2[0], p2[1], p2[2]));
        vertex.push_back(VertexAttributes(p2[0], p2[1], p2[2]));
        vertex.push_back(VertexAttributes(p3[0], p3[1], p3[2]));
        vertex.push_back(VertexAttributes(p3[0], p3[1], p3[2]));
        vertex.push_back(VertexAttributes(p1[0], p1[1], p1[2]));
    }

    rasterize_lines(program, uniform, vertex, 0.5, frameBuffer);
}

void get_shading_program(Program &program)
{
  

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: transform the position and the normal

        VertexAttributes out;
        out.position = uniform.P * uniform.camera_transformation * uniform.trafo * va.position;
        Eigen::Vector3d N;
        N = va.N;

    
    
    

        // TODO: compute the correct lighting

        
        Vector3d lights_color(0.3, 0.3, 0.3);
        Vector3d p (va.position(0)/va.position(3), 
                    va.position(1)/va.position(3), 
                    va.position(2)/va.position(3));

    
            for (int i = 0; i < uniform.light_positions.size(); ++i){

            const Vector3d light_position = uniform.light_positions[i];
            const Vector3d &light_color = uniform.light_colors[i];
            const Vector3d Li = (light_position - p).normalized();

            Vector3d diff_color = uniform.obj_diffuse_color;

            // TODO: Add shading parameters

            // Diffuse contribution
            const Vector3d diffuse = diff_color * std::max(Li.dot(N), 0.0);
            // std::cout << std::max(Li.dot(N), 0.0)<<std::endl;

            // calculating the specular equation according to the basic shading component model equation
            // calcuatin v , l and h  ::   h = (v+l) / | v+l|
            const Vector3d v = (uniform.camera_position - p).normalized();
            const Vector3d l = (light_position - p).normalized();
            const Vector3d h = (v + l).normalized();

            const double specular3 = N.dot(h);
            const double specular1 = std::max(specular3, 0.);
            const double specular2 = std::pow(specular1, obj_specular_exponent);

            const Vector3d specular = uniform.obj_specular_color * specular2;

            // Attenuate lights according to the squared distance to the lights
            const Vector3d D = light_position - p;
            lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
        }

            
            out.color = lights_color;

            return out;

        
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: create the correct fragment
        return FragmentAttributes (va.color(0), va.color(1), va.color(2), 1,va.position(0),va.position(1),va.position(2),va.position(3));
        // return FragmentAttributes(1,0,0,1);

    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: implement the depth check

        if (fa.position[2] < previous.depth)
        {
            FrameBufferAttributes out(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
            out.depth = fa.position[2];
            return out;
        }
        else
            return previous;
    };
}

void flat_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);

    
    Eigen::Matrix4f trafo = compute_rotation(alpha);
    uniform.trafo = trafo;

    

    

    //TODO: compute the normals for flat vertex
    vector<VertexAttributes> vertex;
    for (int i = 0; i < facets.rows(); i++)
    {
        int v1 = facets(i, 0);
        int v2 = facets(i, 1);
        int v3 = facets(i, 2);

        Vector3d p1 = vertices.row(v1);
        Vector3d p2 = vertices.row(v2);
        Vector3d p3 = vertices.row(v3);

        VertexAttributes va0(p1[0], p1[1], p1[2]);
        VertexAttributes va1(p2[0], p2[1], p2[2]);
        VertexAttributes va2(p3[0], p3[1], p3[2]);

        Vector3d tri_normal;
        Vector3d A = (p3-p1);
        Vector3d B = (p2-p1);

        tri_normal = A.cross(B).normalized();

        va0.N = tri_normal;
        va1.N = tri_normal;
        va2.N = tri_normal;

        

        vertex.push_back(va0);
        vertex.push_back(va1);
        vertex.push_back(va2);
    }

    // TODO: set material colors
    uniform.obj_diffuse_color = obj_diffuse_color;
    uniform.obj_specular_color = obj_specular_color;
    uniform.light_colors = light_colors;
    uniform.light_positions = light_positions;


    rasterize_triangles(program, uniform, vertex, frameBuffer);
}

void pv_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);

    
    Eigen::Matrix4f trafo = compute_rotation(alpha);
    uniform.trafo = trafo;

  

    //TODO: compute the vertex normals as vertex normal average
     
    
    //TODO: create vertex attributes
    vector<VertexAttributes> vertex;

    MatrixXd v_normal(vertices.rows(),3);
    //setting all the rows to 0
    
    for(int i = 0; i< v_normal.rows();i++){
        v_normal(i,0) = 0;
        v_normal(i,1) = 0;
        v_normal(i,2) = 0;
    }
       
    for (int i = 0; i < facets.rows(); i++)
    {
        int v1 = facets(i, 0);
        int v2 = facets(i, 1);
        int v3 = facets(i, 2);

        Vector3d p1 = vertices.row(v1);
        Vector3d p2 = vertices.row(v2);
        Vector3d p3 = vertices.row(v3);

        VertexAttributes va0 (p1[0], p1[1], p1[2]);
        VertexAttributes va1 (p2[0], p2[1], p2[2]);
        VertexAttributes va2  (p3[0], p3[1], p3[2]);

        Vector3d tri_normal;
        Vector3d A = (p3 - p1);
        Vector3d B = (p2 - p1);

        tri_normal = A.cross(B).normalized();


        v_normal.row(v1) += tri_normal;
        v_normal.row(v2) += tri_normal;
        v_normal.row(v3) += tri_normal;
    }

    for (int i = 0; i < facets.rows(); i++)
    {
        int v1 = facets(i, 0);
        int v2 = facets(i, 1);
        int v3 = facets(i, 2);

        Vector3d p1 = vertices.row(v1);
        Vector3d p2 = vertices.row(v2);
        Vector3d p3 = vertices.row(v3);

        VertexAttributes va0  (p1[0], p1[1], p1[2]);
        VertexAttributes va1  (p2[0], p2[1], p2[2]);
        VertexAttributes va2  (p3[0], p3[1], p3[2]);

        va0.N = v_normal.row(v1).normalized();
        va1.N = v_normal.row(v2).normalized();
        va2.N = v_normal.row(v3).normalized();

        vertex.push_back(va0);
        vertex.push_back(va1);
        vertex.push_back(va2);
    }
    //TODO: set material colors
    uniform.obj_diffuse_color = obj_diffuse_color;
    uniform.obj_specular_color = obj_specular_color;
    uniform.light_colors = light_colors;
    uniform.light_positions = light_positions;

    rasterize_triangles(program, uniform, vertex, frameBuffer);
}

void wireframe_gif_render(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    const char *fileName = "wireframe.gif";
    vector<uint8_t> image;
    int delay = 25;
    GifWriter g;
    GifBegin(&g, fileName, frameBuffer.rows(), frameBuffer.cols(), delay);
    

    for (float i = 0; i < 25; i += 1)
    {
        frameBuffer.setConstant(FrameBufferAttributes());
        wireframe_render(i/25.0 * 2 * 3.14 , frameBuffer);
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }
    GifEnd(&g);
}

void flat_gif_render(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    const char *fileName = "flat.gif";
    vector<uint8_t> image;
    int delay = 25;
    GifWriter g;
    GifBegin(&g, fileName, frameBuffer.rows(), frameBuffer.cols(), delay);

    for (float i = 0; i < 35; i += 1)
    {
        frameBuffer.setConstant(FrameBufferAttributes());
        flat_shading(i / 25.0 * 2 * 3.14, frameBuffer);
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }
    GifEnd(&g);
}

void pv_gif_render(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    const char *fileName = "pv.gif";
    vector<uint8_t> image;
    int delay = 25;
    GifWriter g;
    GifBegin(&g, fileName, frameBuffer.rows(), frameBuffer.cols(), delay);

    for (float i = 0; i < 25; i += 1)
    {
        frameBuffer.setConstant(FrameBufferAttributes());
        pv_shading(i / 25.0 * 2 * 3.14, frameBuffer);
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }
    GifEnd(&g);
}

    int main(int argc, char *argv[])
    {
        setup_scene();

        int W = H * aspect_ratio;
        Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer(W, H);
        vector<uint8_t> image;
        

       // simple_render(frameBuffer);
       // framebuffer_to_uint8(frameBuffer, image);
       // stbi_write_png("simple.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

        //  wireframe_render(0, frameBuffer);
        //  framebuffer_to_uint8(frameBuffer, image);
        //  stbi_write_png("wireframe.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

        // flat_shading(0, frameBuffer);
        // framebuffer_to_uint8(frameBuffer, image);
        // stbi_write_png("flat_shading.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

       // pv_shading(0, frameBuffer);
       // framebuffer_to_uint8(frameBuffer, image);
       // stbi_write_png("pv_shading.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

        // TODO: add the animation
       // wireframe_gif_render(0,frameBuffer);
       // frameBuffer.setConstant(FrameBufferAttributes());

      // flat_gif_render(0, frameBuffer);

       //pv_gif_render(0, frameBuffer);

        // flat_shading(0, frameBuffer);
        // framebuffer_to_uint8(frameBuffer, image);
       //  stbi_write_png("flat_shading_perspective.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

         // pv_shading(0, frameBuffer);
        //  framebuffer_to_uint8(frameBuffer, image);
        //  stbi_write_png("pv_shading_perspective.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);
         return 0;
    }
