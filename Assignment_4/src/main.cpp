////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <fstream>
#include <algorithm>
#include <numeric>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Class to store tree
////////////////////////////////////////////////////////////////////////////////
class AABBTree
{
public:
    class Node
    {
    public:
        AlignedBox3d bbox;
        int parent;   // Index of the parent node (-1 for root)
        int left;     // Index of the left child (-1 for a leaf)
        int right;    // Index of the right child (-1 for a leaf)
        int triangle; // Index of the node triangle (-1 for internal nodes)
    };

    std::vector<Node> nodes;
    int root;

    AABBTree() = default;                           // Default empty constructor
    AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh
    
};

////////////////////////////////////////////////////////////////////////////////
// Scene setup, global variables
////////////////////////////////////////////////////////////////////////////////
const std::string data_dir = DATA_DIR;
const std::string filename("raytrace.png");
const std::string mesh_filename(data_dir + "bunny.off");

//Camera settings
const double focal_length = 2;
const double field_of_view = 0.7854; //45 degrees
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 2);

// Maximum number of recursive calls
const int max_bounce = 5;

// Triangle Mesh
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)
AABBTree bvh;

// Objects
std::vector<Vector3d> sphere_centers;
std::vector<double> sphere_radii;
std::vector<Matrix3d> parallelograms;

//Material for the object, same material for all objects
const Vector4d obj_ambient_color(0.0, 0.5, 0.0, 0);
const Vector4d obj_diffuse_color(0.5, 0.5, 0.5, 0);
const Vector4d obj_specular_color(0.2, 0.2, 0.2, 0);
const double obj_specular_exponent = 256.0;
const Vector4d obj_reflection_color(0.7, 0.7, 0.7, 0);

// Precomputed (or otherwise) gradient vectors at each grid node
const int grid_size = 20;
std::vector<std::vector<Vector2d>> grid;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector4d> light_colors;
//Ambient light
const Vector4d ambient_light(0.2, 0.2, 0.2, 0);

//Fills the different arrays
void setup_scene()
{
    //Loads file
    std::ifstream in(mesh_filename);
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

    // Spheres
    sphere_centers.emplace_back(10, 0, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(7, 0.05, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(4, 0.1, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(1, 0.2, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-2, 0.4, 1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-5, 0.8, -1);
    sphere_radii.emplace_back(1);

    sphere_centers.emplace_back(-8, 1.6, 1);
    sphere_radii.emplace_back(1);

    // parallelograms
    parallelograms.emplace_back();
    parallelograms.back() << -100, 100, -100,
        -1.25, 0, -1.2,
        -100, -100, 100;

    //setup tree
    bvh = AABBTree(vertices, facets);

    //Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);
}
// We need to make this function visible
Vector4d shoot_ray(const Vector3d &ray_origin, const Vector3d &ray_direction, int max_bounce);

////////////////////////////////////////////////////////////////////////////////
// BVH Code
////////////////////////////////////////////////////////////////////////////////

AlignedBox3d bbox_from_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c)
{
    AlignedBox3d box;
    box.extend(a);
    box.extend(b);
    box.extend(c);
    return box;
}



AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F)
{
    // Compute the centroids of all the triangles in the input mesh
    MatrixXd centroids(F.rows(), V.cols());
    centroids.setZero();
    for (int i = 0; i < F.rows(); ++i)
    {
        for (int k = 0; k < F.cols(); ++k)
        {
            centroids.row(i) += V.row(F(i, k));
        }
        centroids.row(i) /= F.cols();
    }

    // TODO

    // Top-down approach.
    // Split each set of primitives into 2 sets of roughly equal size,
    // based on sorting the centroids along one direction or another.
    // list of triangles _indies = [0....n]
    // build i , j, parallel


    
    
    
    }

////////////////////////////////////////////////////////////////////////////////
// Intersection code
////////////////////////////////////////////////////////////////////////////////

// Compute the intersection between a ray and a sphere, return -1 if no intersection
double ray_sphere_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, int index, Vector3d &p, Vector3d &N)
{
    // TODO, implement the intersection between the ray and the sphere at index index.
    // return t or -1 if no intersection

    double t = 0;

    // solving the quadratic equation of at^2 + b t + c = 0
    const double a = ray_direction.dot(ray_direction);                                                                                             // a =  (d.d)
    const double b = 2 * ray_direction.dot((ray_origin - sphere_centers[index]));                                                                  // b = 2 x d x (e-c)
    const double c = ((ray_origin - sphere_centers[index]).dot((ray_origin - sphere_centers[index]))) - sphere_radii[index] * sphere_radii[index]; // c = (e-c) x (e-c)
    const double d = (b * b - 4 * a * c);

    if (d < 0)
    {
        return -1;
    }
    else
    {
        // TODO set the correct intersection point, update p to the correct value
        //  The ray hit the sphere, compute the exact intersection point
        t = (-b - sqrt(d)) / (2 * a);

        const Vector3d intersection = ray_origin + t * ray_direction;
        const Vector3d ray_normal = (intersection - sphere_centers[index]).normalized();

        p = intersection;
        N = ray_normal;

        return t;
    }

    return -1;
}

// Compute the intersection between a ray and a paralleogram, return -1 if no intersection
double ray_parallelogram_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, int index, Vector3d &p, Vector3d &N)
{
    // TODO, implement the intersection between the ray and the parallelogram at index index.
    // return t or -1 if no intersection

    // double t = 0;

    const Vector3d pgram_origin = parallelograms[index].col(0);
    const Vector3d A = parallelograms[index].col(1);
    const Vector3d B = parallelograms[index].col(2);
    const Vector3d pgram_u = A - pgram_origin;
    const Vector3d pgram_v = B - pgram_origin;

    MatrixXd a(3, 3);

    a.col(0) = -pgram_u;
    a.col(1) = -pgram_v;
    a.col(2) = ray_direction;

    const Vector3d b = (parallelograms[index].col(0) - ray_origin); // computes the value of vector3d

    const Vector3d k = a.colPivHouseholderQr().solve(b); // solve the value of k  in the form  k = (A)(inverse) (B)
    const double u = k[0];
    const double v = k[1];
    const double t = k[2];

    if (t < 0)
    {
        return -1;
    }

    if (t >= 0 && u >= 0 && v >= 0 && u <= 1 && v <= 1)
    {
        Vector3d ray_intersection = ray_origin + t * ray_direction;
        Vector3d ray_normal = pgram_v.cross(pgram_u).normalized();

        p = ray_intersection; // p = index;
        N = ray_normal;

        return t;
    }
    return -1;
}


double ray_triangle_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &a, const Vector3d &b, const Vector3d &c, Vector3d &p, Vector3d &N)
{
    // TODO
    // Compute whether the ray intersects the given triangle.
    // If you have done the parallelogram case, this should be very similar to it.

    MatrixXd a1(3, 3); // Assigns the corresponding paralloegram variables to matrix a
    a1.col(0) = -(b-a);
    a1.col(1) = -(c-a);
    a1.col(2) = ray_direction;

    const Vector3d b1 = (a - ray_origin); // computes the value of vector3d

    const Vector3d k = a1.colPivHouseholderQr().solve(b1);

    const double u = k[0];
    const double v = k[1];
    const double t = k[2];

    if (t < 0)
    {
        return -1;
    }

    if (t >= 0 && u >= 0 && v >= 0 && u + v <= 1)
    {
        Vector3d ray_intersection = ray_origin + t * ray_direction;
        Vector3d ray_normal = (b-a).cross(c-a).normalized();

        p = ray_intersection; // p = index;
        N = ray_normal;

        return t;
    }
    return -1;
}

bool ray_box_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const AlignedBox3d &box)
{
    // TODO
    // Compute whether the ray intersects the given box.
    // we are not testing with the real surface here anyway.

    const double x_min = box.min()(0);
    const double y_min = box.min()(1);
    const double z_min = box.min()(2);

    const double x_max = box.max()(0);
    const double y_max = box.max()(1);
    const double z_max = box.max()(2);

    const double txmin = (x_min - ray_origin(0)) / ray_direction(0);     
    const double txmax = (x_max - ray_origin(0))  /ray_direction(0);
    const double tymin = (y_min - ray_origin(1)) /ray_direction(1);
    const double tymax = (y_max - ray_origin(1)) /ray_direction(1);
    if ((txmin > tymax) || (tymin > txmax))
     { return false; }
    
    return true;
    
}

//Finds the closest intersecting object returns its index
//In case of intersection it writes into p and N (intersection point and normals)
bool find_nearest_object(const Vector3d &ray_origin, const Vector3d &ray_direction, Vector3d &p, Vector3d &N)
{
    Vector3d tmp_p, tmp_N;
    bool has_intersection = false;
    int closest_index = -1;
    double closest_t = std::numeric_limits<double>::max(); // closest t is "+ infinity"

    for (int i = 0; i < sphere_centers.size(); ++i)
    {
        // returns t and writes on tmp_p and tmp_N
        const double t = ray_sphere_intersection(ray_origin, ray_direction, i, tmp_p, tmp_N);
        // We have intersection
        if (t >= 0)
        {
            // The point is before our current closest t
            if (t < closest_t)
            {
                closest_index = i;
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
                has_intersection = true;
            }
        }
    }

    for (int i = 0; i < parallelograms.size(); ++i)
    {
        // returns t and writes on tmp_p and tmp_N
        const double t = ray_parallelogram_intersection(ray_origin, ray_direction, i, tmp_p, tmp_N);
        // We have intersection
        if (t >= 0)
        {
            // The point is before our current closest t
            if (t < closest_t)
            {
                closest_index = sphere_centers.size() + i;
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
                has_intersection = true;
            }
        }
    }

        // TODO
    // Method (1): Traverse every triangle and return the closest hit.

    
    for (int i = 0; i<facets.rows();  i++)
    {
         int v1 = facets (i,0);
         int v2 = facets (i,1);
         int v3 = facets (i,2);

         Vector3d p1 = vertices.row(v1);
         Vector3d p2 = vertices.row(v2);
         Vector3d p3 = vertices.row(v3);

        double t = ray_triangle_intersection(ray_origin,ray_direction,p1,p2,p3,tmp_p,tmp_N);
        // We have intersection
        if (t >= 0)
        {
            // The point is before our current closest t
            if (t < closest_t)
            {
                // std::cout<<t<<std::endl;
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
                has_intersection = true;
            }
        }
    }


    // Method (2): Traverse the BVH tree and test the intersection with a
    // triangles at the leaf nodes that intersects the input ray.


    return has_intersection;
}
// Checks if the light is visible
bool is_light_visible(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &light_position)
{
    // TODO: Determine if the light is visible here
    // Use find_nearest_object -> index, p , (N), use index to check if something intersects
    Vector3d p, N;

    bool index = find_nearest_object(ray_origin, ray_direction, p, N);
    if (index == false)
    {
        return true;
    }
    const double dp = (ray_origin - p).norm();
    const double dl = (ray_origin - light_position).norm();

    if (dp < dl)
    {
        return false;
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Raytracer code
////////////////////////////////////////////////////////////////////////////////

Vector4d shoot_ray(const Vector3d &ray_origin, const Vector3d &ray_direction,int max_bounce)
{
    //Intersection point and normal, these are output of find_nearest_object
    Vector3d p, N;

    const bool nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

    if (!nearest_object)
    {
        // Return a transparent color
        return Vector4d(0, 0, 0, 0);
    }

    // Ambient light contribution
    const Vector4d ambient_color = obj_ambient_color.array() * ambient_light.array();

    // Punctual lights contribution (direct lighting)
    Vector4d lights_color(0, 0, 0, 0);
    for (int i = 0; i < light_positions.size(); ++i)
    {
        const Vector3d &light_position = light_positions[i];
        const Vector4d &light_color = light_colors[i];
        const Vector3d Li = (light_position - p).normalized();

                // TODO: Shoot a shadow ray to determine if the light should affect the intersection point and call is_light_visible
        Vector3d shadow_ray = p + Li * 1e-4;
        const Vector3d L = (light_position - p).normalized();
        bool b = is_light_visible(shadow_ray, L, light_position);
        if (b == false)
        {
            continue;
        }

        Vector4d diff_color = obj_diffuse_color;

        if (nearest_object == true)
        {
            // Compute UV coodinates for the point on the sphere
            const double x = p(0) - sphere_centers[nearest_object][0];
            const double y = p(1) - sphere_centers[nearest_object][1];
            const double z = p(2) - sphere_centers[nearest_object][2];
            const double tu = acos(z / sphere_radii[nearest_object]) / M_PI;
            const double tv = (M_PI + atan2(y, x)) / (2 * M_PI);

           // diff_color = procedural_texture(tu, tv);
        }

        // TODO: Add shading parameters

        // Diffuse contribution
        
        const Vector4d diffuse = diff_color * std::max(Li.dot(N), 0.0);

        // Specular contribution
        const Vector3d Hi = (Li - ray_direction).normalized();
        const Vector4d specular = obj_specular_color * std::pow(std::max(N.dot(Hi), 0.0), obj_specular_exponent);
        // Vector3d specular(0, 0, 0);

        // Attenuate lights according to the squared distance to the lights
        const Vector3d D = light_position - p;
        lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
    }

    Vector4d refl_color = obj_reflection_color;
    if (nearest_object == true)
    {
        refl_color = Vector4d(0.5, 0.5, 0.5, 0);
    }
    // TODO: Compute the color of the reflected ray and add its contribution to the current point color.
    // use refl_color

    Vector3d ray_dir = ((2 * N * (N.dot(ray_direction)) - ray_direction) * -1).normalized();
    Vector4d reflection_color(0, 0, 0, 0);

    if (max_bounce > 0)
     {
        reflection_color = shoot_ray((p + (1e-4 * ray_dir)), ray_dir, max_bounce - 1);
        reflection_color[0] = reflection_color[0] * refl_color[0];
        reflection_color[1] = reflection_color[1] * refl_color[1];
        reflection_color[2] = reflection_color[2] * refl_color[2];
    }

    // Rendering equation
    Vector4d C = ambient_color + lights_color + reflection_color;

    //Set alpha to 1
    C(3) = 1;

    return C;
}

////////////////////////////////////////////////////////////////////////////////

void raytrace_scene()
{
    std::cout << "Simple ray tracer." << std::endl;

    int w = 640;
    int h = 480;
    MatrixXd R = MatrixXd::Zero(w, h);
    MatrixXd G = MatrixXd::Zero(w, h);
    MatrixXd B = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

    // The camera always points in the direction -z
    // The sensor grid is at a distance 'focal_length' from the camera center,
    // and covers an viewing angle given by 'field_of_view'.
    double aspect_ratio = double(w) / double(h);
    //TODO
    double h1 = focal_length * tan(field_of_view / 2);
    double image_y = h1;                     // y=  h = f * tan (0/2) g //TODO: compute the correct pixels size // given the top focal length f and angle given at the top
    double image_x = aspect_ratio * image_y; // apsect ratio * y //TODO: compute the correct pixels size

    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    const Vector3d image_origin(-image_x, image_y, camera_position[2] - focal_length);
    const Vector3d x_displacement(2.0 / w * image_x, 0, 0);
    const Vector3d y_displacement(0, -2.0 / h * image_y, 0);

    for (unsigned i = 0; i < w; ++i)
    {
        for (unsigned j = 0; j < h; ++j)
        {
            const Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;

            // Prepare the ray
            Vector3d ray_origin;
            Vector3d ray_direction;

            if (is_perspective)
            {
                // Perspective camera
                ray_origin = camera_position;
                ray_direction = (pixel_center - camera_position).normalized();
            }
            else
            {
                // Orthographic camera
                ray_origin = pixel_center;
                ray_direction = Vector3d(0, 0, -1);
            }

            const Vector4d C = shoot_ray(ray_origin, ray_direction,max_bounce);
            R(i, j) = C(0);
            G(i, j) = C(1);
            B(i, j) = C(2);
            A(i, j) = C(3);
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    setup_scene();

    raytrace_scene();
    return 0;
}
