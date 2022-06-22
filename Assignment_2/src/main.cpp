// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

void raytrace_sphere()
{
    std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

    const std::string filename("sphere_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the
    // unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);


    // prepare the sphere setting the center and sphere radius for the generic case (read me point 5)
    const Vector3d sphere_center(0, 0, 0);
    const double sphere_radius = 0.9;

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            // Intersect with the sphere
            // NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
            //Vector2d ray_on_xy(ray_origin(0), ray_origin(1));//
            //const Vector3d sphere_center(0, 0, 0);
            //const double sphere_radius = 0.9;

            // solving the quadratic equation of at^2 + b t + c = 0
            const double a = ray_direction.dot(ray_direction);  // a =  (d.d)
            const double b =  2* ray_direction.dot((ray_origin - sphere_center));  // b = 2 x d x (e-c)
            const double c = ((ray_origin - sphere_center).dot((ray_origin - sphere_center))) - sphere_radius * sphere_radius;  // c = (e-c) x (e-c)
            const double d = (b * b - 4 * a * c);  // d = b^2 - 4 x a x c 
            
            if (d >= 0)  // check if the value of the discriminate is greater than 0  it checks the given a ray corrdinates lies inside the bounds of the sphere
            {

                // The ray hit the sphere, compute the exact intersection point 
                const double t = (-b - sqrt(d)) / (2 * a);

                // Compute normal at the intersection point
                const Vector3d intersection = ray_origin + t * ray_direction;
                const Vector3d ray_normal = (intersection - sphere_center).normalized();

                // Simple diffuse model
                C(i, j) = (light_position - intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}


// Make a function which makes a matrix of 3 x 3 size and a vector b and solve the linear system of equations of
// the form  Ak = b
Vector3d raytrace_intersect(const Vector3d pgram_origin, const Vector3d pgram_u, const Vector3d pgram_v,
                            const Vector3d ray_origin, const Vector3d ray_direction)

{
    MatrixXd a(3, 3);   // Assigns the corresponding paralloegram variables to matrix a
    a.col(0) = -pgram_u;
    a.col(1) = -pgram_v;
    a.col(2) = ray_direction;

    const Vector3d b = (pgram_origin - ray_origin);  // computes the value of vector3d

    const Vector3d k = a.colPivHouseholderQr().solve(b);  // solve the value of k  in the form  k = (A)(inverse) (B)
    //std::cout << "The solution is:\n"  << k << std::endl;

    // const double u = k[0];
    // const double v = k[1];
    // const double t = k[2];

    return k; // returns the vector k which contains the value of u,v and t
}

void raytrace_parallelogram()
{
    std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

    const std::string filename("plane_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            // calling the ray trace intersect function and assigning k vector into its corresponding values u , v and t
            const Vector3d k = raytrace_intersect(pgram_origin, pgram_u, pgram_v, ray_origin, ray_direction);
            const double u = k[0];
            const double v = k[1];
            const double t = k[2];

             

            // TODO: Check if the ray intersects with the parallelogram
            if (t >= 0 && u >= 0 && v >= 0   && u <=1  && v <=1 )
            {
                // TODO: The ray hit the parallelogram, compute the exact intersection
                // point
                Vector3d ray_intersection = ray_origin + t * ray_direction;

                // TODO: Compute normal at the intersection point
               // Vector3d ray_normal = ray_intersection.normalized();
                Vector3d ray_normal = pgram_v.cross(pgram_u).normalized();
                

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_perspective()
{
    std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

    const std::string filename("plane_perspective.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // TODO: Prepare the ray (origin point and direction) // changing the ray equations according to the equations defined in class for
            // perspective projection 
            const Vector3d ray_origin = camera_origin;  // pixel_center //
            const Vector3d ray_direction = (pixel_center - camera_origin).normalized();   // camera_view_direction;
        
           // following the same code here as I did in the raytrace parralleogram above //
            const Vector3d k = raytrace_intersect(pgram_origin, pgram_u, pgram_v, ray_origin, ray_direction);
            const double u = k[0];
            const double v = k[1];
            const double t = k[2];

            // TODO: Check if the ray intersects with the parallelogram
            if (t >= 0 && u >= 0 && v >= 0 && u <= 1 && v <= 1)
            {
                // TODO: The ray hit the parallelogram, compute the exact intersection
                // point
                Vector3d ray_intersection = ray_origin + t * ray_direction;

                // TODO: Compute normal at the intersection point
                Vector3d ray_normal = pgram_v.cross(pgram_u).normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_shading()
{
    std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

    const std::string filename("shading.png");

    // Three different matrices storing the color and Add RGB components instead of the current grey-scale one.
    MatrixXd R = MatrixXd::Zero(800, 800); // Store the color red color matrix
    MatrixXd G = MatrixXd::Zero(800, 800); // Store the color green color matrix
    MatrixXd B = MatrixXd::Zero(800, 800); // Store the color blue color matrix

    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    // defining the camera origin and the camera view direction 
    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / A.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / A.rows(), 0);

    //Sphere setup
    const Vector3d sphere_center(0, 0, 0);
    const double sphere_radius = 0.9;

    //material params
    const Vector3d diffuse_color(1, 0, 1);
    const double specular_exponent = 100;
    const Vector3d specular_color(0., 0, 1);

    // Single light source
    const Vector3d light_position(-1, 1, 1);
    double ambient = 0.1;

    for (unsigned i = 0; i < R.cols(); ++i)
    {
        for (unsigned j = 0; j < R.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            // Intersect with the sphere
            // NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
            Vector2d ray_on_xy(ray_origin(0), ray_origin(1));
            const double sphere_radius = 0.9;

            if (ray_on_xy.norm() < sphere_radius)
            {
                // The ray hit the sphere, compute the exact intersection point
                Vector3d ray_intersection(
                    ray_on_xy(0), ray_on_xy(1),
                    sqrt(sphere_radius * sphere_radius - ray_on_xy.squaredNorm()));

                // Compute normal at the intersection point
                Vector3d ray_normal = ray_intersection.normalized();

                // TODO: Add shading parameter here

                // calculating the diffuse equation according to the basic shading component model equation 
                const double diffuse = (light_position - ray_intersection).normalized().dot(ray_normal);
                const double diffuse1 = std::max(diffuse,0.);

                // calculating the specular equation according to the basic shading component model equation
                // calcuatin v , l and h  ::   h = (v+l) / | v+l| 
                const Vector3d v = (camera_origin- ray_intersection).normalized();
                const Vector3d l = (light_position - ray_intersection).normalized();
                const Vector3d h = (v+l).normalized();

                // const double specular = std::pow(h.dot(ray_normal),specular_exponent); // ?? the specular equation is not correct
                // calculating the diffuse equation according to the basic shading component model equation
                const double specular = ray_normal.dot(h);
                const double specular1 = std::max(specular,0.);
                const double specular2 = std::pow(specular1, specular_exponent);
            

                // Simple diffuse model by adding the ambinent , specular and diffuse equations to the model
                R(i, j) = ambient + diffuse1 * diffuse_color[0] + specular2 * specular_color[0];
                G(i, j) = ambient+ diffuse1 * diffuse_color[1] + specular2 * specular_color[1];
                B(i, j) = ambient+ diffuse1 * diffuse_color[2] + specular2 * specular_color[2];

                // Clamp to zero
                R(i, j) = std::max(R(i, j), 0.);
                G(i, j) = std::max(G(i, j), 0.);
                B(i, j) = std::max(B(i, j), 0.);

                

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

int main()
{
   raytrace_sphere();
   raytrace_parallelogram();
   raytrace_perspective();
   raytrace_shading();

    return 0;
}
