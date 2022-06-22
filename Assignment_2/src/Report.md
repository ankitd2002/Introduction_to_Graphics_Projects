
### Markdown file ###

## parta
## Ray tracing the parralleogram  ##


## I think of this concept as divinding the whole paralleogram into two different triangles 
# and figuring out of how a ray intersect with a triangle similar to the concept taught in the
# class

## Step 1: I set up the  Parameters of the parallelogram (position of the lower-left corner + two sides) 
#        pgram_origin , pgram_u,  pgram_v

#      a = pgram_origin , o = ray_origin  , d = ray_direction , p_u = pgram_u and p_v = pgram_v

## Step 2: we know the implicit form of the triangle is equal with vertices a,b,c is
#           f(u,v) = a + u(b-a) + v(c-a)  and the explicit form = p (t) = e + td  and the ray where the triangle interacts if
#            F(u,v) = p (t)  such that  0 + t d  = a + u P_u + v P_v
#     where u,v and t are unknown points such that t >= 0 and u,v >=0  and u,v <=1  is the point the ray intersects the triangle
# I created a matrix A with 3 x 3 size having values  A (0) = - pu    A(1) = -pv   A(2) = d  and a vector B such that b = a - o 

# Moreover I used the colPivHouseholder function of standard eigen library to solve the system of equations  Ak = b
# where K (0) = u,   K(1) = v   ,  and   K(2) = t



### Step 3: In order to calculate the intersection I did find the value of the ray intersection equation using 
#   Vector3d ray_intersection = ray_origin + t * ray_direction;
# then compute the value of the ray_intersection 

## Step 4: In order to calculate the normal of the intersection point , I did pgram_v.cross(pgram_u).normalized() which is a cross 
# product of two vectors p_v and p_u 


## part b  ray tracing with Perspective Projection 

# Step 5 Modify the ray-sphere intersection to follow the generic case we saw in class.

## I calculated the value of the equation  At^2 + B t + C = 0

# where a = d. d    b =  2 * d * (e - c)    c = (e-c) * (e -c )   d = b^2  - 4 * a * c

## using this equation from the textbook     (d · d)t^2 + 2d · (e − c)t + (e − c) · (e − c) − R^2 = 0

# where d is the he discriminant  and tells us how many real solutions there are. If the discriminant is negative,  its square root is imaginary and the line and sphere do not intersect. If the discriminant is positive, there are two # solutions: one solution where the ray enters  the sphere and one where it leaves. If the discriminant is zero, the ray grazes  the sphere, touching it at exactly one point

# t is calculated as  t = (- b - sqrt(d)) / 2 * a  and interaction point = ray_origin + t * ray_direction;

# if I checked if the value of d is d >= 0 then there is a intersection point in a sphere by the ray
# I checked ray normal as the difference of (interaction point - sphere center)


## Step 6: Modify the ray equation for perspective projection.

# For this I changed mine           
## // TODO: Prepare the ray (origin point and direction) // changing the ray equations according to the equations defined in class for  perspective projection 
# my ray_origin = camera_origin;   and   ray_direction = (pixel_center - camera_origin).normalized();   

