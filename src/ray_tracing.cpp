#include "ray_tracing.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/geometric.hpp>
#include <glm/gtx/component_wise.hpp>
#include <glm/vector_relational.hpp>
DISABLE_WARNINGS_POP()
#include <cmath>
#include <iostream>
#include <limits>


// Returns true if the point p is inside the triangle spanned by v0, v1, v2 with normal n.
// Verify whether ray plane intersection is in triangle
// On edge == true!
bool pointInTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& n, const glm::vec3& p)
{

    // Check if plane equation P*n = d holds! (Should also hold for points on edges!)

    // 3x compute cross product between 2 vectors (containing p), to check direction of normal (check z-value)
    // In OpenGL, anti-clockwise cross product gives positive normal.
    glm::vec3 n1 = glm::normalize(glm::cross((p - v0), (v2 - v0)));
    glm::vec3 n2 = glm::normalize(glm::cross((p - v2), (v1 - v2)));
    glm::vec3 n3 = glm::normalize(glm::cross((p - v1), (v0 - v1)));

    //std::cout << "\n\n n1 = " << n1.x << " " << n1.y << " " << n1.z << std::endl;
    //std::cout << "\n n2 = " << n2.x << " " << n2.y << " " << n2.z << std::endl;
    //std::cout << "\n n3 = " << n3.x << " " << n3.y << " " << n3.z << std::endl;



    // If one of the normals is (0, 0, 0), P is on edge!
    if ((n1.x == 0 && n1.y == 0 && n1.z == 0) || (n2.x == 0 && n2.y == 0 && n2.z == 0) || (n3.x == 0 && n3.y == 0 && n3.z == 0)) {
        //std::cout << "\n\nIntersection point on edge (within triangle)!\n" << std::endl;
        return true;
    }
    // angles between all normals <= 90 degrees
    else if (glm::dot(n1, n2) >= 0 && glm::dot(n1, n3) >= 0 && glm::dot(n2, n3) >= 0) {  // All normals point in same dir
        //std::cout << "\n\nIntersection point within triangle!\n" << std::endl;
        return true;
    }

    return false;
}






// If ray does not intersect plane --> false
// If intersection found --> true + update ray.t!
bool intersectRayWithPlane(const Plane& plane, Ray& ray)
{
    // There is always an intersection except if d is orthogonal to n! Ray // plane
    if (glm::dot(ray.direction, plane.normal) == 0) {
        return false;
    }
    // solution
    float t = (plane.D - glm::dot(ray.origin, plane.normal)) / glm::dot(ray.direction, plane.normal);

    //std::cout << "\nCurrent t = " << ray.t << "\nComputed t = " << t << std::endl;


    // ------- INTERSECTION FOUND: but behind ray origin...

    if (t <= 0) {
        //std::cout << "Intersection behind ray origin...." << std::endl;
        return false;   // t == 0 --> ray origin ON plane
    }

    // ------- INTERSECTION FOUND: but only update ray.t if smaller than current t!

    // if P is closer to the ray origin then current intersection point (t is initialized as MAX_FLOAT)
    if (t < ray.t) {

        ray.t = t;

        //std::cout << "\nNew t smaller, so updated ray.t = " << ray.t << std::endl;

        return true;    // WE ONLY RETURN TRUE IF NEW INTERSECTION IS CLOSER THAN PREVIOUS ONE.
    }

    //std::cout << "\nNew t > current t, so no update of ray.t" << std::endl;

    return false;   // Even if new intersection is further than current one, return true.
}







Plane trianglePlane(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
{
    Plane plane;

    // cross product u x v = normal of plane (containing triangle)
    glm::vec3 u = v1 - v0;
    glm::vec3 v = v2 - v0;
    glm::vec3 n = glm::normalize(glm::cross(u, v));   // unit vector n!
    plane.normal = n;

    // d = dist to origin --> d = P * n
    // Plane = P * n = d ---> take any vertex as point P!
    float D = glm::dot(v0, n);
    plane.D = D;

    //std::cout << "\nComputed plane normal = " << n.x << " " << n.y << " " << n.z << "\nComputed plane's dist to origin D = " << D << std::endl;

    return plane;
}







// struct HitInfo {
//glm::vec3 normal;
//Material material;
//};



// --------------------------------------- These intersection methods are called by BVH


/// Input: the three vertices of the triangle
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, Ray& ray, HitInfo& hitInfo)
{

    Plane plane = trianglePlane(v0, v1, v2);

    // Roll back the ray.t value modified in the intersectRayWithPlane method if the point is not inside the triangle
    // So we store the old ray.t!
    float oldT = ray.t;

    //std::cout << "\nOld ray.t = " << oldT << std::endl;

    if (intersectRayWithPlane(plane, ray)) {  // ray intersects plane (but not necessarily within triangle!)

        float newT = ray.t;      // updated by intersectRayWithPlane() method!

        //std::cout << "\nNew plane intersection, new ray.t = " << newT << ", now check if in triangle:\n" << std::endl;

        glm::vec3 intersection = ray.origin + newT * ray.direction;

        // Check if intersection point is in triangle!
        if (!pointInTriangle(v0, v1, v2, plane.normal, intersection)) {

            /*            std::cout << "\n\nIntersection point with plane, but NOT in triangle! Resetting to old ray.t: " << oldT << std::endl;
                 */
            ray.t = oldT;                         // restore old t value!
            return false;
        }


        // After a successful intersection test, store normal and material in HitInfo!
        // material is set in BVH.intersect (hitInfo.material = mesh.material; )
        hitInfo.normal = plane.normal;


        std::cout << "---------------> TRIANGLE HIT: Updated ray.t from " << oldT << " to " << newT << std::endl;
        return true;      // now the new ray.t remains
    }

    return false;       // no intersection, so nothing changed
}







/// Input: a sphere with the following attributes: sphere.radius, sphere.center
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const Sphere& sphere, Ray& ray, HitInfo& hitInfo)
{

    // Find A, B and C for quadratic equation

    glm::vec3 unitRayDir = glm::normalize(ray.direction);  // Turn D into unit vector! (simplifies calculations)

    float A = glm::length(unitRayDir);                           // Should always give 1!
    float B = 2.0f * glm::dot(unitRayDir, ray.origin - sphere.center);              // B = 2 * D * (O-C)  
    float C = glm::dot(ray.origin - sphere.center, ray.origin - sphere.center) - sphere.radius * sphere.radius;   // C =  ||O-C|| - radius ^ 2

    float discriminant = B * B - 4.0f * A * C;

    std::cout << "\n ------------- A = " << A << "   B = " << B << "   C = " << C << "\nDiscriminant = " << discriminant << std::endl;

    float t0;     // 2 solutions t0 and t1
    float t1;
    float t;      // final solution: giving intersection closest to  ray origin (camPos)

    if (discriminant < 0) {

        std::cout << "D < 0! No intersection point with sphere!" << std::endl;
        return false;
    }
    else if (discriminant == 0) {

        std::cout << "D = 0! Ray tangent to sphere, so 1 intersection point!" << std::endl;

        t = -B / 2.0f * A;

        if (t <= 0) return false;   // Intersection behind origin of ray!

        if (t < ray.t) {     // If this t < current t, update ray.t! (1 ray can intersect many spheres!) 
            ray.t = t;
            glm::vec3 p = ray.origin + t * ray.direction;

            // set normal and material in hitInfo:
            hitInfo.material = sphere.material;
            hitInfo.normal = glm::normalize(p - sphere.center);  // tangent always hit from outside.

            std::cout << "\nSPHERE INTERSECTION: updated hitInfo normal at intersection " << p.x << " " << p.y << " " << p.z << " to " << hitInfo.normal.x << " " << hitInfo.normal.y << " " << hitInfo.normal.z << std::endl;

            return true;
        }
        else {
            return false;   // Else, nothing changed.
        }
    }
    else {
        // Compute t0, t1: there must be 2 solutions, cause D > 0
        t0 = (-B - sqrtf(discriminant)) / 2.0f * A;
        t1 = (-B + sqrtf(discriminant)) / 2.0f * A;

        // If both < 0, then intersection is BEHIND the ray origin!
        if (t0 <= 0 && t1 <= 0) return false;

        // If ray origin INSIDE sphere, 1 t will be pos and the other negative! --> take the intersection IN FRONT of ray origin!
        bool hitFromInside = false;
        if (t0 <= 0 && t1 > 0) {
            t = t1;
            hitFromInside = true;
            std::cout << "\nSphere hit from inside!" << std::endl;
        }
        else if (t1 <= 0 && t0 > 0) {
            t = t0;
            hitFromInside = true;
            std::cout << "\nSphere hit from inside!" << std::endl;
        }
        // If 2 intersection points t, choose point t closest to ray origin!  
        else if (t0 < t1) {
            t = t0;
        }
        else {
            t = t1;
        }
        std::cout << "t0 = " << t0 << "  t1 = " << t1 << " ---------- Final t: " << t << std::endl;

        if (t < ray.t) {      // If final t < current t, update ray.t
            std::cout << "---------------> Updated ray.t from " << ray.t << " to t_in " << t << std::endl;
            ray.t = t;

            glm::vec3 p = ray.origin + t * ray.direction;

            // set normal and material in hitInfo:
            hitInfo.material = sphere.material;
            if (!hitFromInside) {
                hitInfo.normal = glm::normalize(p - sphere.center);
            }
            else {
                hitInfo.normal = glm::normalize(sphere.center - p); // inwards normal!
            }

            std::cout << "\nSPHERE INTERSECTION: updated hitInfo normal to " << hitInfo.normal.x << " " << hitInfo.normal.y << " " << hitInfo.normal.z << std::endl;

            return true;
        }
        else {
            return false;    // ray.t unchanged
        }
    }
}






bool comp(float a, float b)
{
    return (a < b);
}



/// Input: an axis-aligned bounding box with the following parameters: minimum coordinates box.lower and maximum coordinates box.upper
/// Output: if intersects then modify the hit parameter ray.t and return true, otherwise return false
bool intersectRayWithShape(const AxisAlignedBox& box, Ray& ray)
{
    std::cout << "\nRay dir = " << ray.direction.x << " " << ray.direction.y << " " << ray.direction.z << std::endl;

    // Find t_min and t_max for each of the 3 axes

     // If DIVISION BY 0  <----- 1 or 2 of the ray directions is 0, so goes along axis.
    // We wanna ignore that plane! (disregard Nan value) ---> one of them should be FLOAT_MIN and the other FLOAT_MAX

    float t_min_x;
    float t_max_x;
    float t_min_y;
    float t_max_y;
    float t_min_z;
    float t_max_z;
    if (ray.direction.x != 0) {
        t_min_x = (box.lower.x - ray.origin.x) / ray.direction.x;
        t_max_x = (box.upper.x - ray.origin.x) / ray.direction.x;
    }
    else {
        t_min_x = std::numeric_limits<float>::min();
        t_max_x = std::numeric_limits<float>::max();
        if (ray.origin.x < box.lower.x || ray.origin.x > box.upper.x) return false;  // if not within range of box, intersection impossible!
    }
    if (ray.direction.y != 0) {
        t_min_y = (box.lower.y - ray.origin.y) / ray.direction.y;
        t_max_y = (box.upper.y - ray.origin.y) / ray.direction.y;
    }
    else {
        t_min_y = std::numeric_limits<float>::min();
        t_max_y = std::numeric_limits<float>::max();
        if (ray.origin.y < box.lower.y || ray.origin.y > box.upper.y) return false;
    }
    if (ray.direction.z != 0) {
        t_min_z = (box.lower.z - ray.origin.z) / ray.direction.z;
        t_max_z = (box.upper.z - ray.origin.z) / ray.direction.z;
    }
    else {
        t_min_z = std::numeric_limits<float>::min();
        t_max_z = std::numeric_limits<float>::max();
        if (ray.origin.z < box.lower.z || ray.origin.z > box.upper.z) return false;
    }



    // Find t_in and t_out for the 3 axes
    float t_in_x = glm::min(t_min_x, t_max_x);  // If min = FLOAT_MIN, it will be ignored, cause later we take max of all in-points! 
    float t_out_x = glm::max(t_min_x, t_max_x);  // If max = FLOAT_MAX, it will be ignored, cause later we take min of all out-points! 
    float t_in_y = glm::min(t_min_y, t_max_y);
    float t_out_y = glm::max(t_min_y, t_max_y);
    float t_in_z = glm::min(t_min_z, t_max_z);
    float t_out_z = glm::max(t_min_z, t_max_z);

    std::cout << "t_in_x = " << t_in_x << ", t_out_x = " << t_out_x << std::endl;
    std::cout << "t_in_y = " << t_in_y << ", t_out_y = " << t_out_y << std::endl;
    std::cout << "t_in_z = " << t_in_z << ", t_out_z = " << t_out_z << std::endl;

    // Find global t_in and t_out!
    float t_in = std::max({ t_in_x, t_in_y, t_in_z }, comp);  // Smallest t where we crossed all in-points
    float t_out = std::min({ t_out_x, t_out_y, t_out_z }, comp);  // Smallest t where we crossed at least 1 out-point


    std::cout << "Global t_in = " << t_in << std::endl;
    std::cout << "Global t_out = " << t_out << std::endl;


    // Check validity: ray misses or intersection is BEHIND ray origin!
    if (t_in > t_out || t_out < 0) {
        std::cout << "Ray misses box or intersection behind ray origin." << std::endl;
        return false;    // ray.t unchanged.
    }
    else if (t_in < 0 && t_out > 0 && t_out < ray.t) {        // Ray origin IN BOX! 
        std::cout << "---------------> AABB BOX: Updated ray.t from " << ray.t << " to t_in " << t_in << std::endl;
        ray.t = t_out;
        return true;
    }
    else if (t_in < ray.t) {     // check if intersection is closer to current one stored in ray! 
        std::cout << "---------------> AABB BOX: Updated ray.t from " << ray.t << " to t_in " << t_in << std::endl;
        ray.t = t_in;
        return true;
    }
    std::cout << "Box intersected, but t > current ray.t " << ray.t << std::endl;
    return false;
}
