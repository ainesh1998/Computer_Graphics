#include <glm/glm.hpp>
#include <iostream>


class RayTriangleIntersection
{
  public:
    glm::vec3 intersectionPoint;
    float distanceFromCamera;
    ModelTriangle intersectedTriangle;
    glm::vec3 solution;

    RayTriangleIntersection()
    {
    }

    RayTriangleIntersection(glm::vec3 point, float distance, ModelTriangle triangle)
    {
        intersectionPoint = point;
        distanceFromCamera = distance;
        intersectedTriangle = triangle;
        solution = glm::vec3(-1,-1,-1);
    }

    RayTriangleIntersection(glm::vec3 point, float distance, ModelTriangle triangle, glm::vec3 possibleSolution)
    {
        intersectionPoint = point;
        distanceFromCamera = distance;
        intersectedTriangle = triangle;
        solution = possibleSolution;
    }
};

std::ostream& operator<<(std::ostream& os, const RayTriangleIntersection& intersection)
{
    // os << "Intersection is at " << intersection.intersectionPoint << " on triangle " << intersection.intersectedTriangle << " at a distance of " << intersection.distanceFromCamera << std::endl;
    os << "Intersection is at " << "(" << intersection.intersectionPoint.x << "," << intersection.intersectionPoint.y <<"," << intersection.intersectionPoint.z <<")" << " on triangle "
    << intersection.intersectedTriangle << " at a distance of " << intersection.distanceFromCamera << std::endl;

    return os;
}
