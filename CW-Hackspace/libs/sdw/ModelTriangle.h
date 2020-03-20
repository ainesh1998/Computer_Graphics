#include <glm/glm.hpp>
#include "Colour.h"
#include <string>
#include "TexturePoint.h"

class ModelTriangle
{
  public:
    glm::vec3 vertices[3];
    Colour colour;
    TexturePoint texturePoints[3];

    ModelTriangle()
    {
    }

    ModelTriangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, Colour trigColour)
    {
      vertices[0] = v0;
      vertices[1] = v1;
      vertices[2] = v2;
      colour = trigColour;
    }

    ModelTriangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, TexturePoint t1, TexturePoint t2, TexturePoint t3)
    {
      vertices[0] = v0;
      vertices[1] = v1;
      vertices[2] = v2;
      texturePoints[0] = t1;
      texturePoints[1] = t2;
      texturePoints[2] = t3;
    }
};

std::ostream& operator<<(std::ostream& os, const ModelTriangle& triangle)
{
    os << "(" << triangle.vertices[0].x << ", " << triangle.vertices[0].y << ", " << triangle.vertices[0].z << ")" << std::endl;
    os << "(" << triangle.vertices[1].x << ", " << triangle.vertices[1].y << ", " << triangle.vertices[1].z << ")" << std::endl;
    os << "(" << triangle.vertices[2].x << ", " << triangle.vertices[2].y << ", " << triangle.vertices[2].z << ")" << std::endl;
    os << std::endl;
    return os;
}
