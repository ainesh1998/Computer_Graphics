#include <glm/glm.hpp>
#include "Colour.h"
#include <string>
#include "TexturePoint.h"
#include "BumpPoint.h"
#include <vector>
class ModelTriangle
{
  public:
    int ID;
    glm::vec3 vertices[3];
    Colour colour;
    TexturePoint texturePoints[3];
    BumpPoint bumpPoints[3];
    int boundingBoxIndex;
    bool isMirror = false;
    bool isTexture = false;
    bool isBump = false;
    bool isGlass = false;
    bool isMetal = false;
    bool isSpecular = false;
    float velocity = 0;
    float reflectivity = 0;
    float roughness = 0;

    //Texture data
    int textureIndex;
    int bumpIndex;

    ModelTriangle()
    {
        isTexture = false;
        ID = -1;
    }

    ModelTriangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, Colour trigColour, int triangleID)
    {
      vertices[0] = v0;
      vertices[1] = v1;
      vertices[2] = v2;
      colour = trigColour;
      texturePoints[0] = TexturePoint(-1,-1);
      texturePoints[1] = TexturePoint(-1,-1);
      texturePoints[2] = TexturePoint(-1,-1);
      isTexture = false;
      ID = triangleID;
    }

    ModelTriangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, TexturePoint t1, TexturePoint t2, TexturePoint t3, int triangleID)
    {
      vertices[0] = v0;
      vertices[1] = v1;
      vertices[2] = v2;
      texturePoints[0] = t1;
      texturePoints[1] = t2;
      texturePoints[2] = t3;
      bumpPoints[0] = BumpPoint(-1,-1);
      bumpPoints[1] = BumpPoint(-1,-1);
      bumpPoints[2] = BumpPoint(-1,-1);
      isTexture = true;
      ID = triangleID;
    }

    ModelTriangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, BumpPoint b1, BumpPoint b2, BumpPoint b3, int triangleID)
    {
      vertices[0] = v0;
      vertices[1] = v1;
      vertices[2] = v2;
      texturePoints[0] = TexturePoint(-1,-1);
      texturePoints[1] = TexturePoint(-1,-1);
      texturePoints[2] = TexturePoint(-1,-1);
      bumpPoints[0] = b1;
      bumpPoints[1] = b2;
      bumpPoints[2] = b3;
      isBump = true;
      ID = triangleID;
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
