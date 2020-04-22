#include <iostream>

class BoundingBox
{
  public:
    glm::vec3 startVertex; // backBottomLeft
    float width;
    float height;
    float depth;

    BoundingBox(glm::vec3 start, float w, float h, float d)
    {
      startVertex = start;
      width = w;
      height = h;
      depth = d;
    }

    glm::vec3 getBackBottomRight() {
        return glm::vec3(startVertex.x+width, startVertex.y, startVertex.z);
    }

    glm::vec3 getBackTopLeft() {
        return glm::vec3(startVertex.x, startVertex.y+height, startVertex.z);
    }

    glm::vec3 getBackTopRight() {
        return glm::vec3(startVertex.x+width, startVertex.y+height, startVertex.z);
    }

    glm::vec3 getFrontBottomLeft() {
        return glm::vec3(startVertex.x, startVertex.y, startVertex.z+depth);
    }

    glm::vec3 getFrontBottomRight() {
        return glm::vec3(startVertex.x+width, startVertex.y, startVertex.z+depth);
    }

    glm::vec3 getFrontTopLeft() {
        return glm::vec3(startVertex.x, startVertex.y+height, startVertex.z+depth);
    }

    glm::vec3 getFrontTopRight() {
        return glm::vec3(startVertex.x+width, startVertex.y+height, startVertex.z+depth);
    }

    std::vector<glm::vec3> getPoints() {
        std::vector<glm::vec3> points;
        points.push_back(startVertex);
        points.push_back(getFrontTopRight());
        points.push_back(getFrontTopLeft());
        points.push_back(getFrontBottomRight());
        points.push_back(getFrontBottomLeft());
        points.push_back(getBackTopRight());
        points.push_back(getBackTopLeft());
        points.push_back(getBackBottomRight());
        return points;
    }
};
