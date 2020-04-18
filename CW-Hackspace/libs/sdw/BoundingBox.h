#include <iostream>

class BoundingBox
{
  public:
    glm::vec3 startVertex;
    int width;
    int height;
    int depth;

    BoundingBox(glm::vec3 start, int w, int h, int d)
    {
      startVertex = start;
      width = w;
      height = h;
      depth = d;
    }
};
