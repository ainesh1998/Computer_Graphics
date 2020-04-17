#include <iostream>

class BoundingBox
{
  public:
    CanvasPoint topLeft;
    int width;
    int height

    BoundingBox()
    {
    }

    BoundingBox(CanvasPoint start, int w, int h)
    {
      topLeft = start;
      width = w;
      height = h;
    }
};

// std::ostream& operator<<(std::ostream& os, const BumpPoint& point)
// {
//     os << "(" << point.x << ", " << point.y << ")" << std::endl;
//     return os;
// }
