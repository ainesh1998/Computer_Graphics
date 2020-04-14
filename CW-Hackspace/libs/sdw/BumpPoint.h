#include <iostream>

class BumpPoint
{
  public:
    float x;
    float y;

    BumpPoint()
    {
    }

    BumpPoint(float xPos, float yPos)
    {
      x = xPos;
      y = yPos;
    }

    void print()
    {
    }
};

std::ostream& operator<<(std::ostream& os, const BumpPoint& point)
{
    os << "(" << point.x << ", " << point.y << ")" << std::endl;
    return os;
}
