// #include "TexturePoint.h"
#include <iostream>

class CanvasPoint
{
  public:
    float x;
    float y;
    double depth;
    float brightness;
    TexturePoint texturePoint;
    bool isTexture;

    CanvasPoint()
    {
        texturePoint = TexturePoint(-1,-1);
        isTexture = false;
    }

    CanvasPoint(float xPos, float yPos)
    {
      x = xPos;
      y = yPos;
      depth = 0.0;
      brightness = 1.0;
      texturePoint = TexturePoint(-1,-1);
      isTexture = false;
    }

    CanvasPoint(float xPos, float yPos, float pointDepth)
    {
      x = xPos;
      y = yPos;
      depth = pointDepth;
      brightness = 1.0;
      texturePoint = TexturePoint(-1,-1);
      isTexture = false;
    }

    CanvasPoint(float xPos, float yPos, float pointDepth, float pointBrightness)
    {
      x = xPos;
      y = yPos;
      depth = pointDepth;
      brightness = pointBrightness;
      texturePoint = TexturePoint(-1,-1);
      isTexture = false;
    }

    CanvasPoint(float xPos, float yPos, float pointDepth, TexturePoint t)
    {
      x = xPos;
      y = yPos;
      depth = pointDepth;
      brightness = 1.0;
      texturePoint = t;
      isTexture = (t.x != -1 && t.y != -1);
    }

};

std::ostream& operator<<(std::ostream& os, const CanvasPoint& point)
{
    os << "(" << point.x << ", " << point.y << ", " << point.depth << ") " << point.brightness << std::endl;
    return os;
}
