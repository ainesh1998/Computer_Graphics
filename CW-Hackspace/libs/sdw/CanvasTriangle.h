#include "CanvasPoint.h"
#include <iostream>

class CanvasTriangle
{
  public:
    CanvasPoint vertices[3];
    Colour colour;
    bool isTexture;

    CanvasTriangle()
    {
    }

    CanvasTriangle(CanvasPoint v0, CanvasPoint v1, CanvasPoint v2)
    {
      vertices[0] = v0;
      vertices[1] = v1;
      vertices[2] = v2;
      colour = Colour(255,255,255);
      isTexture = v0.isTexture && v1.isTexture && v2.isTexture;
    }

    CanvasTriangle(CanvasPoint v0, CanvasPoint v1, CanvasPoint v2, Colour c)
    {
      vertices[0] = v0;
      vertices[1] = v1;
      vertices[2] = v2;
      colour = c;
      isTexture = v0.isTexture && v1.isTexture && v2.isTexture;
    }

    CanvasTriangle getTextureTriangle() {
        CanvasPoint t0 = CanvasPoint(vertices[0].texturePoint.x, vertices[0].texturePoint.y);
        CanvasPoint t1 = CanvasPoint(vertices[1].texturePoint.x, vertices[1].texturePoint.y);
        CanvasPoint t2 = CanvasPoint(vertices[2].texturePoint.x, vertices[2].texturePoint.y);
        return CanvasTriangle(t0, t1, t2);
    }
};

std::ostream& operator<<(std::ostream& os, const CanvasTriangle& triangle)
{
    os << triangle.vertices[0]  << triangle.vertices[1]  << triangle.vertices[2] << std::endl;
    return os;
}
