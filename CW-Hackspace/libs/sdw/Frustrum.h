#include <iostream>
#include "BoundingBox.h"

class Frustrum
{
  public:
    vec3 frontTopLeft;
    float frontWidth;
    float frontHeight;
    vec3 backTopLeft;
    float backWidth;
    float backHeight;

    Frustrum(vec3 v0, float fw, float fh, vec3 v1, float bw, float bh)
    {
        frontTopLeft = v0;
        frontWidth = fw;
        frontHeight = fh;
        backTopLeft = v1;
        backWidth = bw;
        backHeight = bh;
    }

    bool contains(BoundingBox b) {
        return true;
    }
};
