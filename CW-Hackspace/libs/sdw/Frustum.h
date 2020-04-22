#include <iostream>
// #include <glm/glm.hpp>

class Frustum
{
  public:
    // glm::vec3 frontTopLeft;
    // float frontWidth;
    // float frontHeight;
    // glm::vec3 backTopLeft;
    // float backWidth;
    // float backHeight;
    glm::vec3 frontTopLeft;
    glm::vec3 frontTopRight;
    glm::vec3 frontBottomLeft;
    glm::vec3 frontBottomRight;
    glm::vec3 backTopLeft;
    glm::vec3 backTopRight;
    glm::vec3 backBottomLeft;
    glm::vec3 backBottomRight;


    // Frustum(glm::vec3 v0, float fw, float fh, glm::vec3 v1, float bw, float bh)
    // {
    //     frontTopLeft = v0;
    //     frontWidth = fw;
    //     frontHeight = fh;
    //     backTopLeft = v1;
    //     backWidth = bw;
    //     backHeight = bh;
    // }

    Frustum(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 v3, glm::vec3 v4, glm::vec3 v5,
             glm::vec3 v6, glm::vec3 v7)
    {
        frontTopLeft = v0;
        frontTopRight = v1;
        frontBottomLeft = v2;
        frontBottomRight = v3;
        backTopLeft = v4;
        backTopRight = v5;
        backBottomLeft = v6;
        backBottomRight = v7;
    }

    bool contains(BoundingBox b) {
        return contains(b.startVertex) || contains(b.getBackTopLeft()) ||
               contains(b.getBackBottomRight()) || contains(b.getBackTopRight()) ||
               contains(b.getFrontBottomLeft()) || contains(b.getFrontBottomRight()) ||
               contains(b.getFrontTopLeft()) || contains(b.getFrontTopRight());
    }

    bool contains(glm::vec3 point) {
        // bool yBound = point.y >= backBottomLeft.y && point.y >= backBottomRight.y &&
        //               point.y >= frontBottomLeft.y && point.y >= frontBottomRight.y &&
        //               point.y <= backTopLeft.y && point.y <= backTopRight.y &&
        //               point.y <= frontTopLeft.y && point.y <= frontTopRight.y;
        // bool xBound = point.x >= backBottomLeft.x && point.x >= backTopLeft.x &&
        //               point.x >= frontBottomLeft.x && point.x >= frontTopLeft.x &&
        //               point.x <= backBottomLeft.x && point.x <= backTopRight.x &&
        //               point.x <= frontBottomRight.x && point.x <= frontTopRight.x;
        // bool zBound = point.z >= backBottomLeft.z && point.z >= backTopLeft.z &&
        //               point.z >= backBottomRight.z && point.z >= backTopRight.z &&
        //               point.z <= frontBottomLeft.z && point.z <= frontTopRight.z &&
        //               point.z <= frontBottomRight.z && point.z <= frontTopLeft.z;
        //

        glm::vec3 u = glm::cross((frontBottomLeft - frontBottomRight), (frontBottomLeft - frontTopLeft));
        glm::vec3 v = glm::cross((frontBottomLeft - backBottomLeft), (frontBottomLeft - frontTopLeft));
        glm::vec3 w = glm::cross((frontBottomLeft - backBottomLeft), (frontBottomLeft - frontBottomRight));

        bool xBound = between(glm::dot(u, point), glm::dot(u, frontBottomLeft), glm::dot(u, backBottomLeft));
        bool yBound = between(glm::dot(v, point), glm::dot(v, frontBottomLeft), glm::dot(v, frontBottomRight));
        bool zBound = between(glm::dot(w, point), glm::dot(w, frontBottomLeft), glm::dot(w, frontTopLeft));

        std::cout << xBound << " " << yBound << " " << zBound << '\n';
        return xBound && yBound && zBound;
    }

    bool between(float x, float b1, float b2) {
        return (x >= b1 && x <= b2) || (x >= b2 && x <= b1);
    }
};

// std::ostream& operator<<(std::ostream& os, const Frustum& f)
// {
//     os << "(" << triangle.vertices[0].x << ", " << triangle.vertices[0].y << ", " << triangle.vertices[0].z << ")" << std::endl;
//     os << "(" << triangle.vertices[1].x << ", " << triangle.vertices[1].y << ", " << triangle.vertices[1].z << ")" << std::endl;
//     os << "(" << triangle.vertices[2].x << ", " << triangle.vertices[2].y << ", " << triangle.vertices[2].z << ")" << std::endl;
//     os << std::endl;
//     return os;
// }
