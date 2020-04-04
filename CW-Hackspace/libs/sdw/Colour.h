#include <iostream>

class Colour
{
  public:
    std::string name;
    int red;
    int green;
    int blue;

    Colour()
    {
    }

    Colour(int r, int g, int b)
    {
      name = "";
      red = r;
      green = g;
      blue = b;
    }

    Colour(std::string n, int r, int g, int b)
    {
      name = n;
      red = r;
      green = g;
      blue = b;
    }

    Colour(glm::vec3 c) {
        name = "";
        red = c.x;
        green = c.y;
        blue = c.z;
    }

    uint32_t packed_colour(){
        uint32_t colour = (255<<24) + (int(red)<<16) + (int(green)<<8) + int(blue);
        return colour;
    }

    glm::vec3 toVec3() {
        return glm::vec3(red, green, blue);
    }
};


std::ostream& operator<<(std::ostream& os, const Colour& colour)
{
    os << colour.name << " [" << colour.red << ", " << colour.green << ", " << colour.blue << "]" << std::endl;
    return os;
}
