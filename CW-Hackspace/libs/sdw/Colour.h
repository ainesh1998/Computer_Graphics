#include <iostream>

class Colour
{
  public:
    std::string name;
    int red;
    int green;
    int blue;
    int alpha;

    Colour()
    {
    }

    Colour(uint32_t c) {
        name = "";
        blue = c & 0xFF;
        green = (c & 0xFF00) >> 8;
        red = (c & 0xFF0000) >> 16;
        alpha = c >> 24;
    }

    Colour(int r, int g, int b)
    {
      name = "";
      red = r;
      green = g;
      blue = b;
      alpha = 255;
    }

    Colour(std::string n, int r, int g, int b)
    {
      name = n;
      red = r;
      green = g;
      blue = b;
      alpha = 255;
    }

    Colour(int r, int g, int b, int a)
    {
      name = "";
      red = r;
      green = g;
      blue = b;
      alpha = a;
    }

    Colour(glm::vec3 c) {
        name = "";
        red = c.x;
        green = c.y;
        blue = c.z;
        alpha = 255;
    }

    uint32_t packed_colour(){
        uint32_t colour = (alpha<<24) + (int(red)<<16) + (int(green)<<8) + int(blue);
        return colour;
    }

    glm::vec3 toVec3() {
        return glm::vec3(red, green, blue);
    }

    bool equals(Colour c) {
        return c.red == red && c.green == green && c.blue == blue && c.name == name;
    }
};


std::ostream& operator<<(std::ostream& os, const Colour& colour)
{
    os << colour.name << " [" << colour.red << ", " << colour.green << ", " << colour.blue << "]" << std::endl;
    return os;
}
