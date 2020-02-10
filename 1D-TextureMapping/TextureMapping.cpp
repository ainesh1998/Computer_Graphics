#include <ModelTriangle.h>
#include <CanvasTriangle.h>
#include <DrawingWindow.h>
// #include <CanvasPoint.h>
// #include <Colour.h>
#include <Utils.h>
#include <glm/glm.hpp>
#include <fstream>
#include <vector>

// using namespace std;
// using namespace glm;

#define WIDTH 800
#define HEIGHT 640

void draw();
void update();
void handleEvent(SDL_Event event);
std::vector<float> interpolate(float start, float end, int noOfValues);
std::vector<glm::vec3> interpolate3(glm::vec3 start, glm::vec3 end, int noOfValues);
void drawLine(CanvasPoint start,CanvasPoint end,Colour c);
void drawTriangle(CanvasTriangle triangle);
void drawFilledTriangle(CanvasTriangle triangle);

void greyscale();
void colourScale();

DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);

std::vector<float> interpolate(float start, float end, int noOfValues){
  std::vector<float> vals;
  float stepVal = (end - start)/(noOfValues - 1);
  vals.push_back(start);
  for(int i = 0; i < noOfValues - 1; i++){
    float temp = vals[i] + stepVal;
    vals.push_back(temp);
  }
  // for(int i = 0; i < noOfValues; i++){
  //   std::cout << vals[i] << ' ';
  // }
  return vals;
}

std::vector<glm::vec3> interpolate3(glm::vec3 start, glm::vec3 end, int noOfValues) {
    std::vector<glm::vec3> vals;
    float stepX = (end.x - start.x)/(noOfValues-1);
    float stepY = (end.y - start.y)/(noOfValues-1);
    float stepZ = (end.z - start.z)/(noOfValues-1);

    vals.push_back(start);
    for(int i = 0; i < noOfValues - 1; i++){
      float tempX = vals[i].x + stepX;
      float tempY = vals[i].y + stepY;
      float tempZ = vals[i].z + stepZ;
      glm::vec3 temp(tempX, tempY, tempZ);
      vals.push_back(temp);
    }
    return vals;
}

int main(int argc, char* argv[])
{
  SDL_Event event;

  interpolate(2.2,8.5,7);
  while(true)
  {

    // We MUST poll for events - otherwise the window will freeze !
    if(window.pollForInputEvents(&event)) handleEvent(event);
    update();
    // draw();
    // drawLine(CanvasPoint(150,150),CanvasPoint(300,160),Colour(255,255,0));
    CanvasTriangle triangle = CanvasTriangle(CanvasPoint(150,10),
                              CanvasPoint(140,50),
                            CanvasPoint(300,50),Colour(12,45,60));
  //  drawTriangle(triangle);
    //colourScale();
    // Need to render the frame at the end, or nothing actually gets shown on the screen !
    window.renderFrame();
  }
}

void drawTriangle(CanvasTriangle triangle){
  Colour c = triangle.colour;
  drawLine(triangle.vertices[0],triangle.vertices[1],c);
  drawLine(triangle.vertices[1],triangle.vertices[2],c);
  drawLine(triangle.vertices[2],triangle.vertices[0],c);
}

void drawFilledTriangle(CanvasTriangle triangle){
    //sort vertices in order y position
    if(triangle.vertices[1].y < triangle.vertices[0].y){
        std::swap(triangle.vertices[0],triangle.vertices[1]);
    }

    if(triangle.vertices[2].y < triangle.vertices[1].y){
        std::swap(triangle.vertices[1],triangle.vertices[2]);
        if(triangle.vertices[1].y < triangle.vertices[0].y){
            std::swap(triangle.vertices[1],triangle.vertices[0]);
        }
    }
    // std::cout << triangle << '\n';
    // std::vector<float> vals = interpolate(2.2,8.5,3);
    // for(int i = 0; i < 3; i++){
    //   std::cout << vals[i] << ' ';
    // }
    // std::cout << '\n';
    CanvasPoint v1 = triangle.vertices[0];
    CanvasPoint v2 = triangle.vertices[1];
    CanvasPoint v3 = triangle.vertices[2];
    float slope = (v2.y - v1.y)/(v3.y - v1.y);
    int newX = v1.x + slope * (v3.x - v1.x);
    CanvasPoint v4 = CanvasPoint(newX,v2.y);
    // std::cout << triangle << '\n';
    // std::cout << v4 << '\n';
    Colour c = triangle.colour;
    uint32_t colour = (255<<24) + (int(c.red)<<16) + (int(c.green)<<8) + int(c.blue);


    std::vector<float> leftX = interpolate(v1.x, v4.x, v4.y-v1.y);
    std::vector<float> rightX = interpolate(v1.x, v2.x, v4.y-v1.y);
    for(int i = v1.y; i < v4.y; i++){
        std::vector<float> temp = interpolate(leftX[i - v1.y],rightX[i-v1.y],rightX[i-v1.y]- leftX[i - v1.y]+1);
        for(int j = 0; j < temp.size(); j++){
            window.setPixelColour(temp[j],i,colour);
        }
    }
    leftX = interpolate(v4.x, v3.x, v3.y-v4.y);
    rightX = interpolate(v2.x, v3.x, v3.y-v4.y);
    for(int i = v4.y; i < v3.y; i++){
        std::vector<float> temp = interpolate(leftX[i - v4.y],rightX[i-v4.y],rightX[i-v4.y]- leftX[i - v4.y]+1);
        for(int j = 0; j < temp.size(); j++){
            window.setPixelColour(temp[j],i,colour);
        }
    }
    drawTriangle(triangle);


    // for (int i = v1.y; i < v2.y; i++) {
    // }
}

void drawLine(CanvasPoint start,CanvasPoint end,Colour c){
  float xDiff = end.x - start.x;
  float yDiff = end.y - start.y;
  float numberOfSteps = std::max(abs(xDiff), abs(yDiff));
  float xStepSize = xDiff/numberOfSteps;
  float yStepSize = yDiff/numberOfSteps;
  uint32_t colour = (255<<24) + (int(c.red)<<16) + (int(c.green)<<8) + int(c.blue);
  for (float i=0.0; i<numberOfSteps; i++) {
    float x = start.x + (xStepSize*i);
    float y = start.y + (yStepSize*i);
    window.setPixelColour(round(x), round(y), colour);
  }

}

void draw()
{
  window.clearPixels();
  for(int y=0; y<window.height ;y++) {
    for(int x=0; x<window.width ;x++) {
      float red = rand() % 255;
      float green = 0.0;
      float blue = 0.0;
      uint32_t colour = (255<<24) + (int(red)<<16) + (int(green)<<8) + int(blue);
      window.setPixelColour(x, y, colour);
    }
  }
}

void greyscale() {
    window.clearPixels();
    std::vector<float> pixelRow = interpolate(0, 255, window.width);

    for(int y=0; y<window.height ;y++) {

      for(int x=0; x<window.width ;x++) {
        float pixelValue = 255 - pixelRow[x];
        uint32_t colour = (255<<24) + (int(pixelValue)<<16) + (int(pixelValue)<<8) + int(pixelValue);
        window.setPixelColour(x, y, colour);
      }
    }
}

void colourScale() {
    window.clearPixels();
    glm::vec3 red(255,0,0);
    glm::vec3 yellow(255,255,0);
    std::vector<glm::vec3> redToYellow = interpolate3(red, yellow, window.height);

    glm::vec3 blue(0,0,255);
    glm::vec3 green(0,255,0);
    std::vector<glm::vec3> blueToGreen = interpolate3(blue, green, window.height);

    for(int y=0; y<window.height ;y++) {
        glm::vec3 start = redToYellow[y];
        glm::vec3 end = blueToGreen[y];
        std::vector<glm::vec3> pixelRow = interpolate3(start,end,window.width);

        for(int x=0; x<window.width ;x++) {
            glm::vec3 pixel = pixelRow[x];
            uint32_t colour = (255<<24) + (int(pixel.x)<<16) + (int(pixel.y)<<8) + int(pixel.z);
            window.setPixelColour(x, y, colour);
        }
    }
}

void update()
{
  // Function for performing animation (shifting artifacts or moving the camera)
}

void handleEvent(SDL_Event event)
{
  if(event.type == SDL_KEYDOWN) {
    if(event.key.keysym.sym == SDLK_LEFT) std::cout << "LEFT" << std::endl;
    else if(event.key.keysym.sym == SDLK_RIGHT) std::cout << "RIGHT" << std::endl;
    else if(event.key.keysym.sym == SDLK_UP) std::cout << "UP" << std::endl;
    else if(event.key.keysym.sym == SDLK_DOWN) std::cout << "DOWN" << std::endl;
    else if(event.key.keysym.sym == SDLK_u){

      CanvasTriangle triangle = CanvasTriangle(CanvasPoint(rand()%WIDTH,rand()%HEIGHT),
                                CanvasPoint(rand()%WIDTH,rand()%HEIGHT),
                              CanvasPoint(rand()%WIDTH,rand()%HEIGHT),Colour(rand()%255,rand()%255,rand()%255));
      drawTriangle(triangle);

    }
    else if(event.key.keysym.sym == SDLK_f){

      CanvasTriangle triangle = CanvasTriangle(CanvasPoint(rand()%WIDTH,rand()%HEIGHT),
                                CanvasPoint(rand()%WIDTH,rand()%HEIGHT),
                              CanvasPoint(rand()%WIDTH,rand()%HEIGHT),Colour(rand()%255,rand()%255,rand()%255));
      drawFilledTriangle(triangle);

    }

  }
  else if(event.type == SDL_MOUSEBUTTONDOWN) std::cout << "MOUSE CLICKED" << std::endl;
}
