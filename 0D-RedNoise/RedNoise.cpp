#include <ModelTriangle.h>
#include <CanvasTriangle.h>
#include <DrawingWindow.h>
#include <Utils.h>
#include <glm/glm.hpp>
#include <fstream>
#include <vector>

using namespace std;
using namespace glm;

#define WIDTH 320
#define HEIGHT 240

void draw();
void update();
void handleEvent(SDL_Event event);
vector<float> interpolate(float start, float end, int noOfValues);
vector<vec3> interpolate3(vec3 start, vec3 end, int noOfValues);
void greyscale();
void colourScale();

DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);

vector<float> interpolate(float start, float end, int noOfValues){
  vector<float> vals;
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

vector<vec3> interpolate3(vec3 start, vec3 end, int noOfValues) {
    vector<vec3> vals;
    float stepX = (end.x - start.x)/(noOfValues-1);
    float stepY = (end.y - start.y)/(noOfValues-1);
    float stepZ = (end.z - start.z)/(noOfValues-1);

    vals.push_back(start);
    for(int i = 0; i < noOfValues - 1; i++){
      float tempX = vals[i].x + stepX;
      float tempY = vals[i].y + stepY;
      float tempZ = vals[i].z + stepZ;
      vec3 temp(tempX, tempY, tempZ);
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
    colourScale();
    // Need to render the frame at the end, or nothing actually gets shown on the screen !
    window.renderFrame();
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
    vector<float> pixelRow = interpolate(0, 255, window.width);

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
    vec3 red(255,0,0);
    vec3 yellow(255,255,0);
    vector<vec3> redToYellow = interpolate3(red, yellow, window.height);

    vec3 blue(0,0,255);
    vec3 green(0,255,0);
    vector<vec3> blueToGreen = interpolate3(blue, green, window.height);

    for(int y=0; y<window.height ;y++) {
        vec3 start = redToYellow[y];
        vec3 end = blueToGreen[y];
        vector<vec3> pixelRow = interpolate3(start,end,window.width);

        for(int x=0; x<window.width ;x++) {
            vec3 pixel = pixelRow[x];
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
    if(event.key.keysym.sym == SDLK_LEFT) cout << "LEFT" << endl;
    else if(event.key.keysym.sym == SDLK_RIGHT) cout << "RIGHT" << endl;
    else if(event.key.keysym.sym == SDLK_UP) cout << "UP" << endl;
    else if(event.key.keysym.sym == SDLK_DOWN) cout << "DOWN" << endl;
  }
  else if(event.type == SDL_MOUSEBUTTONDOWN) cout << "MOUSE CLICKED" << endl;
}
