#include <ModelTriangle.h>
#include <CanvasTriangle.h>
#include <DrawingWindow.h>
#include <Utils.h>
#include <glm/glm.hpp>
#include <fstream>
#include <vector>


#define WIDTH 800
#define HEIGHT 640

using glm::vec3;

void draw();
void update();
void handleEvent(SDL_Event event);
std::vector<float> interpolate(float start, float end, int noOfValues);
std::vector<glm::vec3> interpolate3(glm::vec3 start, glm::vec3 end, int noOfValues);
void drawLine(CanvasPoint start,CanvasPoint end,Colour c);
void drawTriangle(CanvasTriangle triangle);
void drawFilledTriangle(CanvasTriangle triangle);
void drawTexturedTriangle(CanvasTriangle triangle,CanvasTriangle texture,std::vector<Colour> payload,int width,int height);
void displayPicture(std::vector<Colour> payload,int width,int height);
std::vector<Colour> readPPM(char* filename,int* width, int* height);

void greyscale();
void colourScale();

DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);

int main(int argc, char* argv[])
{

  SDL_Event event;

  while(true)
  {

    // We MUST poll for events - otherwise the window will freeze !
    if(window.pollForInputEvents(&event)) handleEvent(event);
    update();
    // Need to render the frame at the end, or nothing actually gets shown on the screen !
    window.renderFrame();
  }
}

std::vector<float> interpolate(float start, float end, int noOfValues){
  std::vector<float> vals;
  float stepVal = (end - start)/(noOfValues - 1);
  vals.push_back(start);
  for(int i = 0; i < noOfValues - 1; i++){
    float temp = vals[i] + stepVal;
    vals.push_back(temp);
  }
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


void drawTriangle(CanvasTriangle triangle){
  Colour c = triangle.colour;
  drawLine(triangle.vertices[0],triangle.vertices[1],c);
  drawLine(triangle.vertices[1],triangle.vertices[2],c);
  drawLine(triangle.vertices[2],triangle.vertices[0],c);
}
void drawTexturedTriangle(CanvasTriangle triangle,CanvasTriangle texture,std::vector<Colour> payload,int width,int height){
    //sort vertices in order (y position)

    if(triangle.vertices[1].y < triangle.vertices[0].y){
        std::swap(triangle.vertices[0],triangle.vertices[1]);
        std::swap(texture.vertices[0], texture.vertices[1]);
    }

    if(triangle.vertices[2].y < triangle.vertices[1].y){
        std::swap(triangle.vertices[1],triangle.vertices[2]);
        std::swap(texture.vertices[1],texture.vertices[2]);
        if(triangle.vertices[1].y < triangle.vertices[0].y){
            std::swap(triangle.vertices[1],triangle.vertices[0]);
            std::swap(texture.vertices[1],texture.vertices[0]);
        }
    }

    CanvasPoint v1 = triangle.vertices[0];
    CanvasPoint v2 = triangle.vertices[1];
    CanvasPoint v3 = triangle.vertices[2];
    float slope = (v2.y - v1.y)/(v3.y - v1.y);
    int newX = v1.x + slope * (v3.x - v1.x);
    CanvasPoint v4 = CanvasPoint(newX,v2.y);

    CanvasPoint u1 = texture.vertices[0];
    CanvasPoint u2 = texture.vertices[1];
    CanvasPoint u3 = texture.vertices[2];

    float k_x = (u3.x-u1.x)/(v3.x-v1.x);
    float k_y = (u3.y-u1.y)/(v3.y-v1.y);
    int u4_x = u1.x + k_x * (v4.x-v1.x);
    int u4_y = u1.y + k_y * (v4.y-v1.y);
    CanvasPoint u4 = CanvasPoint(u4_x,u4_y);

    //Compute  flat bottom triangle
    std::vector<vec3> triangleLeft = interpolate3(vec3(v1.x,v1.y,v1.depth), vec3(v2.x,v2.y,v2.depth), (v2.y-v1.y)+1);
    std::vector<vec3> textureLeft = interpolate3(vec3(u1.x,u1.y,u1.depth), vec3(u2.x,u2.y,u2.depth), (v2.y-v1.y)+1);

    std::vector<vec3> triangleRight = interpolate3(vec3(v1.x,v1.y,v1.depth), vec3(v4.x,v4.y,v4.depth), (v2.y-v1.y)+1);
    std::vector<vec3> textureRight = interpolate3(vec3(u1.x,u1.y,u1.depth), vec3(u4.x,u4.y,u4.depth), (v2.y-v1.y)+1);

    for (int i = 0; i < triangleLeft.size(); i++) {
        vec3 startTriangle = vec3((int) triangleLeft[i].x, triangleLeft[i].y, triangleLeft[i].z);
        vec3 endTriangle = vec3((int) triangleRight[i].x, triangleRight[i].y, triangleRight[i].z);
        std::vector<vec3> rakeTriangle = interpolate3(startTriangle, endTriangle, std::abs(endTriangle.x-startTriangle.x)+1);

        vec3 startTexture = vec3((int) textureLeft[i].x, (int) textureLeft[i].y, textureLeft[i].z);
        vec3 endTexture = vec3((int) textureRight[i].x, (int) textureRight[i].y, textureRight[i].z);
        std::vector<vec3> rakeTexture = interpolate3(startTexture, endTexture, std::abs(endTriangle.x-startTriangle.x)+1);

        int y = triangleLeft[i].y;

        for (int j = 0; j < rakeTriangle.size(); j++) {
            int x = rakeTriangle[j].x;
            int ui = rakeTexture[j].x;
            int vi = rakeTexture[j].y;

            Colour c = payload[(int) ui + (int) vi * width];
            window.setPixelColour(x,y,c.packed_colour());
        }
  }

  //Compute Flat Top triangle
  triangleLeft = interpolate3(vec3(v2.x,v2.y,v2.depth), vec3(v3.x,v3.y,v3.depth), (v3.y-v2.y)+1);
  textureLeft = interpolate3(vec3(u2.x,u2.y,u2.depth), vec3(u3.x,u3.y,u3.depth), (v3.y-v2.y)+1);

  triangleRight = interpolate3(vec3(v4.x,v4.y,v4.depth), vec3(v3.x,v3.y,v3.depth), (v3.y-v2.y)+1);
  textureRight = interpolate3(vec3(u4.x,u4.y,u4.depth), vec3(u3.x,u3.y,u3.depth), (v3.y-v2.y)+1);

  for (int i = 0; i < triangleLeft.size(); i++) {
      vec3 startTriangle = vec3((int) triangleLeft[i].x, triangleLeft[i].y, triangleLeft[i].z);
      vec3 endTriangle = vec3((int) triangleRight[i].x, triangleRight[i].y, triangleRight[i].z);
      std::vector<vec3> rakeTriangle = interpolate3(startTriangle, endTriangle, std::abs(endTriangle.x-startTriangle.x)+1);

      vec3 startTexture = vec3((int) textureLeft[i].x, (int) textureLeft[i].y, textureLeft[i].z);
      vec3 endTexture = vec3((int) textureRight[i].x, (int) textureRight[i].y, textureRight[i].z);
      std::vector<vec3> rakeTexture = interpolate3(startTexture, endTexture, std::abs(endTriangle.x-startTriangle.x)+1);

      int y = triangleLeft[i].y;

      for (int j = 0; j < rakeTriangle.size(); j++) {
          int x = rakeTriangle[j].x;
          int ui = rakeTexture[j].x;
          int vi = rakeTexture[j].y;

          Colour c = payload[(int) ui + (int) vi * width];
          window.setPixelColour(x,y,c.packed_colour());
      }
  }
}
void drawFilledTriangle(CanvasTriangle triangle){
    //sort vertices in order (y position)
    if(triangle.vertices[1].y < triangle.vertices[0].y){
        std::swap(triangle.vertices[0],triangle.vertices[1]);
    }

    if(triangle.vertices[2].y < triangle.vertices[1].y){
        std::swap(triangle.vertices[1],triangle.vertices[2]);
        if(triangle.vertices[1].y < triangle.vertices[0].y){
            std::swap(triangle.vertices[1],triangle.vertices[0]);
        }
    }

    CanvasPoint v1 = triangle.vertices[0];
    CanvasPoint v2 = triangle.vertices[1];
    CanvasPoint v3 = triangle.vertices[2];
    float slope = (v2.y - v1.y)/(v3.y - v1.y);
    int newX = v1.x + slope * (v3.x - v1.x);
    CanvasPoint v4 = CanvasPoint(newX,v2.y);

    Colour c = triangle.colour;

    //fill top triangle
    std::vector<vec3> leftSide = interpolate3(vec3(v1.x,v1.y,v1.depth), vec3(v2.x,v2.y,v2.depth), v2.y-v1.y+1);
    std::vector<vec3> rightSide = interpolate3(vec3(v1.x,v1.y,v1.depth), vec3(v4.x,v4.y,v4.depth), v2.y-v1.y+1);

    for (int i = 0; i < leftSide.size(); i++) {
        drawLine(CanvasPoint((int) leftSide[i].x, leftSide[i].y), CanvasPoint((int) rightSide[i].x, rightSide[i].y),c);
    }

    //fill bottom triangle
    leftSide = interpolate3(vec3(v3.x,v3.y,v3.depth), vec3(v2.x,v2.y,v2.depth), std::abs(v2.y-v3.y)+1);
    rightSide = interpolate3(vec3(v3.x,v3.y,v3.depth), vec3(v4.x,v4.y,v4.depth), std::abs(v4.y-v3.y)+1);

    for (int i = 0; i < leftSide.size(); i++) {
        drawLine(CanvasPoint((int) leftSide[i].x, leftSide[i].y), CanvasPoint((int) rightSide[i].x, rightSide[i].y),c);
    }
}

void drawLine(CanvasPoint start,CanvasPoint end,Colour c){
  float xDiff = end.x - start.x;
  float yDiff = end.y - start.y;
  float zDiff = end.depth - start.depth;
  float temp = std::max(abs(xDiff), abs(yDiff));
  float numberOfSteps = std::max(temp, std::abs(zDiff));

  std::vector<vec3> line = interpolate3(vec3(start.x,start.y,start.depth), vec3(end.x,end.y,end.depth), numberOfSteps+1);
  uint32_t colour = (255<<24) + (int(c.red)<<16) + (int(c.green)<<8) + int(c.blue);

  for (int i = 0; i < line.size(); i++) {
      window.setPixelColour(line[i].x, line[i].y, colour);
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
    else if(event.key.keysym.sym == SDLK_d){
        int width;
        int height;
        std::vector<Colour> payload = readPPM("texture.ppm",&width,&height);

        CanvasTriangle texture = CanvasTriangle(CanvasPoint(195,5),
                                  CanvasPoint(395,380),
                                CanvasPoint(65,330),Colour(0,0,255));
        displayPicture(payload,width,height);
        drawTriangle(texture);

    }
    else if(event.key.keysym.sym == SDLK_g){
        int width;
        int height;
        std::vector<Colour> payload = readPPM("texture.ppm",&width,&height);

        CanvasTriangle triangle = CanvasTriangle(CanvasPoint(160,10),
                                  CanvasPoint(300,230),
                                CanvasPoint(10,150),Colour(255,255,255));

        CanvasTriangle texture = CanvasTriangle(CanvasPoint(195,5),
                                  CanvasPoint(395,380),
                                CanvasPoint(65,330),Colour(0,0,255));
        // displayPicture(v,width,height);
        // drawTriangle(texture);
        // drawTriangle(triangle);
        drawTexturedTriangle(triangle,texture,payload,width,height);
    }
    else if(event.key.keysym.sym == SDLK_c){
        window.clearPixels();
    }

  }
  else if(event.type == SDL_MOUSEBUTTONDOWN) std::cout << "MOUSE CLICKED" << std::endl;
}
void displayPicture(std::vector<Colour> payload,int width,int height){
    for(int i = 0; i < width; i++){
        for(int j = 0; j < height; j++){
            uint32_t colour = payload[i + j * width].packed_colour();
            window.setPixelColour(i, j, colour);
        }
    }
}
std::vector<Colour> readPPM(char* filename,int* width, int* height){
    std::ifstream stream;
    stream.open(filename,std::ifstream::in);
    char encoding[3];
    stream.getline(encoding,3);

    char comment[256];
    stream.getline(comment,256);
    char widthText[256];
    char heightText[256];

    stream.getline(widthText,256,' ');
    stream.getline(heightText,256);
    *width = std::stoi(widthText);
    *height = std::stoi(heightText);

    char maxValT[256];
    stream.getline(maxValT,256);
    int maxVal = std::stoi(maxValT);

    char r;
    char g;
    char b;
    std::vector<Colour> payload;
    while(stream.get(r) &&stream.get(g)&& stream.get(b)){
        payload.push_back(Colour(r,g,b));
    }
    stream.clear();
    stream.close();
    return payload;

}
