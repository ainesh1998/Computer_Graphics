#include <ModelTriangle.h>
#include <CanvasTriangle.h>
#include <DrawingWindow.h>
#include <Utils.h>
#include <glm/glm.hpp>
#include <fstream>
#include <vector>
#include <map>



#define WIDTH 800
#define HEIGHT 640

void update();
void handleEvent(SDL_Event event);
std::vector<float> interpolate(float start, float end, int noOfValues);
std::vector<glm::vec3> interpolate3(glm::vec3 start, glm::vec3 end, int noOfValues);
void drawLine(CanvasPoint start,CanvasPoint end,Colour c);
void drawTriangle(CanvasTriangle triangle);
void drawFilledTriangle(CanvasTriangle triangle);
void drawTexturedTriangle(CanvasTriangle triangle,CanvasTriangle texture,std::vector<Colour> payload,int width,int height);
void displayPicture(std::vector<Colour> payload,int width,int height);
std::vector<Colour> readPPM(std::string filename,int* width, int* height);
std::map<std::string,Colour> readMTL(std::string filename);
std::vector<ModelTriangle> readOBJ(std::string filename,float scale);
void wireframe(std::string filename, float stepBack, float focalLength);


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
    // drawLine(u4,u2,Colour(0,255,0));
    // drawLine(v4,v2,Colour(0,255,0));

    //Compute  flat bottom triangle
    float dxdyl = (v2.x - v1.x)/(v2.y -v1.y);
    float dux_dyl = (u2.x - u1.x)/(v2.y -v1.y);
    float duy_dyl = (u2.y- u1.y)/(v2.y -v1.y);

    float dxdyr = (v4.x - v1.x)/(v2.y -v1.y);
    float dux_dyr = (u4.x - u1.x)/(v2.y -v1.y);
    float duy_dyr = (u4.y- u1.y)/(v2.y -v1.y);

    float xl = v1.x;
    float u_xl = u1.x;
    float u_yl = u1.y;

    float xr = v1.x;
    float u_xr = u1.x;
    float u_yr = u1.y;

    for (int y = v1.y; y <= v2.y; y++){
        float ui = u_xl;
        float vi = u_yl;
        float dx = xr - xl;
        float du = (u_xr - u_xl)/dx;
        float dv = (u_yr - u_yl)/dx;
        for(int x = xl; x <= xr;x++){
            Colour c = payload[(int) ui + (int) vi * width];
            window.setPixelColour(x,y,c.packed_colour());
            ui += du;
            vi += dv;
        }

        xl += dxdyl;
        u_xl += dux_dyl;
        u_yl += duy_dyl;
        xr += dxdyr;
        u_xr += dux_dyr;
        u_yr += duy_dyr;
  }

  //Compute Flat Top triangle
  dxdyl = (v3.x - v2.x)/(v3.y -v2.y);
  dux_dyl = (u3.x - u2.x)/(v3.y -v2.y);
  duy_dyl = (u3.y- u2.y)/(v3.y -v2.y);

  dxdyr = (v3.x - v4.x)/(v3.y -v2.y);
  dux_dyr = (u3.x - u4.x)/(v3.y -v2.y);
  duy_dyr = (u3.y- u4.y)/(v3.y -v2.y);

  xl = v3.x;
  u_xl = u3.x;
  u_yl = u3.y;

  xr = v3.x;
  u_xr = u3.x;
  u_yr = u3.y;

  for (int y = v3.y; y > v2.y; y--){
      float ui = u_xl;
      float vi = u_yl;
      float dx = xr - xl;
      float du = (u_xr - u_xl)/dx;
      float dv = (u_yr - u_yl)/dx;

      for(int x = xl; x <= xr;x++){
          Colour c = payload[(int) ui + (int) vi * width];
         window.setPixelColour(x,y,c.packed_colour());
          ui += du;
          vi += dv;
      }
      xl -= dxdyl;
      u_xl -= dux_dyl;
      u_yl -= duy_dyl;
      xr -= dxdyr;
      u_xr -= dux_dyr;
      u_yr -= duy_dyr;
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
    drawLine(v2,v4,Colour(255,255,255));

    //fill top triangle
    float invslope1 = (v2.x - v1.x) / (v2.y - v1.y);
    float invslope2 = (v4.x - v1.x) / (v4.y - v1.y);
    float curx1 = v1.x;
    float curx2 = v1.x;

    for (int scanlineY = v1.y; scanlineY <= v2.y; scanlineY++) {
        drawLine(CanvasPoint(curx1, scanlineY), CanvasPoint(curx2, scanlineY),c);
        curx1 += invslope1;
        curx2 += invslope2;
    }
    
   //  //fill bottom triangle
    float invslope3 = (v3.x - v2.x) / (v3.y - v2.y);
    float invslope4 = (v3.x - v4.x) / (v3.y - v4.y);

    float curx3 = v3.x;
    float curx4 = v3.x;

    for (int scanlineY = v3.y; scanlineY > v2.y; scanlineY--)
   {
     drawLine(CanvasPoint(curx3, scanlineY), CanvasPoint(curx4, scanlineY),c);
     curx3 -= invslope3;
     curx4 -= invslope4;
   }

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

void displayPicture(std::vector<Colour> payload,int width,int height){
    for(int i = 0; i < width; i++){
        for(int j = 0; j < height; j++){
            uint32_t colour = payload[i + j * width].packed_colour();
            window.setPixelColour(i, j, colour);
        }
    }
}


// 3D


std::vector<Colour> readPPM(std::string filename,int* width, int* height){
    std::ifstream stream;
    stream.open(filename.c_str(),std::ifstream::in);
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
std::map<std::string,Colour> readMTL(std::string filename){
    std::map<std::string,Colour> colourMap;
    std::ifstream stream;
    stream.open(filename,std::ifstream::in);

    char newmtl[256];

    while(stream.getline(newmtl, 256, ' ') && strcmp(newmtl, "newmtl") == 0) {
        char colourName[256];
        stream.getline(colourName, 256);

        char mtlProperty[256];
        char rc[256];
        char gc[256];
        char bc[256];

        stream.getline(mtlProperty, 256, ' ');
        stream.getline(rc, 256, ' ');
        stream.getline(gc, 256, ' ');
        stream.getline(bc, 256);

        int r = std::stof(rc) * 255;
        int g = std::stof(gc) * 255;
        int b = std::stof(bc) * 255;

        Colour c = Colour(colourName, r, g, b);
        colourMap[colourName] = c;
        // colours.push_back(c);

        char newLine[256];
        stream.getline(newLine, 256);
    }
    stream.clear();
    stream.close();
    return colourMap;
}
std::vector<ModelTriangle> readOBJ(std::string filename,float scale) {

    std::ifstream stream;
    stream.open(filename + "/" + filename + ".obj",std::ifstream::in);
    char mtlFile[256];
    stream.getline(mtlFile,256,' '); //skip the mtllib
    stream.getline(mtlFile,256);

    std::map<std::string,Colour> colourMap = readMTL(filename + "/" + (std::string)mtlFile);

    std::vector<glm::vec3> vertices;
    std::vector<ModelTriangle> modelTriangles;
    char line[256];
    Colour colour;
    while(stream.getline(line,256)){

        std::string* contents = split(line,' ');
        if(line[0] == 'u'){
            colour = colourMap[contents[1]];
        }
        else if(line[0] == 'v'){
                float x = std::stof(contents[1]) * scale;
                float y = std::stof(contents[2]) * scale;
                float z = std::stof(contents[3]) * scale;
                glm::vec3 v(x,y,z);
                vertices.push_back(v);
        }
        else if(line[0] == 'f'){
            std::string* indexes1 = split(contents[1],'/');
            std::string* indexes2 = split(contents[2],'/');
            std::string* indexes3 = split(contents[3],'/');

            int index1 = std::stoi(indexes1[0]);
            int index2 = std::stoi(indexes2[0]);
            int index3 = std::stoi(indexes3[0]);

            ModelTriangle m = ModelTriangle(vertices[index1 -1],
            vertices[index2 - 1], vertices[index3 -1],colour);
            modelTriangles.push_back(m);
        }
    }

    stream.clear();
    stream.close();
    return modelTriangles;
}

void wireframe(std::string filename, float stepBack, float focalLength) {
    // stepBack = dv, focalLength = di

    std::vector<ModelTriangle> triangles = readOBJ(filename, 200);

    for (int i = 0; i < triangles.size(); i++) {
        std::cout << triangles[i] << '\n';
        glm::vec3 cameraPos = glm::vec3(0, 0, -stepBack);
        std::vector<CanvasPoint> points;
        for (int j = 0; j < 3; j++) {

            glm::vec3 wrtCamera = triangles[i].vertices[j] + cameraPos;
            float ratio = focalLength/(-wrtCamera.z);

            float x = wrtCamera.x * ratio + WIDTH/2;
            float y = (1-wrtCamera.y) * ratio + HEIGHT/2;

            CanvasPoint point = CanvasPoint(x, y);
            points.push_back(point);
        }
        CanvasTriangle triangle = CanvasTriangle(points[0], points[1], points[2], triangles[i].colour);
        drawFilledTriangle(triangle);
    }

}


// EVENT HANDLING


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

    // start of 3D lab
    else if (event.key.keysym.sym == SDLK_o) {
        int temp = 500;
        wireframe("cornell-box", temp, temp/2);
    }

    else if(event.key.keysym.sym == SDLK_c){
        window.clearPixels();
    }
  }
  else if(event.type == SDL_MOUSEBUTTONDOWN) std::cout << "MOUSE CLICKED" << std::endl;
}

void update()
{
  // Function for performing animation (shifting artifacts or moving the camera)
}
