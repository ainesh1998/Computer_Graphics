#include <ModelTriangle.h>
#include <CanvasTriangle.h>
#include <DrawingWindow.h>
#include <Utils.h>
#include <glm/glm.hpp>
#include <fstream>
#include <vector>
#include <map>
#include <glm/gtx/string_cast.hpp>



#define WIDTH 800
#define HEIGHT 640
#define FOCALLENGTH 250

void update(glm::vec3 translation, glm::vec3 rotationAngles);
bool handleEvent(SDL_Event event, glm::vec3* translation, glm::vec3* rotationAngles,bool* orbit);
std::vector<float> interpolate(float start, float end, int noOfValues);
std::vector<glm::vec3> interpolate3(glm::vec3 start, glm::vec3 end, int noOfValues);
void drawLine(CanvasPoint start,CanvasPoint end,Colour c);
void drawTriangle(CanvasTriangle triangle);
void drawFilledTriangle(CanvasTriangle triangle);
void drawFilledTriangle(CanvasTriangle triangle, double** depth_buffer,double near,double far);
void drawTexturedTriangle(CanvasTriangle triangle,CanvasTriangle texture,std::vector<Colour> payload,int width,int height);
void displayPicture(std::vector<Colour> payload,int width,int height);
std::vector<Colour> readPPM(std::string filename,int* width, int* height);
std::map<std::string,Colour> readMTL(std::string filename);
std::vector<ModelTriangle> readOBJ(std::string filename,float scale);
void order_triangle(CanvasTriangle *triangle);
void drawBox(std::vector<ModelTriangle> triangles, float focalLength);
double **malloc2dArray(int dimX, int dimY);
void lookAt(glm::vec3 point);
void orbit(std::vector<ModelTriangle> triangles, int* counter, int* flag,glm::vec3* translation);


DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
glm::vec3 cameraPos = glm::vec3(0, 0, 300);
glm::mat3 cameraOrientation = glm::mat3();
glm::vec3 lookAtPos = glm::vec3(-100,0,-100);
float infinity = std::numeric_limits<float>::infinity();
double depth_buffer[WIDTH][HEIGHT];

int main(int argc, char* argv[])
{
    for(int x = 0; x < WIDTH; x++){
        for(int y = 0; y < HEIGHT; y++){
            depth_buffer[x][y] = std::numeric_limits<float>::infinity();
        }
    }
    SDL_Event event;
    std::vector<ModelTriangle> triangles = readOBJ("cornell-box", 50);
    // lookAt(lookAtPos);
    drawBox(triangles, FOCALLENGTH);
    window.renderFrame();

    int counter = 1;
    int flag = 1;
    while(true)
    {
        glm::vec3 translation = glm::vec3(0,0,0);
        glm::vec3 rotationAngles = glm::vec3(0,0,0);
        bool isUpdate = false;
        bool orbitFlag= false;

        // We MUST poll for events - otherwise the window will freeze !
        if(window.pollForInputEvents(&event)) {
            isUpdate = handleEvent(event, &translation, &rotationAngles,&orbitFlag);
        }
        // std::cout << orbitFlag << '\n';
        if(orbitFlag){
            orbit(triangles,&counter,&flag,&translation);
        }
        if (isUpdate) {
            update(translation, rotationAngles);


            // RENAMED WIREFRAME TO DRAW
            // drawBox(triangles, FOCALLENGTH);

            // Need to render the frame at the end, or nothing actually gets shown on the screen !
            window.renderFrame();
        }

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

    order_triangle(&triangle);
    CanvasPoint v1 = triangle.vertices[0];
    CanvasPoint v2 = triangle.vertices[1];
    CanvasPoint v3 = triangle.vertices[2];
    float slope = (v2.y - v1.y)/(v3.y - v1.y);
    int newX = v1.x + slope * (v3.x - v1.x);
    CanvasPoint v4 = CanvasPoint(newX,v2.y);
    // order_triangle(&texture);
    CanvasPoint u1 = texture.vertices[0];
    CanvasPoint u2 = texture.vertices[1];
    CanvasPoint u3 = texture.vertices[2];

    float k_x = (u3.x-u1.x)/(v3.x-v1.x);
    float k_y = (u3.y-u1.y)/(v3.y-v1.y);
    int u4_x = u1.x + k_x * (v4.x-v1.x);
    int u4_y = u1.y + k_y * (v4.y-v1.y);
    CanvasPoint u4 = CanvasPoint(u4_x,u4_y);

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

//sort triangle vertices in ascending order accroding to y
void order_triangle(CanvasTriangle *triangle){
    if(triangle->vertices[1].y < triangle->vertices[0].y){
        std::swap(triangle->vertices[0],triangle->vertices[1]);
    }

    if(triangle->vertices[2].y < triangle->vertices[1].y){
        std::swap(triangle->vertices[1],triangle->vertices[2]);
        if(triangle->vertices[1].y < triangle->vertices[0].y){
            std::swap(triangle->vertices[1],triangle->vertices[0]);
        }
    }
}

double compute_depth(double depth,double near,double far){
    //saw equation online
    double z = (near + far)/(far - near) + 1/depth * ((-2 * far * near)/(far - near));
    // z = 1/depth;
    // double z = (1/depth -1/near)/(1/far-1/near);
    return z;
}
void drawFilledTriangle(CanvasTriangle triangle,double** depth_buffer,double near,double far){
    order_triangle(&triangle);

    CanvasPoint v1 = triangle.vertices[0];
    CanvasPoint v2 = triangle.vertices[1];
    CanvasPoint v3 = triangle.vertices[2];
    v1.depth = compute_depth(v1.depth,near,far);
    v2.depth = compute_depth(v2.depth,near,far);
    v3.depth = compute_depth(v3.depth,near,far);
    double slope = (v2.y - v1.y)/(v3.y - v1.y);
    int newX = v1.x + slope * (v3.x - v1.x);
    double newZ = v1.depth +  (double)slope * (v3.depth - v1.depth);
    CanvasPoint v4 = CanvasPoint(newX,v2.y,newZ);

    Colour c = triangle.colour;
    // drawLine(v2,v4,Colour(255,255,255));

    //fill top triangle
    float invslope1 = (v2.x - v1.x) / (v2.y - v1.y);
    float invslope2 = (v4.x - v1.x) / (v4.y - v1.y);
    double depthslope1 = (v2.depth - v1.depth) / (double)(v2.y - v1.y);
    double depthslope2 = (v4.depth - v1.depth) / (double)(v2.y - v1.y);
    float curx1 = v1.x;
    float curx2 = v1.x;
    double curDepth1 = v1.depth;
    double curDepth2 = v1.depth;

    for (int y = v1.y; y <= v2.y; y++) {
        float x_max = std::max(curx1,curx2);
        float x_min = std::min(curx1,curx2);
        float dx = x_max - x_min;

        double depth = curx1 < curx2 ? curDepth1 : curDepth2;
        double d_depth = curx1 < curx2 ? (curDepth2 - curDepth1)/dx : (curDepth1 - curDepth2)/dx;

        for(int x = x_min; x <= x_max; x++){
            if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT){
                if(depth < depth_buffer[x][y]){
                    depth_buffer[x][y] = depth;
                    window.setPixelColour(x, y, c.packed_colour());
                    // std::cout << depth << '\n';
                }
            }
            depth += d_depth;
        }
        curDepth1 += depthslope1;
        curDepth2 += depthslope2;
        curx1 += invslope1;
        curx2 += invslope2;
    }

   //  //fill bottom triangle
    float invslope3 = (v3.x - v2.x) / (v3.y - v2.y);
    float invslope4 = (v3.x - v4.x) / (v3.y - v4.y);
    double depthslope3 = (v3.depth - v2.depth) / (double)(v3.y - v2.y);
    double depthslope4 = (v3.depth - v4.depth) / (double)(v3.y - v2.y);

    float curx3 = v3.x;
    float curx4 = v3.x;

    double curDepth3 = v3.depth;
    double curDepth4 = v3.depth;
    // std::cout << curDepth3 << '\n';

    for (int y = v3.y; y > v2.y; y--)
   {
       float x_max = std::max(curx3,curx4);
       float x_min = std::min(curx3,curx4);
       float dx = x_max - x_min;

       double depth = curx3 < curx4 ? curDepth3 : curDepth4;
       double d_depth = curx3 < curx4 ? (curDepth4 - curDepth3)/dx : (curDepth3 - curDepth4)/dx;

       for(int x = x_min; x <= x_max; x++){
           if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT){
               if(depth < depth_buffer[x][y]){
                   depth_buffer[x][y] = depth;
                   // std::cout << depth_buffer[x][y] << '\n';
                   window.setPixelColour(x, y, c.packed_colour());
               }
           }
           depth += d_depth;
       }
     curx3 -= invslope3;
     curx4 -= invslope4;
     curDepth3 -= depthslope3;
     curDepth4 -= depthslope4;
   }
}
void drawFilledTriangle(CanvasTriangle triangle){
    //sort vertices in order (y position)
    order_triangle(&triangle);
    CanvasPoint v1 = triangle.vertices[0];
    CanvasPoint v2 = triangle.vertices[1];
    CanvasPoint v3 = triangle.vertices[2];
    float slope = (v2.y - v1.y)/(v3.y - v1.y);
    int newX = v1.x + slope * (v3.x - v1.x);
    CanvasPoint v4 = CanvasPoint(newX,v2.y);

    Colour c = triangle.colour;
    // drawLine(v2,v4,Colour(255,255,255));

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

void drawLine(CanvasPoint start, CanvasPoint end, Colour c){
  float xDiff = end.x - start.x;
  float yDiff = end.y - start.y;
  float numberOfSteps = std::max(abs(xDiff), abs(yDiff));
  float xStepSize = xDiff/numberOfSteps;
  float yStepSize = yDiff/numberOfSteps;
  uint32_t colour = (255<<24) + (int(c.red)<<16) + (int(c.green)<<8) + int(c.blue);

  // std::cout << numberOfSteps << '\n';
  // std::cout << xStepSize << " " << yStepSize << '\n';
  // std::cout << start << end << '\n';

  float x = start.x;
  float y = start.y;

  for (int i = 0; i < numberOfSteps+1; i++) {
      window.setPixelColour((int) x, (int) y, colour);

      x += xStepSize;
      y += yStepSize;
      // std::cout << x << " " << y << '\n';

      // if (x == end.x) std::cout << "hooray" << '\n';
  }
  // window.setPixelColour((int) end.x, (int) end.y, colour);

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

void drawBox(std::vector<ModelTriangle> modelTriangles, float focalLength) {
    // stepBack = dv, focalLength = di

    window.clearPixels();
    std::vector<CanvasTriangle> triangles;

    double **depth_buffer;
    double dimX = WIDTH;
    double dimY = HEIGHT;

    depth_buffer = malloc2dArray(dimX, dimY);

    for(int x = 0; x < WIDTH; x++){
        for(int y = 0; y < HEIGHT; y++){
            depth_buffer[x][y] = std::numeric_limits<float>::infinity();
        }
    }

    double near = infinity;
    double far = 0;
    for (int i = 0; i < (int) modelTriangles.size(); i++) {
        std::vector<CanvasPoint> points;
        for (int j = 0; j < 3; j++) {
            glm::vec3 wrtCamera = (modelTriangles[i].vertices[j] - cameraPos) * cameraOrientation;
            // std::cout << triangles[i].vertices[j].x << " " << triangles[i].vertices[j].y << " " << triangles[i].vertices[j].z << '\n';
            // std::cout << wrtCamera.x << " " << wrtCamera.y << " " << wrtCamera.z << '\n';
            // std::cout << '\n';
            float ratio = focalLength/(-wrtCamera.z);

            int x = wrtCamera.x * ratio + WIDTH/2;
            int y = (-wrtCamera.y) * ratio + HEIGHT/2;

            if(-wrtCamera.z > far){
                far = -wrtCamera.z;
            }
            if(-wrtCamera.z < near){
                near = -wrtCamera.z;
            }
            CanvasPoint point = CanvasPoint(x, y,-wrtCamera.z);
            // std::cout << modelTriangles[i].vertices[j].z << '\n';
            points.push_back(point);
        }
        CanvasTriangle triangle = CanvasTriangle(points[0], points[1], points[2], modelTriangles[i].colour);
        triangles.push_back(triangle);
    }

    for(int i = 0; i < (int)triangles.size(); i++){
        drawFilledTriangle(triangles[i],depth_buffer,near,far);
    }
    free(depth_buffer);
    // std::cout << near << '\n';
    // std::cout << far << '\n';
}

void lookAt(glm::vec3 point) {
    glm::vec3 forward = glm::normalize(cameraPos - point);
    glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0, -1, 0)));
    glm::vec3 up = glm::normalize(glm::cross(forward, right));
    // std::cout << up.x << " " << up.y << " " << up.z <<  '\n';
    cameraOrientation = glm::inverse(glm::transpose(glm::mat3(right, up, forward)));
    //cameraOrientation = glm::transpose(glm::mat3(right, up, forward));

    // std::cout << glm::to_string(cameraOrientation) << '\n';
}

void orbit(std::vector<ModelTriangle> triangles,int* counter,int* flag,glm::vec3* translation){
    *counter +=  1;
    // *counter *=  -1;
    if(*counter %  50 == 0) *flag *= -1;
    window.clearPixels();
    // std::cout << std::put_time(nullptr) << '\n';
    translation->x -=  10;
    // cameraPos.y +=  10;
    // cameraPos.x -= 10;
    // cameraPos.z += cos(*counter) * 10;

    // std::cout << *counter << '\n';
    // cameraPos.y += sin(*counter) * 10;

    // cameraPos.z += sin(*counter) * 0;
    lookAt(glm::vec3(0,0,-300));
    drawBox(triangles,FOCALLENGTH);
    window.renderFrame();
    // std::cout << "orbit test" << '\n';

}

// EVENT HANDLING


bool handleEvent(SDL_Event event, glm::vec3* translation, glm::vec3* rotationAngles, bool* orbit)
{
    bool toUpdate = true;
    if(event.type == SDL_KEYDOWN) {
        // translate left
        if(event.key.keysym.sym == SDLK_a) translation->x -= 10;
        // translate right
        if(event.key.keysym.sym == SDLK_d) translation->x += 10;
        // translate up
        if(event.key.keysym.sym == SDLK_w) translation->y += 10;
        // translate down
        if(event.key.keysym.sym == SDLK_s) translation->y -= 10;
        // translate back
        if(event.key.keysym.sym == SDLK_x) translation->z += 10;
        // translate front
        if(event.key.keysym.sym == SDLK_e) translation->z -= 10;
        if(event.key.keysym.sym == SDLK_o) *orbit = true;
        // rotate left
        if(event.key.keysym.sym == SDLK_LEFT) rotationAngles->y -= 0.1;
        // rotate right
        if(event.key.keysym.sym == SDLK_RIGHT) rotationAngles->y += 0.1;
        // rotate up
        if(event.key.keysym.sym == SDLK_UP) rotationAngles->x -= 0.1;
        // rotate down
        if(event.key.keysym.sym == SDLK_DOWN) rotationAngles->x += 0.1;

        // look at
        if(event.key.keysym.sym == SDLK_SPACE) {
            lookAt(glm::vec3(20,0,-20));
            toUpdate = false;
        }

        // std::cout << translation->x << " " << translation->y << " " << translation->z << std::endl;
    }
    else if(event.type == SDL_MOUSEBUTTONDOWN) std::cout << "MOUSE CLICKED" << std::endl;

    return toUpdate;
}


// APPLY TRANSFORMATIONS TO CAMERA


void update(glm::vec3 translation, glm:: vec3 rotationAngles) {
    glm::mat3 rotationX = glm::mat3(glm::vec3(1, 0, 0),
                                    glm::vec3(0, cos(rotationAngles.x), -sin(rotationAngles.x)),
                                    glm::vec3(0, sin(rotationAngles.x), cos(rotationAngles.x)));

    glm::mat3 rotationY = glm::mat3(glm::vec3(cos(rotationAngles.y), 0.0, sin(rotationAngles.y)),
                                    glm::vec3(0.0, 1.0, 0.0),
                                    glm::vec3(-sin(rotationAngles.y), 0.0, cos(rotationAngles.y)));

    cameraOrientation *= rotationX;
    cameraOrientation *= rotationY;

    cameraPos += translation * glm::inverse(cameraOrientation);
    // lookAtPos.x += 10;
    // lookAt(lookAtPos);

    //
    // std::cout << glm::row(rotationY, 0).x << " " << glm::row(rotationY, 0).y << " " << glm::row(rotationY, 0).z << '\n';
    // std::cout << glm::row(rotationY, 1).x << " " << glm::row(rotationY, 1).y << " " << glm::row(rotationY, 1).z << '\n';
    // std::cout << glm::row(rotationY, 2).x << " " << glm::row(rotationY, 2).y << " " << glm::row(rotationY, 2).z << '\n';
    // std::cout << "" << '\n';

}

double **malloc2dArray(int dimX, int dimY)
{
    int i;
    double **array = (double **) malloc(dimX * sizeof(double *));

    for (i = 0; i < dimX; i++) {
        array[i] = (double *) malloc(dimY * sizeof(double));
    }
    return array;
}
