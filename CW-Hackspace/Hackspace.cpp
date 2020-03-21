#include <ModelTriangle.h>
#include <CanvasTriangle.h>
#include <DrawingWindow.h>
#include <RayTriangleIntersection.h>
#include <Utils.h>
#include <glm/glm.hpp>
#include <fstream>
#include <vector>
#include <map>
#include <glm/gtx/string_cast.hpp>


#define WIDTH 640
#define HEIGHT 480
#define FOCALLENGTH 250
#define FOV 90
#define INTENSITY 300000
#define AMBIENCE 0.4
#define WORKING_DIRECTORY "cornell-box/"
#define BOX_SCALE 50
#define LOGO_SCALE 0.5

using glm::vec3;

// helper functions
void print_vec3(vec3 point);
double **malloc2dArray(int dimX, int dimY);
void order_triangle(CanvasTriangle *triangle);
void order_textured_triangle(CanvasTriangle *triangle, CanvasTriangle *texture);
std::vector<float> interpolate(float start, float end, int noOfValues);
std::vector<glm::vec3> interpolate3(glm::vec3 start, glm::vec3 end, int noOfValues);
vec3 cramer_rule(glm::mat3 DEMatrix,vec3 SPVector);
bool isEqualTriangle(ModelTriangle t1,ModelTriangle t2);
//load colours from the window to a vector
std::vector<Colour> loadColours();


// file readers
std::vector<Colour> readPPM(std::string filename,int* width, int* height);
//given filename and dimensions create a ppm file
void writePPM(std::string filename,int width, int height, std::vector<Colour> colours);
std::map<std::string,Colour> readMTL(std::string filename);
std::vector<ModelTriangle> readOBJ(std::string filename, std::string mtlName, float scale);
void displayPicture(std::vector<Colour> payload,int width,int height);

// rasteriser
void drawLine(CanvasPoint start,CanvasPoint end,Colour c);
void drawTriangle(CanvasTriangle triangle);
void drawFilledTriangle(CanvasTriangle triangle);
void drawFilledTriangle(CanvasTriangle triangle, double** depth_buffer,double near,double far);
void drawTexturedTriangle(CanvasTriangle triangle,CanvasTriangle texture,std::vector<Colour> payload,int width,int height);
void drawBox(std::vector<ModelTriangle> triangles, float focalLength);

// raytracer
glm::vec3 computeRay(float x,float y,float fov);
RayTriangleIntersection getIntersection(glm::vec3 ray,std::vector<ModelTriangle> modelTriangles,vec3 origin);
void drawBoxRayTraced(std::vector<ModelTriangle> triangles);

// event handling
void lookAt(glm::vec3 point);
bool handleEvent(SDL_Event event, glm::vec3* translation, glm::vec3* rotationAngles);
void update(glm::vec3 translation, glm::vec3 rotationAngles);

// lighting
vec3 computenorm(ModelTriangle t);
float calcProximity(vec3 point,ModelTriangle t,std::vector<ModelTriangle> triangles,vec3 lightPos);
float calcBrightness(glm::vec3 point,ModelTriangle t,std::vector<ModelTriangle> triangles,std::vector<vec3> light_positions);
// generative geometry
void diamondSquare(double** pointHeights, int width, double currentSize);
std::vector<ModelTriangle> generateGeometry(double** pointHeights, int width, int scale);


// GLOBAL VARIABLES //


DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
glm::vec3 cameraPos = glm::vec3(0, 0, 300);
glm::vec3 box_lightPos = glm::vec3(-0.2,4.8,-3.043);
glm::vec3 box_lightPos1 = glm::vec3(2,4.8,-3.043);
glm::vec3 logo_lightPos = glm::vec3(300,59,15);
glm::vec3 lightPos = box_lightPos1;
std::vector<vec3> light_positions = {box_lightPos};
glm::vec3 lightColour = glm::vec3(1,1,1);

glm::mat3 cameraOrientation = glm::mat3();
float infinity = std::numeric_limits<float>::infinity();;
double depth_buffer[WIDTH][HEIGHT];
int mode = 1;
int textureWidth;
int textureHeight;
std::vector<Colour> texture = readPPM("HackspaceLogo/texture.ppm", &textureWidth, &textureHeight);


int main(int argc, char* argv[])
{
    for(int x = 0; x < WIDTH; x++){
        for(int y = 0; y < HEIGHT; y++){
            depth_buffer[x][y] = std::numeric_limits<float>::infinity();
        }
    }
    SDL_Event event;
    // for (float i = -2; i <= 2; i+=0.5) {
    //     // std::cout << i << '\n';
    //     vec3 lightPos = vec3(i,4.8,-3.043);
    //     print_vec3(lightPos);
    //     light_positions.push_back(lightPos);
    // }
    // int width1;
    // int height;
    // std::vector<Colour> colours = readPPM("test.ppm",&width1,&height);
    // writePPM("test1.ppm",width1,height,colours);
    std::vector<ModelTriangle> triangles = readOBJ("cornell-box.obj", "cornell-box.mtl", BOX_SCALE );

    int width = 5;
    double** grid = malloc2dArray(width, width);


    std::vector<ModelTriangle> generatedTriangles = generateGeometry(grid, width, 50);

    drawBox(triangles, FOCALLENGTH);

    window.renderFrame();

    int count = 0;
    while(true)
    {
        glm::vec3 translation = glm::vec3(0,0,0);
        glm::vec3 rotationAngles = glm::vec3(0,0,0);
        bool isUpdate = false;

        // We MUST poll for events - otherwise the window will freeze !
        if(window.pollForInputEvents(&event)) {
            isUpdate = handleEvent(event, &translation, &rotationAngles);
        }

        if (isUpdate) {
            update(translation, rotationAngles);

            // RENAMED WIREFRAME TO DRAW
            if (mode == 1 || mode == 2) drawBox(triangles, FOCALLENGTH);
            else if (mode == 3) {
                time_t tic;
                time(&tic);
                drawBoxRayTraced(triangles);
                time_t toc;
                time(&toc);
                std::cout << "runtime: " << toc-tic << " seconds" << '\n';
            }

            else if (mode == 4) {
                drawBox(generatedTriangles, FOCALLENGTH);
            }

            // Need to render the frame at the end, or nothing actually gets shown on the screen !
            window.renderFrame();
            // std::vector<Colour> colours = loadColours();
            // std::string filename = "image" + std::to_string(count) + ".ppm";
            // writePPM(filename,WIDTH,HEIGHT,colours);
            count++;
        }
    }
}


// HELPER FUNCTIONS //

void print_vec3(vec3 point){
    std::cout << "("<< point.x << " " <<
    point.y << " " << point.z << ")" <<
     '\n';
}

std::vector<Colour> loadColours(){
    std::vector<Colour> colours;
    for (size_t y = 0; y < HEIGHT; y++) {
        for (size_t x = 0; x < WIDTH; x++) {
            uint32_t colour = window.getPixelColour(x,y);
            unsigned char red = (colour & 0x00FF0000) >> 16;
            unsigned char green = (colour & 0x0000FF00) >> 8;
            unsigned char blue = (colour & 0x000000FF);
            colours.push_back(Colour(red,green,blue));
        }
    }
    return colours;
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
    for(int i = 0; i < noOfValues-1; i++){
      float tempX = vals[i].x + stepX;
      float tempY = vals[i].y + stepY;
      float tempZ = vals[i].z + stepZ;
      glm::vec3 temp(tempX, tempY, tempZ);
      vals.push_back(temp);
    }
    return vals;
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

void order_textured_triangle(CanvasTriangle *triangle, CanvasTriangle *texture) {
    if(triangle->vertices[1].y < triangle->vertices[0].y){
        std::swap(triangle->vertices[0],triangle->vertices[1]);
        std::swap(texture->vertices[0],texture->vertices[1]);
    }

    if(triangle->vertices[2].y < triangle->vertices[1].y){
        std::swap(triangle->vertices[1],triangle->vertices[2]);
        std::swap(texture->vertices[1],texture->vertices[2]);

        if(triangle->vertices[1].y < triangle->vertices[0].y){
            std::swap(triangle->vertices[1],triangle->vertices[0]);
            std::swap(texture->vertices[1],texture->vertices[0]);
        }
    }
}



// FILE READING //


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
    std::vector<Colour> payload;
    if (strcmp(encoding,"P3") == 0) {
        char line[256];
        while (stream.getline(line,256)) {
            std::string* colours = split(line,' ');
            int r = stoi(colours[0]);
            int g = stoi(colours[1]);
            int b  = stoi(colours[2]);
            payload.push_back(Colour(r,g,b));
        }
    }
    else{
        char r;
        char g;
        char b;
        while(stream.get(r) &&stream.get(g)&& stream.get(b)){
            //to make it a value between 0 and 255
            unsigned char r1 = r;
            unsigned char g1 = g;
            unsigned char b1 = b;
            payload.push_back(Colour(r1,g1,b1));
        }
    }

    stream.clear();
    stream.close();
    return payload;
}

void writePPM(std::string filename,int width, int height, std::vector<Colour> colours){
    std::ofstream afile(filename, std::ios::out);

    if (afile.is_open()) {
        afile << "P3\n"; //had to use P3 encoding
        afile << "# Comment to match GIMP\n";
        afile << width << " " << height << "\n";
        afile << "255\n";
        for (size_t i = 0; i < colours.size(); i++) {
            char line[256];
            Colour c = colours[i];
            unsigned char r = c.red;
            unsigned char g  = c.green;
            unsigned char b = c.blue;
            sprintf(line, "%d %d %d\n",r,g,b);
            afile<<line;

        }
        afile.close();
    }
}

std::map<std::string,Colour> readMTL(std::string filename){
    std::map<std::string,Colour> colourMap;
    std::ifstream stream;
    stream.open(filename, std::ifstream::in);

    char newmtl[256];

    while(stream.getline(newmtl, 256, ' ')) {

        if (strcmp(newmtl, "newmtl") == 0) {
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

        // else if (strcmp(newmtl, "map_Kd") == 0) {
        //     char textureFile[256];
        //     stream.getline(textureFile, 256);
        //     std::cout << textureFile << '\n';
        //
        //     int width;
        //     int height;
        //     std::vector<Colour> textureMap = readPPM(WORKING_DIRECTORY + (std::string) textureFile, &width, &height);
        //
        //     // for (int i = 0; i < textureMap.size(); i++) {
        //     //     colourMap.push_back(textureMap[i])
        //     // }
        // }
    }
    stream.clear();
    stream.close();
    // std::cout << "finished mtl file" << '\n';
    return colourMap;
}

std::vector<ModelTriangle> readOBJ(std::string filename, std::string mtlName, float scale) {
    //The colour map is getting initialised with dummy values

    std::ifstream stream;
    stream.open(WORKING_DIRECTORY + filename, std::ifstream::in);

    char mtlFile[256];
    stream.getline(mtlFile,256,' '); //skip the mtllib
    stream.getline(mtlFile,256);

    std::map<std::string,Colour> colourMap = readMTL(WORKING_DIRECTORY + (std::string) mtlName);

    std::vector<glm::vec3> vertices;
    std::vector<TexturePoint> texturePoints;
    std::vector<ModelTriangle> modelTriangles;
    char line[256];
    Colour colour = Colour(255,255,255);
    while(stream.getline(line,256)){

        std::string* contents = split(line,' ');
        if(line[0] == 'v' && line[1] == 't'){
            // float x = std::stof(contents[1]);
            // float y = std::stof(contents[2]);
            // TexturePoint point = TexturePoint(x, y);
            // texturePoints.push_back(point);
        }

        else if(line[0] == 'u'){
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

            // int textureIndex1 = std::stoi(indexes1[1]);
            // int textureIndex2 = std::stoi(indexes2[1]);
            // int textureIndex3 = std::stoi(indexes3[1]);

            // ModelTriangle m = ModelTriangle(vertices[index1 -1], vertices[index2 - 1], vertices[index3 -1],
            //                                 texturePoints[textureIndex1-1], texturePoints[textureIndex2-1],
            //                                 texturePoints[textureIndex3-1]);
            ModelTriangle m = ModelTriangle(vertices[index1 -1], vertices[index2 - 1], vertices[index3 -1], colour);
            modelTriangles.push_back(m);
        }
    }

    for (size_t i = 0; i < light_positions.size(); i++) {
        light_positions[i] *= scale;
    }

    stream.clear();
    stream.close();

    return modelTriangles;
}

void displayPicture(std::vector<Colour> payload,int width,int height){
    for(int i = 0; i < width; i++){
        for(int j = 0; j < height; j++){
            uint32_t colour = payload[i + j * width].packed_colour();
            window.setPixelColour(i, j, colour);
        }
    }
}


// RASTERISING //


void drawLine(CanvasPoint start, CanvasPoint end, Colour c){
  float xDiff = end.x - start.x;
  float yDiff = end.y - start.y;
  float numberOfSteps = std::max(abs(xDiff), abs(yDiff));
  float xStepSize = xDiff/numberOfSteps;
  float yStepSize = yDiff/numberOfSteps;
  uint32_t colour = (255<<24) + (int(c.red)<<16) + (int(c.green)<<8) + int(c.blue);

  float x = start.x;
  float y = start.y;

  for (int i = 0; i < numberOfSteps+1; i++) {
      if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT) window.setPixelColour((int) x, (int) y, colour);

      x += xStepSize;
      y += yStepSize;
  }
}

void drawTriangle(CanvasTriangle triangle){
  Colour c = triangle.colour;
  drawLine(triangle.vertices[0],triangle.vertices[1],c);
  drawLine(triangle.vertices[1],triangle.vertices[2],c);
  drawLine(triangle.vertices[2],triangle.vertices[0],c);
}


void drawTexturedTriangle(CanvasTriangle triangle,CanvasTriangle texture,std::vector<Colour> payload,int width,int height){
    order_textured_triangle(&triangle, &texture);

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

    //fill top triangle
    std::vector<vec3> leftSide = interpolate3(vec3(v1.x,v1.y,v1.depth), vec3(v2.x,v2.y,v2.depth), v2.y-v1.y+1);
    std::vector<vec3> rightSide = interpolate3(vec3(v1.x,v1.y,v1.depth), vec3(v4.x,v4.y,v4.depth), v2.y-v1.y+1);

    for (int i = 0; i < leftSide.size(); i++) {
        vec3 start = vec3((int) leftSide[i].x, leftSide[i].y, leftSide[i].z);
        vec3 end = vec3((int) rightSide[i].x, rightSide[i].y, rightSide[i].z);
        std::vector<vec3> rake = interpolate3(start, end, std::abs(end.x-start.x)+1);

        for (int j = 0; j < rake.size(); j++) {
            int x = rake[j].x;
            int y = rake[j].y;
            double depth = rake[j].z;
            if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT){
                if(depth < depth_buffer[x][y]){
                    depth_buffer[x][y] = depth;
                    window.setPixelColour(x, y, c.packed_colour());
                }
            }
        }
    }

   //fill bottom triangle
   leftSide = interpolate3(vec3(v3.x,v3.y,v3.depth), vec3(v2.x,v2.y,v2.depth), std::abs(v2.y-v3.y)+1);
   rightSide = interpolate3(vec3(v3.x,v3.y,v3.depth), vec3(v4.x,v4.y,v4.depth), std::abs(v4.y-v3.y)+1);

    for (int i = 0; i < leftSide.size(); i++) {
        vec3 start = vec3((int) leftSide[i].x, leftSide[i].y, leftSide[i].z);
        vec3 end = vec3((int) rightSide[i].x, rightSide[i].y, rightSide[i].z);
        std::vector<vec3> rake = interpolate3(start, end, std::abs(end.x-start.x)+1);

       for (int j = 0; j < rake.size(); j++) {
           int x = rake[j].x;
           int y = rake[j].y;
           double depth = rake[j].z;
           if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT){
               if(depth < depth_buffer[x][y]){
                   depth_buffer[x][y] = depth;
                   window.setPixelColour(x, y, c.packed_colour());
               }
           }
       }
   }
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
            // std::cout << modelTriangles[i].vertices[j].x << '\n';
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
            CanvasPoint point = CanvasPoint(x, y,-wrtCamera.z, modelTriangles[i].texturePoints[j]);
            // std::cout << modelTriangles[i].vertices[j].z << '\n';
            points.push_back(point);
        }
        CanvasTriangle triangle = CanvasTriangle(points[0], points[1], points[2], modelTriangles[i].colour);
        triangles.push_back(triangle);
    }

    for(int i = 0; i < (int)triangles.size(); i++){
        if (mode == 2 || mode == 4) drawFilledTriangle(triangles[i],depth_buffer,near,far);
        else if (mode == 1) {
            triangles[i].colour = Colour(255, 255, 255);
            drawTriangle(triangles[i]);
        }
    }
    free(depth_buffer);
    // std::cout << near << '\n';
    // std::cout << far << '\n';
}


// RAYTRACING //

/*
DEMatrix * (t,u,v)= SPVector
Cramer's rule is a way to find (t,u,v)
info found here:
https://www.purplemath.com/modules/cramers.html*/
vec3 cramer_rule(glm::mat3 DEMatrix,vec3 SPVector){
    glm::vec3 negRay = glm::column(DEMatrix,0);
    glm::vec3 e0 = glm::column(DEMatrix,1);
    glm::vec3 e1 = glm::column(DEMatrix,2);
    float determinant = glm::determinant(DEMatrix);
    float determinant_x =glm::determinant(glm::mat3(SPVector,e0,e1));

    float t = determinant_x/determinant;
    if(t >=0){
        float determinant_y =glm::determinant(glm::mat3(negRay,SPVector,e1));
        float u = determinant_y/determinant;
        if(u >= 0 && u <= 1){
            float determinant_z =glm::determinant(glm::mat3(negRay,e0,SPVector));
            float v = determinant_z/determinant;
            if(v >= 0 && v<= 1){
                if(u+v<=1){
                    return vec3(t,u,v);
                }
            }
        }
    }
    return glm::vec3(infinity,-1,-1);
}

RayTriangleIntersection getIntersection(glm::vec3 ray,ModelTriangle triangle,vec3 origin){
    glm::vec3 e0 = triangle.vertices[1] - triangle.vertices[0];
    glm::vec3 e1 = triangle.vertices[2] - triangle.vertices[0];
    glm::vec3 SPVector = origin-triangle.vertices[0];
    glm::vec3 negRay = -ray;
    glm::mat3 DEMatrix(negRay, e0, e1);
    glm::vec3 possibleSolution = cramer_rule(DEMatrix,SPVector);
    glm::vec3 point = origin + ray*possibleSolution.x;
    RayTriangleIntersection r = RayTriangleIntersection(point,possibleSolution.x,triangle);
    return r;
}

glm::vec3 computeRay(float x,float y,float fov){
    //code adapted form scratch a pixel tutorial
    // https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-generating-camera-rays/generating-camera-rays
    //0.5 added as ray goes through centre of pixel
    float ndc_x = x/WIDTH;
    float ndc_y = y/HEIGHT;
    float screen_x = 2*ndc_x -1;
    float screen_y = 1 - 2*ndc_y; //as y axis is flipped
    float aspectRatio = (float) WIDTH/ (float) HEIGHT;
    //tan(..) defines the scale
    float camera_x = screen_x * aspectRatio * tan((fov/2 * M_PI/180));
    float camera_y = screen_y * tan((fov/2 * M_PI/180));
    glm::mat3 inv_camera = glm::inverse(cameraOrientation);
    glm::vec3 rayOriginWorld = (vec3(0,0,0)-cameraPos) * inv_camera;
    glm:: vec3 rayPWorld = (vec3(camera_x,camera_y,-1) - cameraPos) * inv_camera;
    //the ray origin is (0,0,0)
    glm::vec3 rayDirection = rayPWorld - rayOriginWorld;
    rayDirection = glm::normalize(rayDirection);
    return rayDirection;
}
bool isEqualTriangle(ModelTriangle t1,ModelTriangle t2){
    return (t1.vertices[0] == t2.vertices[0]
            && t1.vertices[1] == t2.vertices[1]
            && t1.vertices[2] == t2.vertices[2]);
}

void drawBoxRayTraced(std::vector<ModelTriangle> triangles){
    window.clearPixels();
    for (size_t x = 0; x < WIDTH; x++) {
        for (size_t y = 0; y < HEIGHT; y++) {
            vec3 ray1 = computeRay((x+0.5),(y+0.5),FOV);
            vec3 ray2 = computeRay((x),(y),FOV);
            vec3 ray3 = computeRay((x+1),(y),FOV);
            vec3 ray4 = computeRay((x),(y+1),FOV);
            vec3 ray5 = computeRay((x+1),(y+1),FOV);
            std::vector<vec3> rays = {ray1,ray2,ray3,ray4,ray5};
            vec3 sumColour = vec3(0,0,0);
            for (size_t r = 0; r < rays.size(); r++) {
                RayTriangleIntersection final_intersection;
                final_intersection.distanceFromCamera = infinity;
                vec3 ray = rays[r];
                vec3 newColour;
                float minDist = infinity;
                for (size_t i = 0; i < triangles.size(); i++) {
                    RayTriangleIntersection intersection = getIntersection(ray,triangles[i],cameraPos);
                    float distance = intersection.distanceFromCamera;
                    //this is valid as you are setting distance to infinity if it's invalid
                    if(distance < minDist){
                        final_intersection.distanceFromCamera = intersection.distanceFromCamera;
                        float brightness = calcBrightness(intersection.intersectionPoint,triangles[i],triangles,light_positions);
                        vec3 lightColourCorrected = lightColour * brightness;

                        vec3 oldColour = vec3(triangles[i].colour.red, triangles[i].colour.green, triangles[i].colour.blue);
                        newColour = lightColourCorrected * oldColour;


                        Colour c = Colour(newColour.x, newColour.y, newColour.z);
                        intersection.intersectedTriangle.colour = c;

                        minDist = distance;
                    }
                }
                if(final_intersection.distanceFromCamera != infinity){
                     sumColour += newColour;
                }
            }
            vec3 avgColour = (1/(float)rays.size()) * sumColour;
            Colour c = Colour(avgColour.x, avgColour.y, avgColour.z);
            window.setPixelColour(x,y,c.packed_colour());
        }
    }
}


// LIGHTING //
vec3 computenorm(ModelTriangle t){
    vec3 norm = glm::cross((t.vertices[1] - t.vertices[0]),(t.vertices[2] - t.vertices[0]));
    norm = glm::normalize(norm);
    return norm;
}

float calcProximity(glm::vec3 point,ModelTriangle t,std::vector<ModelTriangle> triangles,vec3 lightPos){
    vec3 lightDir = lightPos - point;
    float dist = glm::length(lightDir);
    lightDir = glm::normalize(lightDir);
    vec3 norm = computenorm(t);
    float dot_product = glm::dot(lightDir,norm);

    float distance = glm::distance(lightPos,point);
    float brightness = (float) INTENSITY * std::max(0.f,dot_product)*(1/(2*M_PI* distance * distance));
    if (brightness > 1) brightness = 1;
    if (brightness < AMBIENCE) brightness = AMBIENCE;
    //do shadow calc here
    //lightDir = shadowRay (lightPos - point)
    bool isShadow = false;
    for (size_t i = 0; i < triangles.size(); i++) {
        RayTriangleIntersection shadowIntersection = getIntersection(lightDir,triangles[i],point);
        if(shadowIntersection.distanceFromCamera < dist && !isEqualTriangle(shadowIntersection.intersectedTriangle,t)){
            isShadow = true;
            break;
        }
    }
    if(isShadow) brightness = AMBIENCE/2;
    // std::cout << brightness << '\n';
    return brightness;
}
float calcBrightness(glm::vec3 point,ModelTriangle t,std::vector<ModelTriangle> triangles,std::vector<vec3> light_positions){
    float brightness = 0.f;
    for (size_t i = 0; i < light_positions.size(); i++) {
        brightness += calcProximity(point,t,triangles,light_positions[i]);
        // std::cout << light_positions[i].x<<" "<<light_positions[i].y << " " << light_positions[i].z<<'\n';
    }
    // if(brightness < 0.2) brightness =0.2;
    if(brightness > 1) brightness = 1;
    return brightness;
}


// GENERATIVE GEOMETRY //


double squareStep(double** pointHeights, double centreX, double centreY, double distFromCentre, bool isEven) {
    int rightX = centreX + distFromCentre;
    int bottomY = centreY + distFromCentre;
    int leftX = centreX - distFromCentre;
    int topY = centreY - distFromCentre;

    double topLeft = pointHeights[leftX][topY];
    double topRight = pointHeights[rightX][topY];
    double bottomLeft = pointHeights[rightX][bottomY];
    double bottomRight = pointHeights[rightX][bottomY];

    // std::cout << rightX << " " << bottomY << " " << leftX << " " << topY << '\n';

    return (topLeft + topRight + bottomLeft + bottomRight)/4;
}

double diamondStep(double** pointHeights, int width, int centreX, int centreY, int distFromCentre, bool isEven) {
    int count = 4;

    int rightX = centreX + distFromCentre;
    int bottomY = centreY + distFromCentre;
    int leftX = centreX - distFromCentre;
    int topY = centreY - distFromCentre;

    double left = centreX <= 0 ? 0 : pointHeights[leftX][centreY];
    double right = centreX >= width-1 ? 0 : pointHeights[rightX][centreY];
    double top = centreY <= 0 ? 0 : pointHeights[centreX][topY];
    double bottom = centreY >= width-1 ? 0 : pointHeights[centreX][bottomY];

    // there will only be at most one point outside the grid
    if (centreX == 0 || centreY == 0 || centreX == width-1 || centreY == width-1) {
        count -= 1;
    }

    return (left + right + top + bottom)/count;

}

void diamondSquare(double** pointHeights, int width, double currentSize) {
    double half = (double) (currentSize)/2;
    if (half < 1) return;

    // square step
    for (double x = half; x < width; x += currentSize) {
        for (double y = half; y < width; y += currentSize) {
            // std::cout << x << " " << y << '\n';
            pointHeights[(int) x][(int) y] = squareStep(pointHeights, x, y, half, true);
        }
    }

    // diamond step
    bool isSide = true;
    for (double x = 0; x <= width-1; x += half) {
        if (isSide) {
            for (double y = half; y <= width-half; y += currentSize) {
                // std::cout << x << " " << y << '\n';
                pointHeights[(int) x][(int) y] = diamondStep(pointHeights, width, x, y, half, true);
            }
        }
        else {
            for (double y = 0; y <= width; y += currentSize) {
                pointHeights[(int) x][(int) y] = diamondStep(pointHeights, width, x, y, half, true);
            }
        }
        isSide = !isSide;
    }

    diamondSquare(pointHeights, width, half);
}

std::vector<ModelTriangle> generateGeometry(double** pointHeights, int width, int scale) {
    // initialise grid with random values
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < width; y++) {
            double temp = rand()%width;
            pointHeights[x][y] = temp;
        }
    }

    // run algorithm
    diamondSquare(pointHeights, width, width-1);

    // convert points into triangles to display
    std::vector<ModelTriangle> generatedTriangles;

    for (int x = 1; x < width; x++) {
        for (int y = 1; y < width; y++) {
            vec3 v1 = vec3((x-1) * scale, pointHeights[x-1][y-1] * scale, -(y-1) * scale);
            vec3 v2 = vec3((x) * scale, pointHeights[x][y-1] * scale, -(y-1) * scale);
            vec3 v3 = vec3((x-1) * scale, pointHeights[x-1][y] * scale, -(y) * scale);
            vec3 v4 = vec3((x) * scale, pointHeights[x][y] * scale,  -(y) * scale);

            ModelTriangle t1 = ModelTriangle(v1, v2, v3, Colour(255, 255, 255));
            ModelTriangle t2 = ModelTriangle(v2, v3, v4, Colour(255, 255, 255));

            generatedTriangles.push_back(t1);
            generatedTriangles.push_back(t2);
        }
    }
    return generatedTriangles;
}


// EVENT HANDLING //


void lookAt(glm::vec3 point) {
    glm::vec3 forward = glm::normalize(cameraPos - point);
    glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0, -1, 0)));
    glm::vec3 up = glm::normalize(glm::cross(forward, right));
    // std::cout << up.x << " " << up.y << " " << up.z <<  '\n';
    cameraOrientation = ((glm::mat3(right, up, forward)));
    // std::cout << glm::to_string(cameraOrientation) << '\n';
}

bool handleEvent(SDL_Event event, glm::vec3* translation, glm::vec3* rotationAngles)
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
            lookAt(glm::vec3(0, 0, 0));
            // toUpdate = false;
        }

        if(event.key.keysym.sym == SDLK_1) {
            mode = 1;
            std::cout << "Switched to wireframe mode" << '\n';
        }
        if(event.key.keysym.sym == SDLK_2) {
            mode = 2;
            std::cout << "Switched to rasteriser mode" << '\n';
        }
        if(event.key.keysym.sym == SDLK_3) {
            mode = 3;
            std::cout << "Switched to raytracer mode" << '\n';
        }
        if(event.key.keysym.sym == SDLK_4) {
            mode = 4;
            std::cout << "Testing generated geometry" << '\n';
        }

        // std::cout << translation->x << " " << translation->y << " " << translation->z << std::endl;
    }
    else if(event.type == SDL_MOUSEBUTTONDOWN) {
        std::cout << "MOUSE CLICKED" << std::endl;
        toUpdate = false;
    }
    else toUpdate = false;

    return toUpdate;
}


// APPLY TRANSFORMATIONS TO CAMERA //


void update(glm::vec3 translation, glm:: vec3 rotationAngles) {
    glm::mat3 rotationX = glm::transpose(glm::mat3(glm::vec3(1, 0, 0),
                                    glm::vec3(0, cos(rotationAngles.x), -sin(rotationAngles.x)),
                                    glm::vec3(0, sin(rotationAngles.x), cos(rotationAngles.x))));

    glm::mat3 rotationY = glm::transpose(glm::mat3(glm::vec3(cos(rotationAngles.y), 0.0, sin(rotationAngles.y)),
                                    glm::vec3(0.0, 1.0, 0.0),
                                    glm::vec3(-sin(rotationAngles.y), 0.0, cos(rotationAngles.y))));

    cameraOrientation *= rotationX;
    cameraOrientation *= rotationY;

    cameraPos += translation;
}
