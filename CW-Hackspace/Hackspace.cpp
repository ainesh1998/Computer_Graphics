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
#define WORKING_DIRECTORY ""
#define BOX_SCALE 50
#define LOGO_SCALE 0.3
#define SPHERE_SCALE 10

using glm::vec3;

// helper functions
void print_vec3(vec3 point);
double **malloc2dArray(int dimX, int dimY);
void order_triangle(CanvasTriangle *triangle);
void order_textured_triangle(CanvasTriangle *triangle, CanvasTriangle *texture);
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
void drawRake(vec3 start,vec3 end,Colour c,double** depth_buffer);
void drawTriangle(CanvasTriangle triangle);
void drawFilledTriangle(CanvasTriangle triangle, double** depth_buffer,double near,double far);
void drawTexturedTriangle(CanvasTriangle triangle, double** depth_buffer,double near,double far);
void drawBox(std::vector<ModelTriangle> triangles, float focalLength);

// raytracer
vec3 calcMirrorVec(vec3 point,ModelTriangle t);
vec3 getTextureColour(ModelTriangle triangle, vec3 solution, vec3 point);
glm::vec3 computeRay(float x,float y,float fov);
RayTriangleIntersection getIntersection(glm::vec3 ray,std::vector<ModelTriangle> modelTriangles,vec3 origin);
void drawBoxRayTraced(std::vector<ModelTriangle> triangles);

// event handling
void lookAt(glm::vec3 point);
bool handleEvent(SDL_Event event, glm::vec3* translation, glm::vec3* rotationAngles, glm::vec3* light_translation);
void update(glm::vec3 translation, glm::vec3 rotationAngles,glm::vec3 light_translation);

// lighting
vec3 computenorm(ModelTriangle t);
float calcIntensity(vec3 norm, vec3 lightPos, vec3 point);
float calcShadow(float brightness, std::vector<ModelTriangle> triangles, vec3 point, vec3 lightPos, ModelTriangle t);
float calcProximity(vec3 point,ModelTriangle t,std::vector<ModelTriangle> triangles,vec3 lightPos, vec3 solution);
float calcBrightness(glm::vec3 point,ModelTriangle t,std::vector<ModelTriangle> triangles,std::vector<vec3> light_positions, vec3 solution);

// gouraud and phong shading
void calcVertexNormals(std::vector<ModelTriangle> triangles);
float gouraud(ModelTriangle t, vec3 point, vec3 lightPos, vec3 solution, std::vector<ModelTriangle> triangles);
float phong(ModelTriangle t, vec3 point, vec3 lightPos, vec3 solution, std::vector<ModelTriangle> triangles);

// generative geometry
void diamondSquare(double** pointHeights, int width, double currentSize);
std::vector<ModelTriangle> generateGeometry(double** pointHeights, int width, int scale);

// scene map
void drawScene();

// move object by vec3 vector
void moveObject(std::string name,vec3 moveVec);
void rotateObject(std::string name,vec3 rotationAngles);

// GLOBAL VARIABLES //


DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
glm::vec3 cameraPos = glm::vec3(0, 130, 180);
glm::vec3 box_lightPos = glm::vec3(-0.2,4.8,-3.043);
glm::vec3 box_lightPos1 = glm::vec3(2,4.8,-3.043);
glm::vec3 logo_lightPos = glm::vec3(300,59,15);
glm::vec3 lightPos = box_lightPos1;
std::vector<vec3> light_positions = {box_lightPos1};
glm::vec3 lightColour = glm::vec3(1,1,1);

glm::mat3 cameraOrientation = glm::mat3();
float infinity = std::numeric_limits<float>::infinity();
double depth_buffer[WIDTH][HEIGHT];
int mode = 1;
int textureWidth;
int textureHeight;
std::vector<Colour> texture = readPPM("HackspaceLogo/texture.ppm", &textureWidth, &textureHeight);
std::map<std::string, std::vector<ModelTriangle>> scene;
int newTriangleID = 0;
std::map<int, std::vector<vec3>> triangleVertexNormals; //given a triangle ID, return its vertex normals


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

    // SET UP SCENE
    std::vector<ModelTriangle> logo_triangles = readOBJ("HackspaceLogo/logo.obj", "HackspaceLogo/materials.mtl", LOGO_SCALE );

    std::vector<ModelTriangle> box_triangles = readOBJ("cornell-box/cornell-box.obj", "cornell-box/cornell-box.mtl", BOX_SCALE );

    std::vector<ModelTriangle> sphere_triangles = readOBJ("extra-objects/sphere.obj", "", SPHERE_SCALE);

    for (size_t i = 0; i < light_positions.size(); i++) {
        light_positions[i] *= (float)BOX_SCALE; //cornell box light
    }

    // calculate vertex normals for each triangle of the sphere - for gouraud and phong shading
    calcVertexNormals(sphere_triangles);

    // scene["logo"] = logo_triangles;
    scene["box"] = box_triangles;
    scene["sphere"] = sphere_triangles;

    moveObject("logo",vec3(-35,-25,-100));
    rotateObject("logo",vec3(0,1.5,0));
    moveObject("logo",vec3(0,0,-120));
    moveObject("sphere",vec3(35,100,-100)); // place sphere above red box
    // moveObject("sphere", vec3(-70, 20, -70)); // place sphere in front of blue box

    int width = 5;
    double** grid = malloc2dArray(width, width);

    std::vector<ModelTriangle> generatedTriangles = generateGeometry(grid, width, 50);
    drawScene();

    window.renderFrame();

    int count = 0;
    while(true)
    {
        glm::vec3 translation = glm::vec3(0,0,0);
        glm::vec3 rotationAngles = glm::vec3(0,0,0);
        glm::vec3 light_translation = glm::vec3(0,0,0);
        bool isUpdate = false;

        // We MUST poll for events - otherwise the window will freeze !
        if(window.pollForInputEvents(&event)) {
            isUpdate = handleEvent(event, &translation, &rotationAngles,&light_translation);
        }

        if (isUpdate) {
            update(translation, rotationAngles,light_translation);

            std::cout << "light is at" << '\n';
            print_vec3(light_positions[0]);
            if(mode!=4)drawScene();
            else {
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

std::vector<glm::vec3> interpolate3(glm::vec3 start, glm::vec3 end, int noOfValues) {
    // print_vec3(start);
    // print_vec3(end);
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

bool isEqualTriangle(ModelTriangle t1,ModelTriangle t2){
    return (t1.vertices[0] == t2.vertices[0]
            && t1.vertices[1] == t2.vertices[1]
            && t1.vertices[2] == t2.vertices[2]);
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
    bool mirrored = false;

    while(stream.getline(line,256)){
        std::string* contents = split(line,' ');

        if(line[0]== 'o'){
            mirrored = contents[1].compare("back_wall") == 0;
        }

        if(line[0] == 'v' && line[1] == 't'){
            float x = (int) (std::stof(contents[1]) * textureWidth);
            float y = (int) (std::stof(contents[2]) * textureHeight);
            TexturePoint point = TexturePoint(x, y);
            texturePoints.push_back(point);
        }

        else if(line[0] == 'v' && line[1] == 'n') {
            // vertex normals - don't think we need to do anything, it's just for the sphere
        }

        else if(line[0] == 'u'){
            colour = colourMap[contents[1]];
        }

        else if(line[0] == 'v'){
            // if (filename.compare("extra-objects/sphere.obj")==0) std::cout << offset << '\n';
            float x = std::stof(contents[1]) * scale;
            float y = std::stof(contents[2]) * scale;
            float z = std::stof(contents[3]) * scale;
            glm::vec3 v(x,y,z);
            vertices.push_back(v);
        }

        else if(line[0] == 'f'){
            bool notTextured = contents[1][contents[1].length()-1] == '/' ||
                              contents[2][contents[2].length()-1] == '/' ||
                              contents[3][contents[3].length()-1] == '/';

            std::string* indexes1 = split(contents[1],'/');
            std::string* indexes2 = split(contents[2],'/');
            std::string* indexes3 = split(contents[3],'/');

            int index1 = std::stoi(indexes1[0]);
            int index2 = std::stoi(indexes2[0]);
            int index3 = std::stoi(indexes3[0]);

            std::string directory = filename.substr(0, 13);

            if (!notTextured && directory.compare("extra-objects") != 0) {
                int textureIndex1 = std::stoi(indexes1[1]);
                int textureIndex2 = std::stoi(indexes2[1]);
                int textureIndex3 = std::stoi(indexes3[1]);

                ModelTriangle m = ModelTriangle(vertices[index1 -1], vertices[index2 - 1], vertices[index3 -1],
                                                texturePoints[textureIndex1-1], texturePoints[textureIndex2-1],
                                                texturePoints[textureIndex3-1], newTriangleID);
                modelTriangles.push_back(m);
                newTriangleID++;
            }

            else {
                if (directory.compare("extra-objects") == 0) colour = Colour(255,255,255);
                ModelTriangle m = ModelTriangle(vertices[index1 -1], vertices[index2 - 1], vertices[index3 -1], colour, newTriangleID);
                m.isMirror = mirrored;
                modelTriangles.push_back(m);
                newTriangleID++;
            }
        }
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


void drawLine(CanvasPoint start,CanvasPoint end,Colour c){
    float xDiff = end.x - start.x;
    float yDiff = end.y - start.y;
    float zDiff = end.depth - start.depth;
    float temp = std::max(abs(xDiff), abs(yDiff));
    float numberOfSteps = std::max(temp, std::abs(zDiff));

    std::vector<vec3> line = interpolate3(vec3(start.x,start.y,start.depth), vec3(end.x,end.y,end.depth), numberOfSteps+1);
    uint32_t colour = (255<<24) + (int(c.red)<<16) + (int(c.green)<<8) + int(c.blue);

    for (uint32_t i = 0; i < line.size(); i++) {
        window.setPixelColour(line[i].x, line[i].y, colour);
    }
}

// draws HORIZONTAL lines only
void drawRake(vec3 start, vec3 end, Colour c, double** depth_buffer){
    float numberOfSteps = std::abs(end.x - start.x);
    int y = start.y;
    std::vector<vec3> rake = interpolate3(start, end, numberOfSteps+1);

    for (uint32_t i = 0; i < rake.size(); i++) {
        int x = rake[i].x; double depth = rake[i].z;
        if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT){
            if (depth < depth_buffer[x][y]) {
                depth_buffer[x][y] = depth;
                window.setPixelColour(x, y, c.packed_colour());
            }
        }
    }
}

void drawTriangle(CanvasTriangle triangle){
  Colour c = triangle.colour;
  drawLine(triangle.vertices[0],triangle.vertices[1],c);
  drawLine(triangle.vertices[1],triangle.vertices[2],c);
  drawLine(triangle.vertices[2],triangle.vertices[0],c);
}

double compute_depth(double depth,double near,double far){
    //saw equation online
    // double z = (near + far)/(far - near) + 1/depth * ((-2 * far * near)/(far - near));
    // z = 1/depth;
    double z = (1/depth -1/near)/(1/far-1/near);
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

    for (uint32_t i = 0; i < leftSide.size(); i++) {
        vec3 start = vec3((int) leftSide[i].x, leftSide[i].y, leftSide[i].z);
        vec3 end = vec3((int) rightSide[i].x, rightSide[i].y, rightSide[i].z);
        drawRake(start, end, c, depth_buffer);
    }

   //fill bottom triangle
   leftSide = interpolate3(vec3(v3.x,v3.y,v3.depth), vec3(v2.x,v2.y,v2.depth), std::abs(v2.y-v3.y)+1);
   rightSide = interpolate3(vec3(v3.x,v3.y,v3.depth), vec3(v4.x,v4.y,v4.depth), std::abs(v4.y-v3.y)+1);

    for (uint32_t i = 0; i < leftSide.size(); i++) {
        vec3 start = vec3((int) leftSide[i].x, leftSide[i].y, leftSide[i].z);
        vec3 end = vec3((int) rightSide[i].x, rightSide[i].y, rightSide[i].z);
        drawRake(start, end, c, depth_buffer);
   }
}

void drawTexturedTriangle(CanvasTriangle triangle, double** depth_buffer, double near, double far){
    CanvasTriangle texturedTriangle = triangle.getTextureTriangle();
    order_textured_triangle(&triangle, &texturedTriangle);

    CanvasPoint v1 = triangle.vertices[0];
    CanvasPoint v2 = triangle.vertices[1];
    CanvasPoint v3 = triangle.vertices[2];

    v1.depth = compute_depth(v1.depth,near,far);
    v2.depth = compute_depth(v2.depth,near,far);
    v3.depth = compute_depth(v3.depth,near,far);


    double slope = (v2.y - v1.y)/(v3.y - v1.y);
    int newX = v1.x + slope * (v3.x - v1.x);
    double newZ = v1.depth +  (double) slope * (v3.depth - v1.depth);
    CanvasPoint v4 = CanvasPoint(newX, v2.y, newZ);

    if (v1.y == v3.y) return;

    CanvasPoint u1 = texturedTriangle.vertices[0];
    CanvasPoint u2 = texturedTriangle.vertices[1];
    CanvasPoint u3 = texturedTriangle.vertices[2];

    float k_x = (v3.x==v1.x)? 0 : (u3.x-u1.x)/(v3.x-v1.x);
    float k_y = (v3.y==v1.y)? 0 :(u3.y-u1.y)/(v3.y-v1.y);
    int u4_x = u1.x + k_x * (v4.x-v1.x);
    int u4_y = u1.y + k_y * (v4.y-v1.y);

    CanvasPoint u4 = CanvasPoint(u4_x,u4_y);

    //fill top triangle
    std::vector<vec3> triangleLeft = interpolate3(vec3(v1.x,v1.y,v1.depth), vec3(v2.x,v2.y,v2.depth), (v2.y-v1.y)+1);
    std::vector<vec3> textureLeft = interpolate3(vec3(u1.x,u1.y,u1.depth), vec3(u2.x,u2.y,u2.depth), (v2.y-v1.y)+1);

    std::vector<vec3> triangleRight = interpolate3(vec3(v1.x,v1.y,v1.depth), vec3(v4.x,v4.y,v4.depth), (v2.y-v1.y)+1);
    std::vector<vec3> textureRight = interpolate3(vec3(u1.x,u1.y,u1.depth), vec3(u4.x,u4.y,u4.depth), (v2.y-v1.y)+1);

    for (uint32_t i = 0; i < triangleLeft.size(); i++) {
        vec3 startTriangle = vec3((int) triangleLeft[i].x, triangleLeft[i].y, triangleLeft[i].z);
        vec3 endTriangle = vec3((int) triangleRight[i].x, triangleRight[i].y, triangleRight[i].z);
        std::vector<vec3> rakeTriangle = interpolate3(startTriangle, endTriangle, std::abs(endTriangle.x-startTriangle.x)+1);

        vec3 startTexture = vec3((int) textureLeft[i].x, (int) textureLeft[i].y, textureLeft[i].z);
        vec3 endTexture = vec3((int) textureRight[i].x, (int) textureRight[i].y, textureRight[i].z);

        std::vector<vec3> rakeTexture = interpolate3(startTexture, endTexture, std::abs(endTriangle.x-startTriangle.x)+1);

        int y = triangleLeft[i].y;

        for (uint32_t j = 0; j < rakeTriangle.size(); j++) {
            int x = rakeTriangle[j].x;
            int ui = rakeTexture[j].x;
            int vi = rakeTexture[j].y;
            double depth = rakeTriangle[j].z;

            if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT){
                if (depth < depth_buffer[x][y]) {
                    depth_buffer[x][y] = depth;
                    Colour c = texture[ui + (vi*textureWidth)];
                    window.setPixelColour(x, y, c.packed_colour());
                }
            }
        }
    }

    // fill bottom triangle
    triangleLeft = interpolate3(vec3(v2.x,v2.y,v2.depth), vec3(v3.x,v3.y,v3.depth), (v3.y-v2.y)+1);
    textureLeft = interpolate3(vec3(u2.x,u2.y,u2.depth), vec3(u3.x,u3.y,u3.depth), (v3.y-v2.y)+1);

    triangleRight = interpolate3(vec3(v4.x,v4.y,v4.depth), vec3(v3.x,v3.y,v3.depth), (v3.y-v2.y)+1);
    textureRight = interpolate3(vec3(u4.x,u4.y,u4.depth), vec3(u3.x,u3.y,u3.depth), (v3.y-v2.y)+1);

    for (uint32_t i = 0; i < triangleLeft.size(); i++) {
        vec3 startTriangle = vec3((int) triangleLeft[i].x, triangleLeft[i].y, triangleLeft[i].z);
        vec3 endTriangle = vec3((int) triangleRight[i].x, triangleRight[i].y, triangleRight[i].z);
        std::vector<vec3> rakeTriangle = interpolate3(startTriangle, endTriangle, std::abs(endTriangle.x-startTriangle.x)+1);

        vec3 startTexture = vec3((int) textureLeft[i].x, (int) textureLeft[i].y, textureLeft[i].z);
        vec3 endTexture = vec3((int) textureRight[i].x, (int) textureRight[i].y, textureRight[i].z);

        std::vector<vec3> rakeTexture = interpolate3(startTexture, endTexture, std::abs(endTriangle.x-startTriangle.x)+1);

        int y = triangleLeft[i].y;

        for (uint32_t j = 0; j < rakeTriangle.size(); j++) {
            int x = rakeTriangle[j].x;
            int ui = rakeTexture[j].x;
            int vi = rakeTexture[j].y;
            double depth = rakeTriangle[j].z;
            if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT){
                if (depth < depth_buffer[x][y]) {
                    depth_buffer[x][y] = depth;
                    Colour c = texture[(int) ui + (int) vi * textureWidth];
                    window.setPixelColour(x, y, c.packed_colour());
                }
            }
        }
    }
}

void drawBox(std::vector<ModelTriangle> modelTriangles, float focalLength) {
    // stepBack = dv, focalLength = di

    // window.clearPixels();
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
            points.push_back(point);
        }
        CanvasTriangle triangle = CanvasTriangle(points[0], points[1], points[2], modelTriangles[i].colour);
        triangles.push_back(triangle);
    }

    for(int i = 0; i < (int)triangles.size(); i++){
        if (mode == 2 || mode == 4) {
            if (triangles[i].isTexture) {
                drawTexturedTriangle(triangles[i],depth_buffer,near,far);
            }
            else drawFilledTriangle(triangles[i],depth_buffer,near,far);
        }

        else if (mode == 1) {
            triangles[i].colour = Colour(255, 255, 255);
            drawTriangle(triangles[i]);
        }
    }
    free(depth_buffer);
}


// RAYTRACING //


/*
DEMatrix * (t,u,v)= SPVector
Cramer's rule is a way to find (t,u,v)
info found here:
https://www.purplemath.com/modules/cramers.htm*/
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
    RayTriangleIntersection r = RayTriangleIntersection(point,possibleSolution.x,triangle, possibleSolution);
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

vec3 calcMirrorVec(vec3 point,ModelTriangle t){
    //not sure how to use mirror with multiple light sources
    vec3 incidence = glm::normalize(point - cameraPos);
    vec3 norm = computenorm(t);
    vec3 reflect = incidence -  2.f *(norm * (glm::dot(incidence,norm)));
    reflect = glm::normalize(reflect);
    return reflect;
}

vec3 getTextureColour(ModelTriangle triangle, vec3 solution, vec3 point) {
    float u = solution.y;
    float v = solution.z;

    vec3 t1 = vec3(triangle.texturePoints[0].x, triangle.texturePoints[0].y, 0);
    vec3 t2 = vec3(triangle.texturePoints[1].x, triangle.texturePoints[1].y, 0);
    vec3 t3 = vec3(triangle.texturePoints[2].x, triangle.texturePoints[2].y, 0);

    vec3 texturePoint = t1 + u * (t2 - t1) + v * (t3 - t1);
    Colour c = texture[(int) texturePoint.x + (int) texturePoint.y * textureWidth];

    return vec3(c.red, c.green, c.blue);
}

RayTriangleIntersection getFinalIntersection(std::vector<ModelTriangle> triangles,vec3 ray,vec3 origin,RayTriangleIntersection* original_intersection){
    RayTriangleIntersection final_intersection;
    final_intersection.distanceFromCamera = infinity;
    final_intersection.intersectedTriangle.colour = Colour(0,0,0);
    vec3 newColour;
    float minDist = infinity;
    for (size_t i = 0; i < triangles.size(); i++) {
        RayTriangleIntersection intersection = getIntersection(ray,triangles[i],origin);
        float distance = intersection.distanceFromCamera;

        //this is valid as you are setting distance to infinity if it's invalid
        //!isEqualTriangle used to prevent acne from the mirror (doesn't affect the original raytrace)
        bool hit = (original_intersection != nullptr && !isEqualTriangle(triangles[i],original_intersection->intersectedTriangle))
                    || original_intersection == nullptr;
        if(distance < minDist && hit){
            vec3 oldColour = vec3(triangles[i].colour.red, triangles[i].colour.green, triangles[i].colour.blue);

            // texture mapping
            if (triangles[i].isTexture) {
                oldColour = getTextureColour(triangles[i], intersection.solution, intersection.intersectionPoint);
            }

            float brightness = calcBrightness(intersection.intersectionPoint,triangles[i],triangles,light_positions,intersection.solution);
            vec3 lightColourCorrected = lightColour * brightness;

            newColour = lightColourCorrected * oldColour;

            Colour c = Colour(newColour.x, newColour.y, newColour.z);
            intersection.intersectedTriangle.colour = c;
            final_intersection = intersection;
            minDist = distance;
        }
    }
    return final_intersection;
}

void drawBoxRayTraced(std::vector<ModelTriangle> triangles){
    // window.clearPixels();
    for (size_t x = 0; x < WIDTH; x++) {
        for (size_t y = 0; y < HEIGHT; y++) {
            // complex anti-aliasing - firing multiple rays according to quincux pattern

            vec3 ray1 = computeRay((x+0.5),(y+0.5),FOV);
            // vec3 ray2 = computeRay((x),(y),FOV);
            // vec3 ray3 = computeRay((x+1),(y),FOV);
            // vec3 ray4 = computeRay((x),(y+1),FOV);
            // vec3 ray5 = computeRay((x+1),(y+1),FOV);
            // std::vector<vec3> rays = {ray1,ray2,ray3,ray4,ray5};
            std::vector<vec3> rays = {ray1};
            vec3 sumColour = vec3(0,0,0);
            for (size_t r = 0; r < rays.size(); r++) {
                RayTriangleIntersection final_intersection;
                final_intersection.distanceFromCamera = infinity;
                vec3 ray = rays[r];
                final_intersection = getFinalIntersection(triangles,ray,cameraPos,nullptr);
                Colour c = final_intersection.intersectedTriangle.colour;
                vec3 newColour = {c.red,c.green,c.blue};

                // mirror
                if (final_intersection.intersectedTriangle.isMirror) {
                    //calculate mirror vector
                    // vec3 point = final_intersection.intersectionPoint;
                    // vec3 mirrorRay = calcMirrorVec(point,final_intersection.intersectedTriangle);
                    // // original_intersection is used to ensure mirror doesn't reflect itself
                    // RayTriangleIntersection final_mirror_intersection = getFinalIntersection(triangles,mirrorRay,point,&final_intersection);
                    // Colour c = final_mirror_intersection.intersectedTriangle.colour;
                    // newColour = 0.8f *  vec3(c.red,c.green,c.blue); // 0.8 is to make mirror slightly darker than the real object
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
vec3 computenorm(ModelTriangle t) {
    vec3 norm = glm::cross((t.vertices[1] - t.vertices[0]),(t.vertices[2] - t.vertices[0]));
    norm = glm::normalize(norm);
    return norm;
}

float calcIntensity(vec3 norm, vec3 lightPos, vec3 point) {
    vec3 lightDir = lightPos - point;
    lightDir = glm::normalize(lightDir);
    float dot_product = glm::dot(lightDir,norm);
    float distance = glm::distance(lightPos,point);
    float brightness = (float) INTENSITY * std::max(0.f,dot_product)*(1/(2*M_PI* distance * distance));
    if (brightness > 1) brightness = 1;
    if (brightness < AMBIENCE) brightness = AMBIENCE;

    return brightness;
}

float calcShadow(float brightness, std::vector<ModelTriangle> triangles, vec3 point, vec3 lightPos, ModelTriangle t) {
    float newBrightness = brightness;
    vec3 lightDir = lightPos - point;
    float dist = glm::length(lightDir);
    lightDir = glm::normalize(lightDir);
    bool isShadow = false;

    for (size_t i = 0; i < triangles.size(); i++) {
        RayTriangleIntersection shadowIntersection = getIntersection(lightDir,triangles[i],point);
        if(shadowIntersection.distanceFromCamera < dist && !isEqualTriangle(shadowIntersection.intersectedTriangle,t)){
            isShadow = true;
            break;
        }
    }
    if(isShadow) newBrightness = AMBIENCE/2;
    return newBrightness;
}

float calcProximity(glm::vec3 point,ModelTriangle t,std::vector<ModelTriangle> triangles,vec3 lightPos, vec3 solution){
    vec3 norm = computenorm(t);
    float brightness = calcIntensity(norm, lightPos, point);

    // true if we precalculated the vertex normals for this triangle
    if (triangleVertexNormals.find(t.ID) != triangleVertexNormals.end()) {
        // gouraud shading
        // brightness = gouraud(t, point, lightPos, solution, triangles);

        // phong shading
        brightness = phong(t, point, lightPos, solution, triangles);
    }
    else {
        // just use calcIntensity and calculate shadows like normal
        brightness = calcShadow(brightness, triangles, point, lightPos, t);
    }

    //do shadow calc here
    return brightness;
}

float calcBrightness(glm::vec3 point,ModelTriangle t,std::vector<ModelTriangle> triangles,std::vector<vec3> light_positions, vec3 solution){
    // soft shadows - multiple light sources
    float brightness = 0.f;
    for (size_t i = 0; i < light_positions.size(); i++) {
        brightness += calcProximity(point,t,triangles,light_positions[i], solution);
        // std::cout << light_positions[i].x<<" "<<light_positions[i].y << " " << light_positions[i].z<<'\n';
    }
    if(brightness > 1) brightness = 1;
    return brightness;
}


// GOURAUD AND PHONG SHADING //


void calcVertexNormals(std::vector<ModelTriangle> triangles) {
    for (int i = 0; i < triangles.size(); i++) {
        std::vector<vec3> vertexNormals;
        for (int j = 0; j < 3; j++) {
            vec3 point = triangles[i].vertices[j];
            vec3 avgSurfaceNormal = vec3(0, 0, 0);
            int normalCount = 0;

            for (int k = 0; k < triangles.size(); k++) {
                for (int l = 0; l < 3; l++) {
                    vec3 newPoint = triangles[k].vertices[l];
                    if (newPoint == point) {
                        avgSurfaceNormal += computenorm(triangles[k]);
                        normalCount++;
                    }
                }
            }
            vec3 vertexNormal = avgSurfaceNormal* (1/(float)normalCount);
            vertexNormals.push_back(vertexNormal);
        }
        triangleVertexNormals[triangles[i].ID] = vertexNormals;
    }
}

float gouraud(ModelTriangle t, vec3 point, vec3 lightPos, vec3 solution, std::vector<ModelTriangle> triangles) {
    std::vector<vec3> vertexNormals = triangleVertexNormals[t.ID];
    float brightness0 = calcIntensity(vertexNormals[0], lightPos, t.vertices[0]);
    float brightness1 = calcIntensity(vertexNormals[1], lightPos, t.vertices[1]);
    float brightness2 = calcIntensity(vertexNormals[2], lightPos, t.vertices[2]);

    float dot0 = (glm::dot(glm::normalize(lightPos-t.vertices[0]), vertexNormals[0]));
    float dot1 = (glm::dot(glm::normalize(lightPos-t.vertices[1]), vertexNormals[1]));
    float dot2 = (glm::dot(glm::normalize(lightPos-t.vertices[2]), vertexNormals[2]));
    float dot = dot0 + solution.y*(dot1-dot0) + solution.z*(dot2-dot0);

    float brightness = brightness0 + solution.y*(brightness1-brightness0) + solution.z*(brightness2-brightness0);

    // shadow calculation - not using the shadow ray so I'm not too sure
    if(dot <= 0) brightness = AMBIENCE/2;

    return brightness;
}

float phong(ModelTriangle t, vec3 point, vec3 lightPos, vec3 solution, std::vector<ModelTriangle> triangles) {
    std::vector<vec3> vertexNormals = triangleVertexNormals[t.ID];
    vec3 norm = vertexNormals[0] + solution.y*(vertexNormals[1]-vertexNormals[0]) + solution.z*(vertexNormals[2]-vertexNormals[0]);
    float dot = glm::dot(glm::normalize(lightPos-point), norm);
    float brightness = calcIntensity(norm, lightPos, point);

    // shadow calculation - not using the shadow ray so I'm not too sure
    if (dot <= 0) brightness = AMBIENCE/2;
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

            ModelTriangle t1 = ModelTriangle(v1, v2, v3, Colour(255, 255, 255), newTriangleID);
            ModelTriangle t2 = ModelTriangle(v2, v3, v4, Colour(255, 255, 255), newTriangleID+1);
            newTriangleID += 2;

            generatedTriangles.push_back(t1);
            generatedTriangles.push_back(t2);
        }
    }
    return generatedTriangles;
}

//SCENE//

void drawScene(){
    window.clearPixels();
    std::map<std::string,std::vector<ModelTriangle>>::iterator it;
    std::vector<ModelTriangle> triangles;
    for (it=scene.begin(); it!=scene.end(); ++it){
        //append triangle list
        triangles.insert(triangles.end(),it->second.begin(),it->second.end());
    }
    if(mode==3){
        time_t tic;
        time(&tic);
        drawBoxRayTraced(triangles);
        time_t toc;
        time(&toc);
        std::cout << "runtime: " << toc-tic << " seconds" << '\n';
    }
    else drawBox(triangles,FOCALLENGTH);
}

void moveObject(std::string name,vec3 moveVec){
    std::vector<ModelTriangle> triangles = scene[name];
    for (size_t i = 0; i < triangles.size(); i++) {
        for (size_t j = 0; j < 3; j++) {
            triangles[i].vertices[j] += moveVec;
            // triangles[i].vertices[j].x += moveVec.x;
            // triangles[i].vertices[j].y += moveVec.y;
            // triangles[i].vertices[j].z += moveVec.z;
        }
        // std::cout << triangles[i] << '\n';
    }
    scene[name] = triangles;
}

void rotateObject(std::string name,vec3 rotationAngles){
    glm::mat3 rotation_matrix  = glm::mat3();
    glm::mat3 rotationX = glm::transpose(glm::mat3(glm::vec3(1, 0, 0),
                                    glm::vec3(0, cos(rotationAngles.x), -sin(rotationAngles.x)),
                                    glm::vec3(0, sin(rotationAngles.x), cos(rotationAngles.x))));

    glm::mat3 rotationY = glm::transpose(glm::mat3(glm::vec3(cos(rotationAngles.y), 0.0, sin(rotationAngles.y)),
                                    glm::vec3(0.0, 1.0, 0.0),
                                    glm::vec3(-sin(rotationAngles.y), 0.0, cos(rotationAngles.y))));
    rotation_matrix *= rotationX;
    rotation_matrix *= rotationY;

    std::vector<ModelTriangle> triangles = scene[name];
    for (size_t i = 0; i < triangles.size(); i++) {
        for (size_t j = 0; j < 3; j++) {
            triangles[i].vertices[j] = triangles[i].vertices[j] * rotation_matrix;
        }
        // std::cout << triangles[i] << '\n';
    }
    scene[name] = triangles;
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

bool handleEvent(SDL_Event event, glm::vec3* translation, glm::vec3* rotationAngles,glm::vec3* light_translation)
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

        // light translate left
        if(event.key.keysym.sym == SDLK_j) light_translation->x -= 10;
        // light translate right
        if(event.key.keysym.sym == SDLK_l) light_translation->x += 10;
        // light translate up
        if(event.key.keysym.sym == SDLK_i) light_translation->y += 10;
        // light translate down
        if(event.key.keysym.sym == SDLK_k) light_translation->y -= 10;
        // light translate back
        if(event.key.keysym.sym == SDLK_o) light_translation->z += 10;
        // light translate front
        if(event.key.keysym.sym == SDLK_p) light_translation->z -= 10;


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


void update(glm::vec3 translation, glm:: vec3 rotationAngles, glm::vec3 light_translation) {
    glm::mat3 rotationX = glm::transpose(glm::mat3(glm::vec3(1, 0, 0),
                                    glm::vec3(0, cos(rotationAngles.x), -sin(rotationAngles.x)),
                                    glm::vec3(0, sin(rotationAngles.x), cos(rotationAngles.x))));

    glm::mat3 rotationY = glm::transpose(glm::mat3(glm::vec3(cos(rotationAngles.y), 0.0, sin(rotationAngles.y)),
                                    glm::vec3(0.0, 1.0, 0.0),
                                    glm::vec3(-sin(rotationAngles.y), 0.0, cos(rotationAngles.y))));

    cameraOrientation *= rotationX;
    cameraOrientation *= rotationY;

    cameraPos += translation * glm::inverse(cameraOrientation);
    for (size_t i = 0; i < light_positions.size(); i++) {
        light_positions[i] += light_translation;
    }
}
