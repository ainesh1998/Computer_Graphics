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
#include <GameObject.h>
#include <string>

#define WIDTH 640
#define HEIGHT 480
#define FOCALLENGTH 250
#define FOV 90
#define INTENSITY 300000
#define AMBIENCE 0.4
#define SHADOW_INTENSITY 0.5
#define WORKING_DIRECTORY ""
#define BOX_SCALE 25
#define LOGO_SCALE 0.3
#define SPHERE_SCALE 10

using glm::vec3;

// helper functions
void print_vec3(vec3 point);
double **malloc2dArray(int dimX, int dimY);
void order_triangle(CanvasTriangle *triangle);
void order_triangle(ModelTriangle *triangle);
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
// std::map<std::string,Colour> readMTL(std::string filename);
std::vector<Colour> readMTL(std::string filename,int* textureWidth, int* textureHeight, std::vector<vec3>* bump_map);
std::vector<ModelTriangle> readOBJ(std::string filename, std::string mtlName, float scale);
void displayPicture(std::vector<Colour> payload,int width,int height);

// rasteriser
void drawLine(CanvasPoint start,CanvasPoint end,Colour c);
void drawLineAntiAlias(CanvasPoint start, CanvasPoint end, Colour c, double** depth_buffer);
void drawRake(vec3 start,vec3 end,Colour c,double** depth_buffer);
void drawTriangle(CanvasTriangle triangle, double** depth_buffer);
void drawFilledTriangle(CanvasTriangle triangle, double** depth_buffer);
void drawTexturedTriangle(CanvasTriangle triangle, double** depth_buffer);
void drawBox(std::vector<ModelTriangle> triangles, float focalLength);

// raytracer
bool isFacing(ModelTriangle t, vec3 ray);
vec3 calcReflectedRay(vec3 ray,ModelTriangle t);
vec3 refract(vec3 ray,vec3 norm,float refraction_index);
vec3 getTextureColour(ModelTriangle triangle, vec3 solution, vec3 point);
glm::vec3 computeRay(float x,float y,float fov);
RayTriangleIntersection getIntersection(glm::vec3 ray,std::vector<ModelTriangle> modelTriangles,vec3 origin);
void drawBoxRayTraced(std::vector<ModelTriangle> triangles);

// lighting
vec3 computenorm(ModelTriangle t);
float calcIntensity(vec3 norm, vec3 lightPos, vec3 point, bool isBump);
float calcShadow(float brightness, std::vector<ModelTriangle> triangles, vec3 point, vec3 lightPos, ModelTriangle t);
float calcProximity(vec3 point,ModelTriangle t,std::vector<ModelTriangle> triangles,vec3 lightPos, vec3 solution);
float calcBrightness(glm::vec3 point,ModelTriangle t,std::vector<ModelTriangle> triangles,std::vector<vec3> light_positions, vec3 solution);

// gouraud and phong shading
void calcVertexNormals(std::vector<ModelTriangle> triangles);
float gouraud(ModelTriangle t, vec3 point, vec3 lightPos, vec3 solution, std::vector<ModelTriangle> triangles);
float phong(ModelTriangle t, vec3 point, vec3 lightPos, vec3 solution, std::vector<ModelTriangle> triangles);

// bump mapping
vec3 calcBumpNormal(ModelTriangle t, vec3 solution);

// generative geometry
void diamondSquare(double** pointHeights, int width, double currentSize);
std::vector<ModelTriangle> generateGeometry(double** pointHeights, int width, float scale, int intensity, int count);

// scene map
void drawScene();
void moveObject(std::string name,vec3 moveVec);
vec3 getObjectCentroid(std::string name);
void rotateObject(std::string name,vec3 rotationAngles);
void rotateAroundPoint(std::string name, vec3 rotationAngles, vec3 point);
void rotateAroundAxis(std::string name,vec3 rotationAngles);
void scaleObject(std::string name,float scale);
void scaleYObject(std::string name,float scale);

// physics
bool isCollideGround(std::vector<ModelTriangle> o1, std::vector<ModelTriangle> o2);

// event handling
void lookAt(glm::vec3 point);
bool handleEvent(SDL_Event event, glm::vec3* translation, glm::vec3* rotationAngles, glm::vec3* light_translation, bool* isStart);
void update(glm::vec3 translation, glm::vec3 rotationAngles,glm::vec3 light_translation);


// GLOBAL VARIABLES //


DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
glm::vec3 cameraPos = glm::vec3(0, 130, 280);
glm::vec3 box_lightPos = glm::vec3(-0.2,4.8,-3.043);
glm::vec3 box_lightPos1 = glm::vec3(2,4.8,-3.043);
glm::vec3 logo_lightPos = glm::vec3(300,59,15);
glm::vec3 scene_lightPos = glm::vec3(0,-50,50);
glm::vec3 scene_lightPos1 = glm::vec3(0,400,50);
glm::vec3 lightPos = box_lightPos1;
std::vector<vec3> light_positions = {scene_lightPos,scene_lightPos1};
glm::vec3 lightColour = glm::vec3(1,1,1);

glm::mat3 cameraOrientation = glm::mat3();
float infinity = std::numeric_limits<float>::infinity();
double depth_buffer[WIDTH][HEIGHT];
int mode = 1;
std::map<std::string, std::vector<ModelTriangle>> scene;
int newTriangleID = 0;
std::map<int, std::vector<vec3>> triangleVertexNormals; //given a triangle ID, return its vertex normals
int genCount = 0;
int width = 60;
double** grid = malloc2dArray(width, width);

std::vector<std::vector<Colour>> textures;
std::vector<glm::vec2> textureDimensions;
std::vector<std::vector<vec3>> bump_maps;
std::vector<glm::vec2> bumpDimensions;


int main(int argc, char* argv[])
{
    cameraPos = glm::vec3(0, 296.697, 335.07);
    lookAt(vec3(0,0,0));


    // for(int x = 0; x < WIDTH; x++){
    //     for(int y = 0; y < HEIGHT; y++){
    //         // depth_buffer[x][y] = std::numeric_limits<float>::infinity();
    //         depth_buffer[x][y] = 0;
    //
    //     }
    // }
    SDL_Event event;
    // for (float i = -2; i <= 2; i+=0.5) {
    //     // std::cout << i << '\n';
    //     vec3 lightPos = vec3(i,4.8,-3.043);
    //     print_vec3(lightPos);
    //     light_positions.push_back(lightPos);
    // }

    // SET UP SCENE
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < width; y++) {
            double temp = (rand() % (10 * 2)) - 10;
            grid[x][y] = temp;
        }
    }


    std::vector<ModelTriangle> logo_triangles = readOBJ("HackspaceLogo/logo.obj", "HackspaceLogo/materials.mtl", LOGO_SCALE );

    std::vector<ModelTriangle> box_triangles = readOBJ("cornell-box/cornell-box.obj", "cornell-box/cornell-box.mtl", BOX_SCALE );

    std::vector<ModelTriangle> block_triangles = readOBJ("extra-objects/block.obj", "extra-objects/block.mtl", BOX_SCALE );


    // std::vector<ModelTriangle> sphere_triangles = readOBJ("extra-objects/sphere.obj", "extra-objects/sphere.mtl", SPHERE_SCALE);
    // //for sphere remove texture and set colour to be white
    // for (size_t i = 0; i < sphere_triangles.size(); i++) {
    //     sphere_triangles[i].colour = Colour(255,255,255);
    //     sphere_triangles[i].isTexture = false;
    //     for (size_t j = 0; j < 3; j++) {
    //         sphere_triangles[i].texturePoints[j] = TexturePoint(-1,-1);
    //     }
    // }

    // std::vector<ModelTriangle> generated_triangles = generateGeometry(grid, width, 2.5, 10, genCount);

    std::vector<ModelTriangle> ground_triangles = readOBJ("extra-objects/ground.obj", "extra-objects/ground.mtl", 0.6);

    // std::vector<ModelTriangle> empty_box_triangles = readOBJ("extra-objects/empty-box.obj", "extra-objects/empty-box.mtl", BOX_SCALE);

    // for (size_t i = 0; i < light_positions.size(); i++) {
    //     light_positions[i] *= (float)BOX_SCALE; //cornell box light
    // }

    // calculate vertex normals for each triangle of the sphere - for gouraud and phong shading
    // calcVertexNormals(sphere_triangles);

    scene["logo"] = logo_triangles;
    scene["box"] = box_triangles;
    // scene["sphere"] = sphere_triangles;
    // scene["terrain"] = generated_triangles;
    scene["ground"] = ground_triangles;
    // scene["box"] = empty_box_triangles;
    scene["block"] = block_triangles;

    // moveObject("logo",vec3(-35,-25,-100));
    // moveObject("logo",vec3(-100,50,-100));
    moveObject("ground",vec3(0,0,-50));
    moveObject("logo",vec3(-100,500,0)); // set logo to world origin
    moveObject("box",vec3(0,-165,90));
    scaleYObject("block", 2.5);
    moveObject("block", vec3(-120,-20,-120));

    // moveObject("logo",vec3(-50,240,0));
    // rotateObject("logo",vec3(0,90,0));
    // moveObject("logo",vec3(0,0,-120));
    // // moveObject("sphere",vec3(35,100,-100)); // place sphere above red box
    // moveObject("sphere", vec3(-70, 20, -70)); // place sphere in front of blue box
    // scaleObject("ground",0.5f);

    drawScene();

    window.renderFrame();

    float velocity = 0;
    float unbounciness = 6; // the higher the value, the less bouncy the logo is
    bool hasCollided = false;
    bool hasLanded = false;
    bool isStart = false;
    int riseCount = 0;
    float riseVelocity = 2;
    float rotationSpeed = 0;
    vec3 lookAtPos = vec3(0,0,0);
    float fallVelocity = 0;
    bool hasToppled = false;
    int currentFrame = 0;

    // for falling block
    vec3 block_centroid = getObjectCentroid("block");
    vec3 blockToOrigin = glm::normalize(vec3(-block_centroid.x, 0, -block_centroid.z));

    float minAngle = infinity;
    vec3 forward;

    for (int i = 0; i < block_triangles.size(); i++) {
        vec3 norm = computenorm(scene["block"][i]);
        float angle = std::acos(glm::dot(norm, blockToOrigin));

        if (angle < minAngle) {
            forward = norm;
            minAngle = angle;
        }
    }

    forward.y = 0;
    float yAngle = std::acos(glm::dot(vec3(0,0,1), forward)) * 180.0f/M_PI;

    while(true)
    {
        glm::vec3 translation = glm::vec3(0,0,0);
        glm::vec3 rotationAngles = glm::vec3(0,0,0);
        glm::vec3 light_translation = glm::vec3(0,0,0);
        bool isUpdate = false;

        // We MUST poll for events - otherwise the window will freeze !
        if(window.pollForInputEvents(&event)) {
            isUpdate = handleEvent(event, &translation, &rotationAngles,&light_translation,&isStart);
        }


        if (isUpdate || isStart) {
            if (isStart) {
                moveObject("logo",vec3(0,-velocity,0));
                rotateObject("logo",vec3(0,rotationSpeed,0));

                vec3 block_centroid = getObjectCentroid("block");
                vec3 rotatePoint = (scene["block"][4].vertices[1] + scene["block"][4].vertices[2]) * 0.5f;
                vec3 block_centroid_y = vec3(block_centroid.x, 0, block_centroid.z);
                vec3 rotatePoint_y = vec3(rotatePoint.x, 0, rotatePoint.z);

                rotateAroundAxis("block", vec3(0, -yAngle, 0));
                rotateAroundPoint("block", vec3(fallVelocity, 0, 0), rotatePoint);
                rotateAroundPoint("block", vec3(0, yAngle, 0), block_centroid);

                if (glm::length(block_centroid_y) < glm::length(rotatePoint_y)) {
                    fallVelocity += 0.1;
                }

                else {
                    fallVelocity -= 0.1;
                }

                // toppled over
                if (scene["block"][0].vertices[2].y < rotatePoint.y) {
                    fallVelocity = 0;
                    hasToppled = true;
                }

                // standing straight
                else if (scene["block"][2].vertices[1].y <= rotatePoint.y) {
                    fallVelocity = 0;
                }

                if (isCollideGround(scene["ground"], scene["logo"]) && !hasCollided) {
                    velocity *= -1;
                    velocity += unbounciness;
                    hasCollided = true; // to remove multiple collision detections for the same collision
                    if (rotationSpeed < 1) rotationSpeed += 0.3;
                    if (!hasToppled) fallVelocity += 1.35;
                }

                else if (!isCollideGround(scene["ground"], scene["logo"])){
                    hasCollided = false;
                }

                // only increase velocity if it hasn't landed (otherwise it'll fall through the ground)
                if (!hasLanded) velocity++;
                else {
                    if (riseVelocity > 0) {
                        light_positions[0].y += riseVelocity;
                        moveObject("logo",vec3(0,riseVelocity,0));
                        moveObject("box",vec3(0,riseVelocity,0));
                        if (riseCount > 130) riseVelocity -= 0.05;
                        riseCount++;
                        lookAtPos += vec3(0, riseVelocity, 0);
                        // lookAt(lookAtPos);
                    }
                }

                // it's landed
                if (hasCollided && velocity >= 0) {
                    velocity = 0;
                    hasLanded = true;
                }
            }

            update(translation, rotationAngles,light_translation);

            if(light_translation != vec3(0,0,0)){
                std::cout << "light is at" << '\n';
                print_vec3(light_positions[0]);
            }
            drawScene();

            // Need to render the frame at the end, or nothing actually gets shown on the screen !
            window.renderFrame();

            // capping it at 200 frames because that's probably enough for 10 seconds
            if (currentFrame < 400) {
                if(isStart){
                    if (currentFrame%2 == 0) {
                        std::vector<Colour> colours = loadColours();
                        std::string filename = "video/image" + std::to_string(currentFrame/2) + ".ppm";
                        std::cout << "Creating frame " << std::to_string(currentFrame/2) << '\n';
                        writePPM(filename,WIDTH,HEIGHT,colours);
                    }
                    currentFrame++;
                }
            }
            else {
                std::cout << "Finished video" << '\n';
                isStart = false;
            }
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

void order_triangle_X(ModelTriangle *triangle){
    if(triangle->vertices[1].x < triangle->vertices[0].x){
        std::swap(triangle->vertices[0],triangle->vertices[1]);
    }

    if(triangle->vertices[2].x < triangle->vertices[1].x){
        std::swap(triangle->vertices[1],triangle->vertices[2]);
        if(triangle->vertices[1].x < triangle->vertices[0].x){
            std::swap(triangle->vertices[1],triangle->vertices[0]);
        }
    }
}

void order_triangle_Z(ModelTriangle *triangle){
    if(triangle->vertices[1].z < triangle->vertices[0].z){
        std::swap(triangle->vertices[0],triangle->vertices[1]);
    }

    if(triangle->vertices[2].z < triangle->vertices[1].z){
        std::swap(triangle->vertices[1],triangle->vertices[2]);
        if(triangle->vertices[1].z < triangle->vertices[0].z){
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

    if (comment[0] == '#') {
        stream.getline(widthText,256,' ');
        stream.getline(heightText,256);
    }
    else {
        std::string* widthHeight = split(comment, ' ');
        strcpy(widthText, widthHeight[0].c_str());
        strcpy(heightText, widthHeight[1].c_str());
    }

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

std::vector<Colour> readMTL(std::string filename,int* textureWidth, int* textureHeight, std::vector<vec3>* bump_map){
    std::vector<Colour> colours;
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
            colours.push_back(c);

            char newLine[256];
            stream.getline(newLine, 256);
        }

        else if (strcmp(newmtl, "map_Kd") == 0) {
            char textureFile[256];
            stream.getline(textureFile, 256);
            std::string* contents = split(filename,'/'); // Since the filename contains the directory
            contents[0] += "/";
            colours = readPPM( contents[0] + (std::string) textureFile, textureWidth, textureHeight);
        }

        else if (strcmp(newmtl, "map_Bump") == 0) {
            // read bump map
            char textureFile[256];
            stream.getline(textureFile, 256);
            std::string* contents = split(filename,'/'); // Since the filename contains the directory
            contents[0] += "/";
            std::vector<Colour> tempColours = readPPM( contents[0] + (std::string) textureFile, textureWidth, textureHeight);

            for (int i = 0; i < tempColours.size(); i++) {
                vec3 normal = vec3(tempColours[i].red, tempColours[i].green, tempColours[i].blue);
                normal = glm::normalize(normal);
                normal = vec3(normal.x, normal.z, normal.y);
                bump_map->push_back(normal);
            }
        }
    }
    stream.clear();
    stream.close();
    return colours;
}

std::vector<ModelTriangle> readOBJ(std::string filename, std::string mtlName, float scale) {
    std::ifstream stream;
    stream.open(WORKING_DIRECTORY + filename, std::ifstream::in);

    char mtlFile[256];
    stream.getline(mtlFile,256,' '); //skip the mtllib
    stream.getline(mtlFile,256);

    int textureWidth;
    int textureHeight;
    std::vector<vec3> bump_map;

    std::vector<Colour> colours = readMTL(WORKING_DIRECTORY + (std::string) mtlName,&textureWidth,&textureHeight,&bump_map);

    std::vector<glm::vec3> vertices;
    std::vector<TexturePoint> texturePoints;
    std::vector<BumpPoint> bumpPoints;
    std::vector<ModelTriangle> modelTriangles;
    char line[256];
    Colour colour = Colour(255,255,255);
    bool mirrored = false;
    bool isTextured = false;
    bool isBumped = false;
    bool isGlass = false;
    int textureIndex = textures.size();
    int bumpIndex = bump_maps.size();

    while(stream.getline(line,256)){
        std::string* contents = split(line,' ');
        if(contents[0].compare("o") == 0){
            mirrored = contents[1].compare("mirror") == 0;
            isGlass = contents[1].compare("short_box") == 0;
        }

        if(contents[0].compare("vt")== 0){
            float x = (int) (std::stof(contents[1]) * (textureWidth-1));
            float y = (int) (std::stof(contents[2]) * (textureHeight-1));
            TexturePoint point = TexturePoint(x, y);
            texturePoints.push_back(point);
        }

        else if(contents[0].compare("vn") == 0) {
            // vertex normals - don't think we need to do anything, it's just for the sphere
        }

        else if(contents[0].compare("usemtl") == 0){
            for (size_t i = 0; i < colours.size(); i++) {
                if(colours[i].name.compare(contents[1]) == 0){
                    colour = colours[i];
                }
            }
        }

        else if (contents[0].compare("vb") == 0) { // don't think this is legit obj
            float x = (int) (std::stof(contents[1]) * (textureWidth-1));
            float y = (int) (std::stof(contents[2]) * (textureHeight-1));
            BumpPoint point = BumpPoint(x, y);
            bumpPoints.push_back(point);
        }

        else if(contents[0].compare("v") == 0){
            float x = std::stof(contents[1]) * scale;
            float y = std::stof(contents[2]) * scale;
            float z = std::stof(contents[3]) * scale;
            vec3 newPoint = vec3(x,y,z);
            vertices.push_back(newPoint);
        }

        else if(contents[0].compare("f") == 0){
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

            ModelTriangle m;

            // for now a surface must either be textured, bumped, a mirror or regular
            // might be cool to have a bumped mirror

            if (bump_map.size() > 0) {
                int bumpIndex1 = std::stoi(indexes1[2]);
                int bumpIndex2 = std::stoi(indexes2[2]);
                int bumpIndex3 = std::stoi(indexes3[2]);

                m = ModelTriangle(vertices[index1 -1], vertices[index2 - 1], vertices[index3 -1],
                                  bumpPoints[bumpIndex1-1], bumpPoints[bumpIndex2-1],
                                  bumpPoints[bumpIndex3-1], newTriangleID);
                m.bumpIndex = bumpIndex;
                m.colour = colour;
                isBumped = true;
            }

            else if (!notTextured) {
                int textureIndex1 = std::stoi(indexes1[1]);
                int textureIndex2 = std::stoi(indexes2[1]);
                int textureIndex3 = std::stoi(indexes3[1]);

                m = ModelTriangle(vertices[index1 -1], vertices[index2 - 1], vertices[index3 -1],
                                  texturePoints[textureIndex1-1], texturePoints[textureIndex2-1],
                                  texturePoints[textureIndex3-1], newTriangleID);

                m.textureIndex = textureIndex;
                isTextured = true;
            }

            else {
                m = ModelTriangle(vertices[index1 -1], vertices[index2 - 1], vertices[index3 -1], colour, newTriangleID);
                m.isMirror = mirrored;
                m.isGlass = isGlass;
            }

            modelTriangles.push_back(m);
            newTriangleID++;
        }
    }

    if (isTextured) {
        textureDimensions.push_back(glm::vec2(textureWidth, textureHeight));
        textures.push_back(colours);
    }

    if (isBumped) {
        bumpDimensions.push_back(glm::vec2(textureWidth, textureHeight));
        bump_maps.push_back(bump_map);
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
    float numberOfSteps = std::max(abs(xDiff), abs(yDiff));

    std::vector<vec3> line = interpolate3(vec3(start.x,start.y,start.depth), vec3(end.x,end.y,end.depth), numberOfSteps+1);

    for (uint32_t i = 0; i < line.size(); i++) {
        window.setPixelColour(line[i].x, line[i].y, c.packed_colour());
    }
}

void drawLineAntiAlias(CanvasPoint start, CanvasPoint end, Colour c, double** depth_buffer) {
    // https://www.geeksforgeeks.org/anti-aliased-line-xiaolin-wus-algorithm/

    vec3 newStart = start.toVec3();
    vec3 newEnd = end.toVec3();
    int isSteep = std::abs(end.y - start.y) > std::abs(end.x - start.x);

   // swap the co-ordinates if steep so that all lines are shallow - swap back when drawing
    if (isSteep)
    {
       newStart = vec3(newStart.y, newStart.x, newStart.z);
       newEnd = vec3(newEnd.y, newEnd.x, newEnd.z);
    }
    // sort by x value
    if (newStart.x > newEnd.x)
    {
       vec3 temp = newStart;
       newStart = newEnd;
       newEnd = temp;
    }

   // interpolate the line
    std::vector<vec3> line = interpolate3(newStart, newEnd, std::abs(newEnd.x - newStart.x)+1);

    for (uint32_t i = 0; i < line.size(); i++) {
        int x = line[i].x;
        float yLine = line[i].y; // the actual y-value at point x - could be between two pixels

        // Since the y-value could be between two pixels, we calculate the distance of this y-value
        // to the pixels above and below
        float dist1 = std::abs(yLine - ((int) yLine)); // pixel above
        float dist2 = 1 - dist1; // the distance between two pixels is 1, so dist2 = 1 - dist1

        int y1 = yLine; // pixel below
        int y2 = yLine - 1; // pixel above

        if (isSteep) {
            // swap the coordinates back
            if (x >= 0 && x < HEIGHT && y1 >= 0 && y1 < WIDTH){
                if (depth_buffer[y1][x] <= line[i].z) {
                    Colour newColour1 = Colour(c.toVec3() * dist1 + Colour(window.getPixelColour(y1,x)).toVec3() * dist2);
                    Colour newColour2 = Colour(c.toVec3() * dist2 + Colour(window.getPixelColour(y2,x)).toVec3() * dist1);
                    window.setPixelColour(y1, x, newColour1.packed_colour());
                    window.setPixelColour(y2, x, newColour2.packed_colour());
                    depth_buffer[y1][x] = line[i].z;
                }
            }

        }
        else {
            if (x >= 0 && x < WIDTH && y1 >= 0 && y1 < HEIGHT){
                if (line[i].z > depth_buffer[x][y1] ) {
                    Colour newColour1 = Colour(c.toVec3() * dist1 + Colour(window.getPixelColour(x,y1)).toVec3() * dist2);
                    Colour newColour2 = Colour(c.toVec3() * dist2 + Colour(window.getPixelColour(x,y2)).toVec3() * dist1);
                    window.setPixelColour(x, y1, newColour1.packed_colour());
                    window.setPixelColour(x, y2, newColour2.packed_colour());
                    depth_buffer[x][y1] = line[i].z;
                }
            }
        }
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
            if (depth > depth_buffer[x][y]) {
                depth_buffer[x][y] = depth;
                window.setPixelColour(x, y, c.packed_colour());
            }
        }
    }
}

void drawTriangle(CanvasTriangle triangle, double** depth_buffer){
    Colour c = triangle.colour;

    drawLineAntiAlias(triangle.vertices[0],triangle.vertices[1],c,depth_buffer);
    drawLineAntiAlias(triangle.vertices[1],triangle.vertices[2],c,depth_buffer);
    drawLineAntiAlias(triangle.vertices[2],triangle.vertices[0],c,depth_buffer);
}

void drawFilledTriangle(CanvasTriangle triangle,double** depth_buffer){
    order_triangle(&triangle);

    CanvasPoint v1 = triangle.vertices[0];
    CanvasPoint v2 = triangle.vertices[1];
    CanvasPoint v3 = triangle.vertices[2];
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

   // drawTriangle(triangle, depth_buffer);

}

float compute_texture_row(float z_far,float z_near,float c_far,float c_near,float v,float textureHeight){
    float term1 = (c_far/z_far)*(textureHeight - v) + (c_near/z_near)*v;
    float term2 = (1/z_far)*(textureHeight - v) + (1/z_near)*v;
    return term1/term2;
}

void drawTexturedTriangle(CanvasTriangle triangle, double** depth_buffer){
    CanvasTriangle texturedTriangle = triangle.getTextureTriangle();
    order_textured_triangle(&triangle, &texturedTriangle);

    CanvasPoint v1 = triangle.vertices[0];
    CanvasPoint v2 = triangle.vertices[1];
    CanvasPoint v3 = triangle.vertices[2];

    double slope = (v2.y - v1.y)/(v3.y - v1.y);
    int newX = v1.x + slope * (v3.x - v1.x);
    double newZ = v1.depth +  (double) slope * (v3.depth - v1.depth);
    CanvasPoint v4 = CanvasPoint(newX, v2.y, newZ);

    if (v1.y == v3.y) return;
    CanvasPoint u1 = texturedTriangle.vertices[0];
    CanvasPoint u2 = texturedTriangle.vertices[1];
    CanvasPoint u3 = texturedTriangle.vertices[2];

    u1.x = u1.x * v1.depth;
    u2.x = u2.x * v2.depth;
    u3.x = u3.x * v3.depth;

    u1.y = u1.y * v1.depth;
    u2.y = u2.y * v2.depth;
    u3.y = u3.y * v3.depth;

    float k_x = (v3.x==v1.x)? 0 : (u3.x-u1.x)/(v3.x-v1.x);
    float k_y = (v3.y==v1.y)? 0 : (u3.y-u1.y)/(v3.y-v1.y);
    float u4_x = u1.x + k_x * (v4.x-v1.x);
    float u4_y = u1.y + k_y * (v4.y-v1.y);

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

        vec3 startTexture = vec3( textureLeft[i].x, textureLeft[i].y, textureLeft[i].z);

        vec3 endTexture = vec3(textureRight[i].x, textureRight[i].y, textureRight[i].z);

        std::vector<vec3> rakeTexture = interpolate3(startTexture, endTexture, std::abs(endTriangle.x-startTriangle.x)+1);

        int y = triangleLeft[i].y;

        for (uint32_t j = 0; j < rakeTriangle.size(); j++) {
            int x = rakeTriangle[j].x;
            float u = rakeTexture[j].x;
            float v = rakeTexture[j].y;

            double depth = rakeTriangle[j].z;
            int ui = u/depth;
            int vi = v/depth;


            if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT){
                if (depth >= depth_buffer[x][y]) {
                    depth_buffer[x][y] = depth;
                    int textureWidth = textureDimensions[triangle.textureIndex].x;
                    // int textureHeight = textureDimensions[triangle.textureIndex].y;
                    int texturePoint = ui + (vi*textureWidth);

                    //Added if guard as there were cases when texturePoint was out of bounds
                    // if(texturePoint < textureWidth*textureHeight){
                        Colour c = textures[triangle.textureIndex][texturePoint];
                        window.setPixelColour(x, y, c.packed_colour());
                    // }

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

        vec3 startTexture = vec3( textureLeft[i].x,  textureLeft[i].y, textureLeft[i].z);
        vec3 endTexture = vec3(textureRight[i].x, textureRight[i].y, textureRight[i].z);

        std::vector<vec3> rakeTexture = interpolate3(startTexture, endTexture, std::abs(endTriangle.x-startTriangle.x)+1);

        int y = triangleLeft[i].y;

        for (uint32_t j = 0; j < rakeTriangle.size(); j++) {
            int x = rakeTriangle[j].x;
            float u = rakeTexture[j].x;
            float v = rakeTexture[j].y;

            double depth = rakeTriangle[j].z;
            int ui = u/depth;
            int vi = v/depth;

            if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT){
                if (depth >= depth_buffer[x][y]) {
                    depth_buffer[x][y] = depth;

                    int textureWidth = textureDimensions[triangle.textureIndex].x;
                    // int textureHeight = textureDimensions[triangle.textureIndex].y;
                    int texturePoint = ui + (vi*textureWidth);

                    //Added if guard as there were cases when texturePoint was out of bounds
                    // if(texturePoint < textureWidth*textureHeight){
                        Colour c = textures[triangle.textureIndex][texturePoint];
                        window.setPixelColour(x, y, c.packed_colour());
                    // }

                }
            }
        }
    }
}

bool inRange(float x,float min,float max){
    return x >= min && x <= max;
}
void drawBox(std::vector<ModelTriangle> modelTriangles, float focalLength) {
    // stepBack = dv, focalLength = di

    std::vector<CanvasTriangle> triangles;

    double **depth_buffer;
    double dimX = WIDTH;
    double dimY = HEIGHT;

    depth_buffer = malloc2dArray(dimX, dimY);

    for(int x = 0; x < WIDTH; x++){
        for(int y = 0; y < HEIGHT; y++){
            depth_buffer[x][y] = 0;
        }
    }

    for (int i = 0; i < (int) modelTriangles.size(); i++) {
        std::vector<CanvasPoint> points;
        for (int j = 0; j < 3; j++) {
            glm::vec3 wrtCamera = (modelTriangles[i].vertices[j] - cameraPos) * cameraOrientation;
            float ratio = focalLength/(-wrtCamera.z);

            int x = wrtCamera.x * ratio + WIDTH/2;
            int y = (-wrtCamera.y) * ratio + HEIGHT/2;

            CanvasPoint point = CanvasPoint(x, y,-wrtCamera.z, modelTriangles[i].texturePoints[j]);
            points.push_back(point);
        }
        if(inRange(points[0].depth,100,1000)&& inRange(points[1].depth,100,1000) && inRange(points[2].depth,100,1000)){ //near plane clipping
            CanvasTriangle triangle = CanvasTriangle(points[0], points[1], points[2], modelTriangles[i].colour);
            triangle.textureIndex = modelTriangles[i].textureIndex;
            triangles.push_back(triangle);
        }

    }

    for(int i = 0; i < (int)triangles.size(); i++){
        for (int j = 0; j < 3; j++) {
            triangles[i].vertices[j].depth = 1/triangles[i].vertices[j].depth;
        }

        if (mode == 2) {
            if (triangles[i].isTexture) {
                drawTexturedTriangle(triangles[i],depth_buffer);
            }
            else drawFilledTriangle(triangles[i],depth_buffer);
        }

        else if (mode == 1) {
            triangles[i].colour = Colour(255, 255, 255);
            drawTriangle(triangles[i], depth_buffer);
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

vec3 calcReflectedRay(vec3 ray,ModelTriangle t){
    //not sure how to use mirror with multiple light sources
    vec3 incidence = ray;
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

    int textureWidth = textureDimensions[triangle.textureIndex].x;
    Colour c = textures[triangle.textureIndex][(int) texturePoint.x + (int) texturePoint.y * textureWidth];

    return vec3(c.red, c.green, c.blue);
}

//Calculate if the ModelTriangle normal is facing the camera
bool isFacing(ModelTriangle t, vec3 ray){
    vec3 toCamera = -ray; //invert the ray to get the vertex pointing towards the camera
    vec3 norm = computenorm(t);
    float val = glm::dot(toCamera,norm);
    // std::cout << val << '\n';
    return (val>=0.f);
}


//https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-shading/reflection-refraction-fresnel
vec3 refract(vec3 ray,vec3 norm,float refractive_index){
    float cosi = glm::dot(ray,norm);
    //keep cosi within -1 and 1
    if(cosi < -1) cosi = -1;
    else if(cosi > 1) cosi = 1;
    float etai = 1; //refractive index of medium the ray is in i.e air
    float etat = refractive_index; //refractive index of medium the ray is entering i.e 1.5 for glass
    vec3 n = norm;
    if(cosi < 0){
        cosi = -cosi;
    }else{
        //I think this is if the ray is leaving the surface
        std::swap(etai,etat);
        n = -norm;
    }

    float eta = etai/etat; //snell's law
    float k = 1 - eta * eta * (1 - cosi * cosi);
    return k < 0 ? vec3(0,0,0) : eta * ray + (eta * cosi - sqrtf(k)) * n;
}

//https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-shading/reflection-refraction-fresnel
// Defines proportions for refractive and reflected colour to be mixed
float fresnel(vec3 ray,vec3 norm,float refractive_index){
    float kr;
    float cosi = glm::dot(ray,norm);
    //keep cosi within -1 and 1
    if(cosi < -1) cosi = -1;
    else if(cosi > 1) cosi = 1;
    float etai = 1; //refractive index of medium the ray is in i.e air
    float etat = refractive_index; //refractive index of medium the ray is entering i.e 1.5 for glass
    float sint = etai/etat * sqrtf(std::max(0.f,1-cosi*cosi));
    if(sint >= 1) kr = 1;
    else{

        float cost = sqrtf(std::max(0.f, 1 - sint * sint));
        cosi = fabsf(cosi); //same as abs but to specify it's a float
        float rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        kr = (rs * rs + rp * rp) / 2;
    }
    return kr;
}
RayTriangleIntersection getFinalIntersection(std::vector<ModelTriangle> triangles,vec3 ray,vec3 origin,RayTriangleIntersection* original_intersection,int depth){
    RayTriangleIntersection final_intersection;
    final_intersection.distanceFromCamera = infinity;
    final_intersection.intersectedTriangle.colour = Colour(0,0,0);
    vec3 newColour;
    if(depth < 5){
        float minDist = infinity;
        for (size_t i = 0; i < triangles.size(); i++) {
            if(isFacing(triangles[i],ray)||triangles[i].isGlass){
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
        }
        // glass
        if (final_intersection.intersectedTriangle.isGlass) {
            //reflection
            vec3 point = final_intersection.intersectionPoint;
            vec3 glassReflectedRay = calcReflectedRay(ray,final_intersection.intersectedTriangle);
            RayTriangleIntersection glass_reflected_intersection = getFinalIntersection(triangles,glassReflectedRay,point,&final_intersection,depth+1);
            Colour r =glass_reflected_intersection.intersectedTriangle.colour;
            vec3 reflected_colour = vec3(r.red,r.green,r.blue);

            //refraction
            //calculate refraction vector
            float refractive_index = 1.5; //made this a variable in case we want to change it later
            vec3 norm = computenorm(final_intersection.intersectedTriangle);
            vec3 glassRefractedRay = refract(ray,norm,refractive_index);
            RayTriangleIntersection final_glass_intersection = getFinalIntersection(triangles,glassRefractedRay,point,&final_intersection,depth+1);
            Colour c = final_glass_intersection.intersectedTriangle.colour;
            vec3 refracted_colour = vec3(c.red,c.green,c.blue);

            //final colour should be a mixture of both reflection and refraction
            //fresnel defines proportion
            float kr = fresnel(ray,norm,refractive_index);
            vec3 fin_colour = reflected_colour * kr + refracted_colour * (1-kr);
            if(glassRefractedRay == vec3(0,0,0)) fin_colour = reflected_colour;
            final_intersection.intersectedTriangle.colour = Colour(fin_colour.x,fin_colour.y,fin_colour.z);
            // final_intersection = glass_reflected_intersection;

        }
        // mirror
        if (final_intersection.intersectedTriangle.isMirror) {
            //calculate mirror vector
            vec3 point = final_intersection.intersectionPoint;
            vec3 mirrorRay = calcReflectedRay(ray,final_intersection.intersectedTriangle);
            // original_intersection is used to ensure mirror doesn't reflect itself
            RayTriangleIntersection final_mirror_intersection = getFinalIntersection(triangles,mirrorRay,point,&final_intersection,1);
            Colour c = final_mirror_intersection.intersectedTriangle.colour;
            newColour = 0.8f *  vec3(c.red,c.green,c.blue); // 0.8 is to make mirror slightly darker than the real object
            final_mirror_intersection.intersectedTriangle.colour = Colour(newColour.x,newColour.y,newColour.z);
            final_intersection = final_mirror_intersection;
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
                final_intersection = getFinalIntersection(triangles,ray,cameraPos,nullptr,1);
                Colour c = final_intersection.intersectedTriangle.colour;
                vec3 newColour = {c.red,c.green,c.blue};

                if(final_intersection.distanceFromCamera != infinity){
                     sumColour += newColour;
                }
            }
            vec3 avgColour = (1/(float)rays.size()) * sumColour;
            Colour c = Colour(avgColour.x, avgColour.y, avgColour.z);
            window.setPixelColour(x,y,c.packed_colour());
        }
    }
    // std::cout << "distance:" << minDist <<'\n';
}


// LIGHTING //
vec3 computenorm(ModelTriangle t) {
    vec3 norm = glm::cross((t.vertices[1] - t.vertices[0]),(t.vertices[2] - t.vertices[0]));
    norm = glm::normalize(norm);
    return norm;
}

float calcIntensity(vec3 norm, vec3 lightPos, vec3 point, bool isBump) {
    vec3 lightDir = lightPos - point;
    lightDir = glm::normalize(lightDir);
    float dot_product = glm::dot(lightDir,norm);
    float distance = glm::distance(lightPos,point);
    float brightness = (float) INTENSITY*(1/(2*M_PI* distance * distance));
    brightness *= std::max(0.f,dot_product);

    if (!isBump) brightness *= std::max(0.f,dot_product);

    if (brightness > 1) brightness = 1;
    if (brightness < AMBIENCE) brightness = AMBIENCE;

    if (isBump) brightness *= std::max(0.f,dot_product);

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
    if(isShadow) newBrightness *= SHADOW_INTENSITY;
    return newBrightness;
}

float calcProximity(glm::vec3 point,ModelTriangle t,std::vector<ModelTriangle> triangles,vec3 lightPos, vec3 solution){
    vec3 norm = computenorm(t);

    if (t.isBump) norm = calcBumpNormal(t, solution);

    float brightness = calcIntensity(norm, lightPos, point, t.isBump);

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
    for (size_t i = 0; i < triangles.size(); i++) {
        std::vector<vec3> vertexNormals;
        for (int j = 0; j < 3; j++) {
            vec3 point = triangles[i].vertices[j];
            vec3 avgSurfaceNormal = vec3(0, 0, 0);
            int normalCount = 0;

            for (size_t k = 0; k < triangles.size(); k++) {
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
    float brightness0 = calcIntensity(vertexNormals[0], lightPos, t.vertices[0], false);
    float brightness1 = calcIntensity(vertexNormals[1], lightPos, t.vertices[1], false);
    float brightness2 = calcIntensity(vertexNormals[2], lightPos, t.vertices[2], false);

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
    float brightness = calcIntensity(norm, lightPos, point, false);

    // shadow calculation - not using the shadow ray so I'm not too sure
    if (dot <= 0) brightness = AMBIENCE/2;
    return brightness;
}


// BUMP MAPPING //


vec3 calcBumpNormal(ModelTriangle t, vec3 solution) {
    float u = solution.y;
    float v = solution.z;

    vec3 b1 = vec3(t.bumpPoints[0].x, t.bumpPoints[0].y, 0);
    vec3 b2 = vec3(t.bumpPoints[1].x, t.bumpPoints[1].y, 0);
    vec3 b3 = vec3(t.bumpPoints[2].x, t.bumpPoints[2].y, 0);

    vec3 bumpPoint = b1 + u * (b2 - b1) + v * (b3 - b1);

    int bumpWidth = bumpDimensions[t.bumpIndex].x;
    vec3 norm = bump_maps[t.bumpIndex][(int) bumpPoint.x + (int) bumpPoint.y * bumpWidth];

    return norm;
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
    if (half < 0.1) return;

    // square step
    for (double x = half; x < width; x += currentSize) {
        for (double y = half; y < width; y += currentSize) {
            std::cout << x << " " << y << '\n';
            pointHeights[(int) x][(int) y] = squareStep(pointHeights, x, y, half, true);
        }
    }

    // diamond step
    bool isSide = true;
    for (double x = 0; x <= width-1; x += half) {
        if (isSide) {
            for (double y = half; y <= width-half; y += currentSize) {
                pointHeights[(int) x][(int) y] = diamondStep(pointHeights, width, x, y, half, true);
            }
        }
        else {
            for (double y = 0; y <= width; y += currentSize) {
                // std::cout << pointHeights[(int)x][(int)y] << " " << diamondStep(pointHeights,width,  x, y, half, true) << '\n';
                pointHeights[(int) x][(int) y] = diamondStep(pointHeights, width, x, y, half, true);
            }
        }
        isSide = !isSide;
    }

    // diamondSquare(pointHeights, width, half);
}

std::vector<ModelTriangle> generateGeometry(double** pointHeights, int width, float scale, int intensity, int count) {
    // initialise grid with random values

    double currentSize = width-1;
    // run algorithm
    for (int i = 0; i < count; i++) {
        diamondSquare(pointHeights, width, currentSize);
        currentSize = currentSize/2;
        std::cout << currentSize << '\n';
    }

    // convert points into triangles to display
    std::vector<ModelTriangle> generated_triangles;

    for (int x = 1; x < width; x++) {
        for (int y = 1; y < width; y++) {
            // if (x >= width-2 || y >= width-2) {
                vec3 v1 = vec3(x-1, pointHeights[x-1][y-1], y-1) * scale;
                vec3 v2 = vec3(x, pointHeights[x][y-1], y-1) * scale;
                vec3 v3 = vec3(x-1, pointHeights[x-1][y], y) * scale;
                vec3 v4 = vec3(x, pointHeights[x][y], y) * scale;

                ModelTriangle t1 = ModelTriangle(v1, v2, v3, Colour(255, 0, 0), newTriangleID);
                ModelTriangle t2 = ModelTriangle(v2, v3, v4, Colour(255, 0, 0), newTriangleID+1);
                newTriangleID += 2;

                // std::cout << t1 << " " << t2 << '\n';

                generated_triangles.push_back(t1);
                generated_triangles.push_back(t2);
            // }
        }
    }
    return generated_triangles;
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
        }
    }
    scene[name] = triangles;
}

vec3 getObjectCentroid(std::string name) {
    std::vector<ModelTriangle> triangles = scene[name];

    //get centre of mass
    vec3 centroid = vec3(0,0,0);
    for (size_t i = 0; i < triangles.size(); i++) {
        for (size_t j = 0; j < 3; j++) {
            centroid += triangles[i].vertices[j];
        }
    }
    centroid = (1/((float) triangles.size()*3)) * centroid;

    return centroid;
}

void rotateAroundAxis(std::string name,vec3 rotationAngles){
    vec3 centroid = getObjectCentroid(name);
    rotateAroundPoint(name, rotationAngles, centroid);
}

void rotateObject(std::string name,vec3 rotationAngles){
    glm::mat3 rotation_matrix  = glm::mat3();
    rotationAngles *= M_PI/180.0f;
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
            triangles[i].vertices[j] = rotation_matrix* triangles[i].vertices[j] ;
        }
    }
    scene[name] = triangles;
}

void rotateAroundPoint(std::string name, vec3 rotationAngles, vec3 point){
    moveObject(name,vec3(-point.x,-point.y,-point.z));
    rotateObject(name, rotationAngles);
    moveObject(name, vec3(point.x,point.y,point.z));
}

void scaleObject(std::string name,float scale){
    std::vector<ModelTriangle> triangles = scene[name];
    for (size_t i = 0; i < triangles.size(); i++) {
        for (size_t j = 0; j < 3; j++) {
            triangles[i].vertices[j] *= scale;
        }
   }
   scene[name] = triangles;
}

void scaleYObject(std::string name,float scale){
    std::vector<ModelTriangle> triangles = scene[name];
    for (size_t i = 0; i < triangles.size(); i++) {
        for (size_t j = 0; j < 3; j++) {
            triangles[i].vertices[j].y *= scale;
        }
   }
   scene[name] = triangles;
}


// PHYSICS //


bool isCollideGround(std::vector<ModelTriangle> ground, std::vector<ModelTriangle> object) {
    ModelTriangle orderX = ground[0];
    ModelTriangle orderZ = ground[0];

    order_triangle_X(&orderX);
    order_triangle_Z(&orderZ);

    float leftX = orderX.vertices[0].x;
    float rightX = orderX.vertices[2].x;
    float backZ = orderZ.vertices[0].z;
    float frontZ = orderZ.vertices[2].z;

    for (int i = 0; i < ground.size(); i++) {
        float groundY = ground[i].vertices[0].y;

        for (int j = 0; j < object.size(); j++) {
            vec3 v1 = object[j].vertices[0];
            vec3 v2 = object[j].vertices[1];
            vec3 v3 = object[j].vertices[2];

            CanvasTriangle triangle = CanvasTriangle(CanvasPoint(v1.x, v1.y, v1.z), CanvasPoint(v2.x, v2.y, v2.z), CanvasPoint(v3.x, v3.y, v3.z));
            order_triangle(&triangle);

            bool xzBound = true;

            for (int k = 0; k < 3; k++) {
                bool temp = object[j].vertices[k].x >= leftX && object[j].vertices[k].x <= rightX &&
                            object[j].vertices[k].z >= backZ && object[j].vertices[k].z <= frontZ;
                xzBound = xzBound && temp;
            }

            bool yBound = triangle.vertices[0].y <= groundY && triangle.vertices[2].y >= groundY;
            if (yBound && xzBound) {
                return true;
            }
        }
    }
    return false;
}


// EVENT HANDLING //


void lookAt(glm::vec3 point) {
    glm::vec3 forward = glm::normalize(cameraPos - point);
    glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0, -1, 0)));
    glm::vec3 up = glm::normalize(glm::cross(forward, right));
    cameraOrientation = ((glm::mat3(right, up, forward)));
}

bool handleEvent(SDL_Event event, glm::vec3* translation, glm::vec3* rotationAngles,glm::vec3* light_translation, bool* isStart)
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

        if(event.key.keysym.sym == SDLK_q) *isStart = !(*isStart);


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
        // if(event.key.keysym.sym == SDLK_4) {
        //     genCount++;
        //     scene["terrain"] = generateGeometry(grid, width, 2.5, 10, genCount);
        // }

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
