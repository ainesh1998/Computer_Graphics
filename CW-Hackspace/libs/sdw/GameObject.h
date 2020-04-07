class GameObject
{
    public:
        std::string name;
        std::vector<ModelTriangle> triangles;
        std::vector<Colour> texture;
        int textureHeight;
        int textureWidth;

        GameObject(std::string n, std::vector<ModelTriangle> modelTriangles) {
            name = n;
            triangles = modelTriangles;
        }

        GameObject(std::string n, std::vector<ModelTriangle> modelTriangles, std::vector<Colour> payload, int tHeight, int tWidth) {
            name = n;
            triangles = modelTriangles;
            texture = payload;
            textureHeight = tHeight;
            textureWidth = tWidth;
        }

};
