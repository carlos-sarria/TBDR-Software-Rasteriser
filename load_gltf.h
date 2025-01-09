#ifndef LOAD_GLTF_H
#define LOAD_GLTF_H

#include "math.h"
#include "raster.h"
#include "tiny_gltf.h"

struct TRANSFORM
{
    VEC3 translation;
    QUATERNION rotation;
    VEC3 scale;
};

struct LIGHT
{
    unsigned int type;
    TRANSFORM transform;
};

struct CAMERA
{
    unsigned int type;
    TRANSFORM transform;
    float aspectRatio;
    VEC3 from;
    VEC3 to;
    float yfov;
    float zfar;
    float znear;
};

struct MESH
{
    VERTEX* vertexBuffer;
    unsigned short* indexBuffer;
    unsigned int vertexCount;
    unsigned int indexCount;
    TRANSFORM transform;
    int baseColorTexture;
    int metallicRoughnessTexture;
    int emissiveTexture;
    int normalTexture;
    VEC3 center;
    VEC3 boundingBoxA, boundingBoxB;
};

struct SCENE
{
    std::vector<MESH> meshes;
    std::vector<TEXTURE> textures;
    std::vector<CAMERA> cameras;
    std::vector<LIGHT> lights;
    std::string path;
};

bool loadDDS(const char* textureFileName, TEXTURE& texture);
void load_gltf(const char* fileName);
void free_gltf();

#endif // LOAD_GLTF_H
