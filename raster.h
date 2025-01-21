#ifndef RASTER_H
#define RASTER_H

#include "math.h"
#include <vector>

#define PARAMETERBUFFER_SIZE 1000000
#define TILE_SIZE 32

struct VERTEX
{
    VEC3 pos;
    VEC3 nor;
    VEC2 uv;
};

struct TEXTURE
{
    unsigned int width;
    unsigned int height;
    unsigned int *data;
};

enum CULLING
{
    CULL_NONE,
    CULL_BACK,
    CULL_FRONT
};

enum BLEND_MODE
{
    NONE,
    ALPHA,
    ADDITIVE
};

struct MATERIAL {
    TEXTURE baseColor;
    TEXTURE metallicRoughness;
    TEXTURE emissive;
    TEXTURE normal;
    TEXTURE reflection;
    BLEND_MODE blend_mode;
    float factor; // value from 0.0f to 1.0f
    unsigned int color; // used if texture is nullptr
    bool smooth_shade;
};

struct TR_VERTEX
{
    VEC3 pos;
    VEC2 uv;
    float intensity;
};

struct TILE_BUFFER
{
    float depth;
    float u;
    float v;
    unsigned int material;
    unsigned int color;
};

struct TRI_POINTER
{
    uintptr_t v0;
    uintptr_t v1;
    uintptr_t v2;
    unsigned int material;
};

struct TILE_POINTER
{
    uintptr_t link;
    TRI_POINTER pointer;
};

struct PARAMETER_BUFFER
{
    TR_VERTEX       * vertices;     // Transformed vertices: x,y,z, u,v, shade (6 floats)
    TRI_POINTER     * triPointers;  // Triangle vertices pointers: v0, v1, v2, material (4 uints)
    TILE_POINTER    * tilePointers; // Tile link-list (one per triangle per tile): 1 next pointer, 1 triangle pointer (2 uints)
    uintptr_t       * tileSeeds;    // Keep the last triangle pointers: 1 uint per tile
    TILE_BUFFER     * tileBuffer;   // The rectangular tile to rasterise in
    unsigned int currentVertex;
    unsigned int currentTriangle;
    unsigned int currentTilePointer;
    unsigned int numberTiles;

    unsigned int numberMaterials;
    std::vector<MATERIAL> materials;

    float invTile;
    unsigned int tiledFrameHeight;
    unsigned int tiledFrameWidth;
};

struct RENDER_STATE
{
    float  *depthBuffer;
    unsigned long *colorBuffer;

    unsigned int frameWidth;
    unsigned int frameHeight;

    MATERIAL material;

    VEC3 lightPosition;
    VEC3 eyePosition;

    VEC3 lightInvPosition;
    VEC3 eyeInvPosition;

    MATRIX worldMatrix;
    MATRIX projectionMatrix;
    MATRIX viewMatrix;

    PARAMETER_BUFFER pb;
};

void rasterStartRender();

void rasterEndRender();

void rasterClear(unsigned int color=0x00000000, float depth=0.0f);

void rasterInitialise(const int &width, const int &height, void *debugBuffer = nullptr);

void rasterRelease();

void rasterSendVertices (VERTEX *vertices, const unsigned int &numVertices, unsigned short *indices, const unsigned int &numIndices, VEC3 center);

void rasterSetMaterial(MATERIAL material);

void rasterSetWorldMatrix(MATRIX m);

void rasterSetViewMatrix(MATRIX m);

void rasterSetProjectionMatrix(MATRIX m);

void rasterSetLight(VEC3 pos);

void rasterSetEye(VEC3 pos);

#endif // RASTER_H
