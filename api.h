#ifndef API_H
#define API_H

#include "math.h"

struct VERTEX
{
    VEC3 pos;
    VEC3 trans;
    VEC3 nor;
    VEC2 uv;
};

struct TEXTURE
{
    unsigned int width;
    unsigned int height;
    unsigned int *data; // 0xAARRGGBB
};

void apiSetupScreenBuffer(void *screenBuffer, int width, int height);
void apiStartRender();
void apiEndRender();

void apiSetWorldMatrix(MATRIX m);
void apiSetViewMatrix(MATRIX m);
void apiSetProjectionMatrix(MATRIX m);

void apiSetLight(VEC3 lightPosition);

// vertices are of the form: fX, fY, fZ, fNX, fNY, fNZ, fU, fV
// Triangle list
void apiSendVertices (VERTEX *vertices, const unsigned int &numVertices, unsigned short *indices, const unsigned int &numTriangles,
                     TEXTURE texture, MATRIX transformationMatrix);


#endif // API_H
