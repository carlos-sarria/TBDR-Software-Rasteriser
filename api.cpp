#include "api.h"
#include "math.h"
#include "log.h"
#include "tiler.h"

void apiSetWorldMatrix(MATRIX m) {tilerSetWorldMatrix(m);};
void apiSetViewMatrix(MATRIX m)  {tilerSetViewMatrix(m);};
void apiSetProjectionMatrix(MATRIX m)  {tilerSetProjectionMatrix(m);};
void apiSetLight(VEC3 lightPosition) {tilerSetLight(lightPosition);}

void apiSetupScreenBuffer(void *buffer, int width, int height)
{
    tilerSetup(width, height, MEGABYTE, buffer);
}

void apiStartRender()
{
    tilerClear();
}

void apiEndRender()
{
}

// vertices are of the form: fX, fY, fZ, fNX, fNY, fNZ, fU, fV
void apiSendVertices (VERTEX *vertices, const unsigned int &numVertices, unsigned short *indices, const unsigned int &numIndices,
                     TEXTURE texture, MATRIX transformationMatrix)
{
    tilerSetMaterial(texture);
    tilerSendVertices (vertices, numVertices, indices, numIndices);
}
