#include "api.h"
#include "math.h"
#include "render.h"
#include "log.h"
#include "tiler.h"

void apiSetupScreenBuffer(void *buffer, int width, int height)
{
    tilerSetup(width, height, MEGABYTE, buffer);
}

void apiStartRender()
{

}

void apiEndRender()
{
}


// vertices are of the form: fX, fY, fZ, fNX, fNY, fNZ, fU, fV
void apiSendVertices (VERTEX *vertices, const unsigned int &numVertices, unsigned short *indices, const unsigned int &numTriangles,
                     TEXTURE texture, MATRIX transformationMatrix)
{
    //tilerSetMaterial(const int &materialID);
    tilerSetTransformation(transformationMatrix.f);
    tilerSendVertices (vertices, numVertices, indices, numTriangles);
}
