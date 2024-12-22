#include "api.h"
#include "math.h"
#include "render.h"
#include "log.h"

unsigned long * frameBuffer;
int frameWidth;
int frameHeight;

void apiSetupScreenBuffer(void *buffer, int width, int height)
{
    frameBuffer = (unsigned long *)buffer;
    frameWidth = width;
    frameHeight = height;
}

void apiStartRender()
{

}

void apiEndRender()
{
}

void apiTransformVertices (VERTEX *vertices, const unsigned int &numVertices, MATRIX t)
{
    VEC3 screenCenter(frameWidth/2,frameHeight/2, 0.0f);

    for(int i=0;i<numVertices;i++)
    {
        // Transform
        VEC3 pos;// = t * vertices[i].pos;
        float r[4] = {vertices[i].pos.x, vertices[i].pos.y, vertices[i].pos.z, 1.0f};
        pos.x = r[0]*t.f[0]+r[1]*t.f[4]+r[2]*t.f[8]+r[3]*t.f[12];
        pos.y = r[0]*t.f[1]+r[1]*t.f[5]+r[2]*t.f[9]+r[3]*t.f[13];
        pos.z = r[0]*t.f[2]+r[1]*t.f[6]+r[2]*t.f[10]+r[3]*t.f[14];
        float div = 500.0f/pos.z;
        pos.x = pos.x * div;
        pos.y = pos.y * div;
        pos = pos + screenCenter;

        vertices[i].trans = pos;
    }

    //apiLog("X VALS %d max = %f min = %f \n",numVertices, max,min);
}

// vertices are of the form: fX, fY, fZ, fNX, fNY, fNZ, fU, fV
void apiSendVertices (VERTEX *vertices, const unsigned int &numVertices, unsigned short *indices, const unsigned int &numTriangles,
                     TEXTURE texture, MATRIX transformationMatrix)
{
    unsigned int colors[5] = {0x00FF0000,0x0000FF00,0x000000FF,0x00FFFF00,0x0000FFFF};
#if 1
    apiTransformVertices (vertices, numVertices, transformationMatrix);

    for (int i=0;i<numTriangles;i++)
    {
        VEC2 t0, t1, t2;
        t0.x = vertices[indices[i*3+0]].trans.x;
        t0.y = vertices[indices[i*3+0]].trans.y;
        t1.x = vertices[indices[i*3+1]].trans.x;
        t1.y = vertices[indices[i*3+1]].trans.y;
        t2.x = vertices[indices[i*3+2]].trans.x;
        t2.y = vertices[indices[i*3+2]].trans.y;

        rasterizeTriangle(frameBuffer, frameWidth, frameHeight, colors[i%5], t0, t1, t2);
    }
#else
    rasterizeTriangle(frameBuffer, frameWidth, colors[2],
                        VEC2(10.0f,frameHeight-10),
                        VEC2(frameWidth/2.0f,10.0f),
                        VEC2(frameWidth-10,frameHeight-10));
#endif
}
