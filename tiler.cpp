#include "tiler.h"
#include <stdlib.h>

T_TRI  *tilerTB;
VERTEX *tilerVB;
int    *tilerPB;
char   *tilerSC;

int   frameWidth, frameHeight;
int   tilerMaterial;
float tilerTM[16];
unsigned int colors[5] = {0x00FF0000,0x0000FF00,0x000000FF,0x00FFFF00,0x0000FFFF};
unsigned long *tilerDebugBuffer;

void tilerSetup(const int &width, const int &height, const unsigned int& bytesPM, void *debugBuffer)
{
    tilerTB = (T_TRI *) malloc(bytesPM);
    tilerVB = (VERTEX *) malloc(bytesPM);
    tilerPB = (int *) malloc(bytesPM);
    tilerSC = (char *) malloc((frameWidth/TILE_SIZE)*(frameHeight/TILE_SIZE)*sizeof(char));
    frameWidth = width;
    frameHeight = height;
    tilerDebugBuffer = (unsigned long *)debugBuffer;
}

void tilerRelease()
{
    if(tilerTB) { free(tilerTB); tilerTB = nullptr;}
    if(tilerVB) { free(tilerVB); tilerVB = nullptr;}
    if(tilerPB) { free(tilerPB); tilerPB = nullptr;}
}

void tilerSetMaterial(const int &materialID)
{
    tilerMaterial = materialID;
}

void tilerSetTransformation(const float transformationMatrix[16])
{
    memcpy (tilerTM, transformationMatrix, sizeof(float)*16);
}

void tilerRasterize(unsigned long *dest, const unsigned int &destWidth, const unsigned int &destHeight, const unsigned int &color,
                       VEC3 t0, VEC3 t1, VEC3 t2)
{
    if (t0.y > t1.y) std::swap(t0, t1);
    if (t0.y > t2.y) std::swap(t0, t2);
    if (t1.y > t2.y) std::swap(t1, t2);
    int total_height = t2.y - t0.y;

    for (int i = 0; i < total_height; i++)
    {
        bool second_half = i > t1.y - t0.y || t1.y == t0.y;
        int segment_height = second_half ? t2.y - t1.y : t1.y - t0.y;
        float alpha = (float)i / total_height;
        float beta = (float)(i - (second_half ? t1.y - t0.y : 0)) / segment_height;

        int Ax, Ay, Bx, By;
        Ax = int(t0.x + (t2.x - t0.x) * alpha);
        Bx = int(second_half ? t1.x + (t2.x - t1.x) * beta : t0.x + (t1.x - t0.x) * beta);
        Ay = int(t0.y + (t2.y - t0.y) * alpha);
        By = int(second_half ? t1.y + (t2.y - t1.y) * beta : t0.y + (t1.y - t0.y) * beta);
        if (Ax > Bx) { std::swap(Ax, Bx); std::swap(Ay, By);}

        unsigned long incr = (Ax + ((int)t0.y + i) * destWidth);
        if(incr < destWidth*destHeight) // ??
        {
            for (int x = Ax; x <= Bx; x++)
            {
                if(incr > 0 && incr < destWidth*destHeight) dest[incr] = color;
                incr++;
            }
        }
    }
}

inline VEC3 tilerTransform (VEC3 pos)
{
    VEC3 out;
    VEC3 screenCenter(frameWidth/2,frameHeight/2, 0.0f);

    out.x = pos.x*tilerTM[0] + pos.y*tilerTM[4] + pos.z*tilerTM[8]  + 1.0f*tilerTM[12];
    out.y = pos.x*tilerTM[1] + pos.y*tilerTM[5] + pos.z*tilerTM[9]  + 1.0f*tilerTM[13];
    out.z = pos.x*tilerTM[2] + pos.y*tilerTM[6] + pos.z*tilerTM[10] + 1.0f*tilerTM[14];

    float div = screenCenter.x/out.z;
    out.x = out.x * div;
    out.y = out.y * div;
    out = out + screenCenter;

    return out;
}

void tilerSendVertices (VERTEX *vertices, const unsigned int &numVertices, unsigned short *indices, const unsigned int &numTriangles)
{
    for (int i=0; i<numTriangles*3;)
    {
        VEC3 t0 = tilerTransform (vertices[indices[i++]].pos);
        VEC3 t1 = tilerTransform (vertices[indices[i++]].pos);
        VEC3 t2 = tilerTransform (vertices[indices[i++]].pos);

        tilerRasterize(tilerDebugBuffer, frameWidth, frameHeight, colors[i%5], t0, t1, t2);
    }
}
