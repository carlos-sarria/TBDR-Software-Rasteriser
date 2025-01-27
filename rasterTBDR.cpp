#include "raster.h"
//#include "log.h"
#include <stdlib.h>

#define RASTER_IMR 0 // 1 = IMR, 0 = TBDR

unsigned int debug_colors[] = {0xFFFF0000,0xFF00FF00,0xFF0000FF,0xFFFFFF00,0xFFFF00FF,0xFF00FFFF,0xFF800000,0xFF008000,0xFF000080};
unsigned int currColor = 0;
RENDER_STATE rs;

inline void boundingBox(VEC3 p0, VEC3 p1, VEC3 p2, VEC2INT &a, VEC2INT &b)
{
    if((p0.x<=p1.x) && (p0.x<=p2.x)) a.x = p0.x;
    if((p1.x<=p2.x) && (p1.x<=p0.x)) a.x = p1.x;
    if((p2.x<=p0.x) && (p2.x<=p1.x)) a.x = p2.x;

    if((p0.y<=p1.y) && (p0.y<=p2.y)) a.y = p0.y;
    if((p1.y<=p2.y) && (p1.y<=p0.y)) a.y = p1.y;
    if((p2.y<=p0.y) && (p2.y<=p1.y)) a.y = p2.y;

    if((p0.x>=p1.x) && (p0.x>=p2.x)) b.x = p0.x;
    if((p1.x>=p2.x) && (p1.x>=p0.x)) b.x = p1.x;
    if((p2.x>=p0.x) && (p2.x>=p1.x)) b.x = p2.x;

    if((p0.y>=p1.y) && (p0.y>=p2.y)) b.y = p0.y;
    if((p1.y>=p2.y) && (p1.y>=p0.y)) b.y = p1.y;
    if((p2.y>=p0.y) && (p2.y>=p1.y)) b.y = p2.y;
}
#define FOCUS 0.01f
inline unsigned int blendPBR(unsigned int U, unsigned int V)
{
    unsigned int map = U+V*rs.material.baseColor.width;
    unsigned int tex_color = (rs.material.baseColor.data==0) ? rs.material.color : rs.material.baseColor.data[map];
    unsigned int nor_color = (rs.material.normal.data==0) ? 0xFFBEBEFF : rs.material.normal.data[map];
    unsigned int met_color = (rs.material.metallicRoughness.data==0) ? 0xFFFFFFFF : rs.material.metallicRoughness.data[map];

    VEC3 color      = UNPACK(tex_color);
    VEC3 normal     = UNPACK(nor_color);
    VEC3 metal      = UNPACK(met_color);
    normal = (normal-128.0f)*(1.0f/128.0f); // from -1.0 to 1.0
    metal = metal * (1.0f/255.f); // from 0.0 to 1.0

    float shade = (normal * rs.lightInvPosition) * 0.5f + 0.5f;
    shade *= metal.x; // x: ambient occlusion
    float specular =  (shade < (1.0f-FOCUS)) ? 0.0f : (shade-(1.0f-FOCUS))*(1.0f/FOCUS)*255.0f;

    float shininess = specular*(1.0f-metal.y)*0.2f; // y: roughness, z: metallicity

    float r = shade*color.x + shininess;
    float g = shade*color.y + shininess;
    float b = shade*color.z + shininess;

    return PACK(0xFF,r,g,b);
}

// Barycentric coordinates. Cramer's rule to solve linear systems
inline void barycentric(VEC3 v0, VEC3 v1, VEC3 v2, VEC3 p, VEC3 &by)
{
    VEC3 br0 = v1 - v0;
    VEC3 br1 = v2 - v0;
    float d00 = DOT(br0, br0);
    float d01 = DOT(br0, br1);
    float d11 = DOT(br1, br1);
    float invD = 1.0 / (d00 * d11 - d01 * d01);

    VEC3 br2 = p - v0;
    float d20 = DOT(br2, br0);
    float d21 = DOT(br2, br1);
    by.y = (d11 * d20 - d01 * d21) * invD;
    by.z = (d00 * d21 - d01 * d20) * invD;
    by.x = 1.0f - by.y - by.z;
}

void transform(VERTEX *v, const unsigned int &numVertices, TR_VERTEX *meshTVB, VEC3 center)
{
    MATRIX mMVP = rs.worldMatrix * rs.viewMatrix * rs.projectionMatrix;

    MATRIX invW = rs.worldMatrix;
    invW.inverse();

    rs.lightInvPosition = invW * (rs.lightPosition-center);
    rs.lightInvPosition.normalize();

    rs.eyePosition = VEC3(0.0f,0.0f, 1.0f);
    rs.eyeInvPosition = invW * rs.eyePosition;//(center-rs.eyePosition);
    rs.eyeInvPosition.normalize();

    for (int i = 0; i < numVertices; i++)
    {
        VEC3 out;

        out.x = v[i].pos.x*mMVP.f[0] + v[i].pos.y*mMVP.f[4] + v[i].pos.z*mMVP.f[8]  + 1.0f*mMVP.f[12];
        out.y = v[i].pos.x*mMVP.f[1] + v[i].pos.y*mMVP.f[5] + v[i].pos.z*mMVP.f[9]  + 1.0f*mMVP.f[13];
        out.z = v[i].pos.x*mMVP.f[2] + v[i].pos.y*mMVP.f[6] + v[i].pos.z*mMVP.f[10] + 1.0f*mMVP.f[14];

        float shade = v[i].nor.dot(rs.lightInvPosition) * 0.5 + 0.5f;

        float div = 1.0f/out.z;
        meshTVB[i].pos.x = ceil(out.x * div * (rs.frameWidth/2) + rs.frameWidth/2); // ceil = pixel left-top co nvention
        meshTVB[i].pos.y = ceil(out.y * div * (rs.frameWidth/2) + rs.frameHeight/2);
        meshTVB[i].pos.z = div;

        meshTVB[i].uv.u = v[i].uv.u * div;
        meshTVB[i].uv.v = v[i].uv.v * div;

            meshTVB[i].intensity = shade * div;
    }
}
/// testTriangle
/// Returns true if a triangle (p0,p1,p2) is touching an orthogonal rectangle (a,b)
#define LINE_TEST(x,y) ((((x2-x1)*(y1-y)-(x1-x)*(y2-y1))<0)?-1:1)
inline bool testTriangle(VEC3 p0, VEC3 p1, VEC3 p2, VEC2 a, VEC2 b)
{
    int totalSign[3] = {0,0,0};
    float x1,x2,y1,y2;

    // line p1-p0
    x1 = p1.x; y1 = p1.y;
    x2 = p0.x; y2 = p0.y;
    totalSign[0] = LINE_TEST(a.x,a.y) + LINE_TEST(a.x,b.y) + LINE_TEST(b.x,a.y) + LINE_TEST(b.x,b.y);

    // line p2-p1
    x1 = p2.x; y1 = p2.y;
    x2 = p1.x; y2 = p1.y;
    totalSign[1] = LINE_TEST(a.x,a.y) + LINE_TEST(a.x,b.y) + LINE_TEST(b.x,a.y) + LINE_TEST(b.x,b.y);

    // line p0-p2
    x1 = p0.x; y1 = p0.y;
    x2 = p2.x; y2 = p2.y;
    totalSign[2] = LINE_TEST(a.x,a.y) + LINE_TEST(a.x,b.y) + LINE_TEST(b.x,a.y) + LINE_TEST(b.x,b.y);

    // Edge comparison that guarantees it is inside the triangle only.
    if(abs(totalSign[0])!=4 && (totalSign[1]==-4||abs(totalSign[1])!=4) && (totalSign[2]==-4||abs(totalSign[2])!=4)) return true; // edge 1
    if(abs(totalSign[1])!=4 && (totalSign[2]==-4||abs(totalSign[2])!=4) && (totalSign[0]==-4||abs(totalSign[0])!=4)) return true; // edge 2
    if(abs(totalSign[2])!=4 && (totalSign[0]==-4||abs(totalSign[0])!=4) && (totalSign[1]==-4||abs(totalSign[1])!=4)) return true; // edge 3
    if(totalSign[0]==-4 && totalSign[1]==-4 && totalSign[2]==-4) return true; // fully inside: all -4

    return false; // outside
}

inline bool cull(VEC3 v0, VEC3 v1, VEC3 v2)
{
    // Backface culling: sign = ab*ac.y - ac.x*ab.y;
    if((v0.x-v1.x)*(v0.y-v2.y) - (v0.x-v2.x)*(v0.y-v1.y) >= 0.0f) return true;

    // Depth culling (front plane only) TODO: Front plane clipping
    // Brute-force polygon removal
    if(v0.z<0.0f || v1.z<0.0f || v2.z<0.0f) return true;

    VEC2 a(0, 0);
    VEC2 b(rs.frameWidth, rs.frameHeight);
    return !testTriangle(v0, v1, v2, a, b);
}

void perfectTiling(TRI_POINTER *currentTriangle)
{
    VEC3 p0(currentTriangle->v0->pos);
    VEC3 p1(currentTriangle->v1->pos);
    VEC3 p2(currentTriangle->v2->pos);
    VEC2INT a, b;

    boundingBox(p0, p1, p2, a, b);

    a.x = truncf(a.x*rs.pb.invTile);
    b.x = truncf(b.x*rs.pb.invTile);
    a.y = truncf(a.y*rs.pb.invTile);
    b.y = truncf(b.y*rs.pb.invTile);

    for (int y = a.y; y<=b.y; y++)
    {
        unsigned int tile = a.x+y*rs.pb.tiledFrameWidth;
        for (int x = a.x; x<=b.x; x++)
        {
            if(x<rs.pb.tiledFrameWidth && y<rs.pb.tiledFrameHeight)
            {
                VEC2 a(x*TILE_SIZE, y*TILE_SIZE);
                VEC2 b(a.x+TILE_SIZE, a.y+TILE_SIZE);

                if (testTriangle(p0, p1, p2, a, b))
                {
                    uintptr_t s = rs.pb.tileSeeds[tile];
                    rs.pb.tileSeeds[tile] = (uintptr_t)&rs.pb.tilePointers[rs.pb.currentTilePointer];
                    rs.pb.tilePointers[rs.pb.currentTilePointer].next = s; // link-list, 0 marks end of list
                    rs.pb.tilePointers[rs.pb.currentTilePointer].pointer = currentTriangle;
                    rs.pb.currentTilePointer++;
                }
            }
            tile++;
        }
    }
}

// Depth comparison and varyings iterators
bool rasterISP(unsigned int tileX, unsigned int tileY)
{
    uintptr_t next = rs.pb.tileSeeds[tileX+tileY*rs.pb.tiledFrameWidth];
    VEC2 a(tileX*TILE_SIZE, tileY*TILE_SIZE);
    VEC2 b(a.x+TILE_SIZE, a.y+TILE_SIZE);

    if (next==0) return false; // Empty tile

    while(1)
    {
        TILE_POINTER *tilePointer = (TILE_POINTER*)next;
        TRI_POINTER *triPointer = tilePointer->pointer;
        next = tilePointer->next;

        TR_VERTEX *v0 = triPointer->v0;
        TR_VERTEX *v1 = triPointer->v1;
        TR_VERTEX *v2 = triPointer->v2;
        unsigned int material = triPointer->material;

        VEC3 p0(v0->pos), p1(v1->pos), p2(v2->pos);

        VEC3 vD(v0->pos.z, v1->pos.z, v2->pos.z);
        VEC3 vU(v0->uv.u, v1->uv.u, v2->uv.u);
        VEC3 vV(v0->uv.v, v1->uv.v, v2->uv.v);

        double d = 1.0f/TILE_SIZE;

        VEC3 pA(a.x,a.y,0.0f), pB(b.x-(1.0f/TILE_SIZE),a.y,0.0f), pC(a.x,b.y-(1.0f/TILE_SIZE),0.0f), bryA, bryB, bryC;

        barycentric(p0, p1, p2, pA, bryA);
        barycentric(p0, p1, p2, pB, bryB);
        barycentric(p0, p1, p2, pC, bryC);

        float bryX_incrx = (bryB.x-bryA.x)*d;
        float bryX_incry = (bryC.x-bryA.x)*d-(bryB.x-bryA.x);

        float bryY_incrx = (bryB.y-bryA.y)*d;
        float bryY_incry = (bryC.y-bryA.y)*d-(bryB.y-bryA.y);

        float bryZ_incrx = (bryB.z-bryA.z)*d;
        float bryZ_incry = (bryC.z-bryA.z)*d-(bryB.z-bryA.z);

        float D = vD.dot(bryA);
        float dB = vD.dot(bryB);
        float dC = vD.dot(bryC);
        float D_incrx = (dB-D)*d;
        float D_incry = (dC-D)*d-(dB-D);

        float U = vU.dot(bryA);
        float uB = vU.dot(bryB);
        float uC = vU.dot(bryC);
        float U_incrx = (uB-U)*d;
        float U_incry = (uC-U)*d-(uB-U);

        float V = vV.dot(bryA);
        float vB = vV.dot(bryB);
        float vC = vV.dot(bryC);
        float V_incrx = (vB-V)*d;
        float V_incry = (vC-V)*d-(vB-V);

        TILE_BUFFER *tf_ptr = rs.pb.tileBuffer;

        for(int y=a.y; y<b.y; y++)
        {
            for(int x=a.x; x<b.x; x++)
            {
                if(!(bryA.x<0.0 || bryA.y<0.0 || bryA.z<0.0))
                {
                    float depth  =  D;

                    if(depth>=tf_ptr->depth) // Depth comparison GREATER-EQUAL
                    {
                        // Update the depth buffer with the new value
                        tf_ptr->depth = depth;
                        tf_ptr->u = U;
                        tf_ptr->v = V;
                        tf_ptr->material = material;
                    }
                }
                tf_ptr++;
                bryA.x += bryX_incrx;
                bryA.y += bryY_incrx;
                bryA.z += bryZ_incrx;
                D += D_incrx;
                U += U_incrx;
                V += V_incrx;
            }
            bryA.x += bryX_incry;
            bryA.y += bryY_incry;
            bryA.z += bryZ_incry;
            D += D_incry;
            U += U_incry;
            V += V_incry;
        }
        if(next == 0) break;
    }

    return true;
}
void rasterTSP(unsigned int tileX, unsigned int tileY)
{
    VEC2INT a(tileX*TILE_SIZE, tileY*TILE_SIZE);
    VEC2INT b(a.x+TILE_SIZE, a.y+TILE_SIZE);

    TILE_BUFFER *tf_ptr = rs.pb.tileBuffer;
    for(int y=a.y; y<b.y; y++)
    {
        for(int x=a.x; x<b.x; x++)
        {
            if(tf_ptr->depth>0 && x<rs.frameWidth && y<rs.frameHeight)
            {
                float invD = 1.0f/tf_ptr->depth;

                unsigned int u = (unsigned int)(tf_ptr->u*invD*rs.material.baseColor.width) & (rs.material.baseColor.width-1);
                unsigned int v = (unsigned int)(tf_ptr->v*invD*rs.material.baseColor.height) & (rs.material.baseColor.height-1);

                rs.colorBuffer[x+y*rs.frameWidth] = blendPBR(u,v);
            }
            // else  // TEST
            // {
            //     rs.colorBuffer[x+y*rs.frameWidth] = 0xFFFF0000;
            // }
            tf_ptr++;
        }
    }
}

void rasterTA(unsigned short *indices, const unsigned int &numIndices, TR_VERTEX *meshTVB)
{
    rs.pb.materials.push_back(rs.material);

    for (int i = 0; i < numIndices;)
    {
        unsigned short in0 = indices[i++];
        unsigned short in1 = indices[i++];
        unsigned short in2 = indices[i++];

        if (cull(meshTVB[in0].pos, meshTVB[in1].pos, meshTVB[in2].pos)) continue;

        // Store this triangle
        rs.pb.triPointers[rs.pb.currentTriangle].v0 = &meshTVB[in0];
        rs.pb.triPointers[rs.pb.currentTriangle].v1 = &meshTVB[in1];
        rs.pb.triPointers[rs.pb.currentTriangle].v2 = &meshTVB[in2];
        rs.pb.triPointers[rs.pb.currentTriangle].material = rs.pb.materials.size()-1;

        perfectTiling(&rs.pb.triPointers[rs.pb.currentTriangle]);
        rs.pb.currentTriangle++;
    }
}

///////////////////////////////////
/// External API
///////////////////////////////////

void apiSetWorldMatrix(MATRIX m) { rs.worldMatrix = m; }

void apiSetViewMatrix(MATRIX m) { rs.viewMatrix = m; }

void apiSetProjectionMatrix(MATRIX m) { rs.projectionMatrix = m; }

void apiSetLight(VEC3 pos) { rs.lightPosition = pos;};
void apiSetEye(VEC3 pos) { rs.eyePosition = pos;};

void apiInitialise(const int &width, const int &height, void *debugBuffer)
{
    rs.frameWidth = width;
    rs.frameHeight = height;
    rs.depthBuffer = (float *) malloc(rs.frameWidth*rs.frameHeight*sizeof(float));
    rs.colorBuffer = (unsigned long *)debugBuffer;
    rs.material.baseColor.data = 0;
    rs.material.baseColor.height = 0;
    rs.material.baseColor.width = 0;
    rs.material.blend_mode = NONE;
    rs.material.factor = 1.0f;
    rs.material.color = 0xFFFFFFFF;
    rs.material.smooth_shade = true;

    rs.pb.invTile = 1.0f/TILE_SIZE;
    rs.pb.tiledFrameHeight = ceilf((float)rs.frameHeight*rs.pb.invTile);
    rs.pb.tiledFrameWidth = ceilf((float)rs.frameWidth*rs.pb.invTile);

    rs.pb.numberTiles   = rs.pb.tiledFrameWidth * rs.pb.tiledFrameHeight;
    rs.pb.vertices      = (TR_VERTEX *)malloc(PARAMETERBUFFER_SIZE);
    rs.pb.triPointers   = (TRI_POINTER *)malloc(PARAMETERBUFFER_SIZE);
    rs.pb.tilePointers  = (TILE_POINTER *)malloc(PARAMETERBUFFER_SIZE);
    rs.pb.tileSeeds     = (uintptr_t *)malloc(rs.pb.numberTiles*sizeof(uintptr_t));
    rs.pb.tileBuffer    = (TILE_BUFFER *)malloc(TILE_SIZE*TILE_SIZE*sizeof(TILE_BUFFER));
}

void apiRelease()
{
    if(rs.depthBuffer)      {free(rs.depthBuffer); rs.depthBuffer = nullptr;}
    if(rs.pb.vertices)      {free(rs.pb.vertices); rs.pb.vertices = nullptr;}
    if(rs.pb.triPointers)   {free(rs.pb.triPointers); rs.pb.triPointers = nullptr;}
    if(rs.pb.tilePointers)  {free(rs.pb.tilePointers); rs.pb.tilePointers = nullptr;}
    if(rs.pb.tileSeeds)     {free(rs.pb.tileSeeds); rs.pb.tileSeeds = nullptr;}
    if(rs.pb.tileBuffer)    {free(rs.pb.tileBuffer); rs.pb.tileBuffer = nullptr;}
}

void apiClear(unsigned int color, float depth)
{
    unsigned int i = rs.frameWidth*rs.frameHeight;
    while(i--)
    {
        rs.depthBuffer[i]= depth;
        rs.colorBuffer[i] = color;
    }
}

void apiSetMaterial(MATERIAL material)
{
    rs.material = material;
}

void apiStartRender()
{
    rs.pb.currentVertex = 0;
    rs.pb.currentTilePointer = 0;
    rs.pb.currentTriangle = 0;
    memset(rs.pb.tileSeeds,0,rs.pb.numberTiles*sizeof(uintptr_t));

    rs.pb.materials.clear();
}

void apiEndRender()
{
    for (int y=0; y<rs.pb.tiledFrameHeight; y++)
    {
        for (int x=0; x<rs.pb.tiledFrameWidth; x++)
        {
            memset(rs.pb.tileBuffer,0,TILE_SIZE*TILE_SIZE*sizeof(TILE_BUFFER)); // clean up tile buffer
            if(rasterISP(x,y)) rasterTSP(x,y);
        }
    }
}

void apiSendVertices (VERTEX *vertices, const unsigned int &numVertices, unsigned short *indices, const unsigned int &numIndices, VEC3 center)
{
#if 1
    // Transform vertices and allocate them sequentially
    TR_VERTEX *transformed = rs.pb.vertices + rs.pb.currentVertex*sizeof(TR_VERTEX);
    transform (vertices, numVertices, transformed, center);
    rs.pb.currentVertex += numVertices;

    // Processing the triangle list (store also sequentially)
    rasterTA(indices, numIndices, transformed);

#else // TEST single triangle
    float div = 0.1f;
    rs.pb.vertices[1] = {VEC3(10.0f,710.0f,0.1f), VEC2(1.0f*div,1.0f*div), 0.0f};
    rs.pb.vertices[0] = {VEC3(512.0f,10.0f,0.1f),   VEC2(0.5f*div,0.0f*div), 0.0f};
    rs.pb.vertices[2] = {VEC3(1000.0f,510.0f,0.1f),   VEC2(0.0f*div,1.0f*div), 0.0f};
    unsigned short i[3] = {0,1,2};
    rasterTA(i, 1, rs.pb.vertices);
#endif
}



