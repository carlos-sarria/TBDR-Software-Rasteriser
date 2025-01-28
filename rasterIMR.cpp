#include "raster.h"
#include "log.h"
#include <stdlib.h>

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

inline bool culling (VEC3 v0, VEC3 v1, VEC3 v2)
{
    // Backface culling: sign = ab*ac.y - ac.x*ab.y;
    if((v0.x-v1.x)*(v0.y-v2.y) - (v0.x-v2.x)*(v0.y-v1.y) >= 0.0f) return true;

    // Depth culling (front plane only) TODO: Front plane clipping
    if(v0.z<0.0f || v1.z<0.0f || v2.z<0.0f) return true;

    VEC2 a(0, 0);
    VEC2 b(rs.frameWidth, rs.frameHeight);
    return !testTriangle(v0, v1, v2, a, b);
}
#define FOCUS 0.01f
inline unsigned int blendPBR (unsigned int U, unsigned int V)
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

inline unsigned int blend (unsigned int mem_pos, unsigned int U, unsigned int V, float shade)
{
    unsigned int tex_color;
    unsigned int source_color;
    tex_color = (rs.material.baseColor.data==0) ? rs.material.color : rs.material.baseColor.data[U+V*rs.material.baseColor.width];

    if(rs.material.blend_mode==NONE && rs.material.smooth_shade==false) return tex_color;

    float red =   (float)(tex_color>>16&0xFF);
    float green = (float)(tex_color>>8&0xFF);
    float blue =  (float)(tex_color&0xFF);

    if(rs.material.smooth_shade)
    {
        shade = CLAMP(shade,0.0f,1.0f);
        red *= shade;
        green *= shade;
        blue *= shade;
    }

    if(rs.material.blend_mode!=NONE)
    {
        source_color = rs.colorBuffer[mem_pos];
        float s_red =   (float)(source_color>>16&0xFF);
        float s_green = (float)(source_color>>8&0xFF);
        float s_blue =  (float)(source_color&0xFF);

        if (rs.material.blend_mode==ADDITIVE)
        {
            red   += s_red;
            green += s_green;
            blue  += s_blue;

            red = CLAMP(red, 0.0f, 255.0);
            green = CLAMP(green, 0.0f, 255.0);
            blue = CLAMP(blue, 0.0f, 255.0);
        }

        if (rs.material.blend_mode==ALPHA)
        {
            float alpha = (float)(tex_color>>24&0xFF)*(rs.material.factor/255.0f);
            float inv_alpha = 1.0f-alpha;
            red   = alpha * red + inv_alpha * s_red;
            green = alpha * green + inv_alpha * s_green;
            blue  = alpha * blue + inv_alpha * s_blue;
        }
    }

    unsigned int res_color = 0xFF<<24|int(red)<<16|int(green)<<8|int(blue);

    return res_color;
}

// Barycentric coordinates. Cramer's rule to solve linear systems
inline void barycentric (VEC3 v0, VEC3 v1, VEC3 v2, VEC3 p, VEC3 &by)
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

void rasterizeIMR(unsigned short *indices, const unsigned int &numIndices, TR_VERTEX *meshTVB)
{
    for (int indexCount = 0; indexCount < numIndices;)
    {
        unsigned short in0 = indices[indexCount++];
        unsigned short in1 = indices[indexCount++];
        unsigned short in2 = indices[indexCount++];

        if (culling(meshTVB[in0].pos, meshTVB[in1].pos, meshTVB[in2].pos)) continue;

        if (meshTVB[in0].pos.y > meshTVB[in1].pos.y) SWAP(in0, in1);
        if (meshTVB[in0].pos.y > meshTVB[in2].pos.y) SWAP(in0, in2);
        if (meshTVB[in1].pos.y > meshTVB[in2].pos.y) SWAP(in1, in2);

        TR_VERTEX v0 = meshTVB[in0];
        TR_VERTEX v1 = meshTVB[in1];
        TR_VERTEX v2 = meshTVB[in2];

        int total_height = v2.pos.y - v0.pos.y;

        VEC3 br0 = v1.pos - v0.pos;
        VEC3 br1 = v2.pos - v0.pos;
        float d00 = DOT(br0, br0);
        float d01 = DOT(br0, br1);
        float d11 = DOT(br1, br1);
        float invD = 1.0 / (d00 * d11 - d01 * d01);

        for (int i = 0; i < total_height; i++)
        {
            int y = int(v0.pos.y)+i;

            if(y<0 || y>=rs.frameHeight) continue;

            int aa, bb;

            float alpha = (float)i / (float)total_height;
            aa = int(v0.pos.x + (v2.pos.x - v0.pos.x) * alpha);

            if(y > v1.pos.y || v1.pos.y == v0.pos.y) // bottom half of the triangle
            {
                float beta = (float)(y - v1.pos.y) / (v2.pos.y - v1.pos.y);
                bb = int(v1.pos.x + (v2.pos.x - v1.pos.x) * beta);
            }
            else
            {
                float beta = (float)(y - v0.pos.y) / (v1.pos.y - v0.pos.y);
                bb = int(v0.pos.x + (v1.pos.x - v0.pos.x) * beta);
            }

            if (aa > bb) SWAP(aa, bb);

            // We split the raster line in 32 pixels chucks to reduce visual artefacts
            int next = (int)truncf((bb-aa)/32.0f)+1;
            for (int inc=0; inc<next; inc++)
            {
                int a = aa + inc*32;
                int b = aa + (inc+1)*32;
                if(b>bb) b = bb;

                float d = 1.0f/float(b-a);

                VEC3 vA(a,y,0.0f), vB(b,y,0.0f), byA, byB;

                VEC3 br2 = vA - v0.pos;
                float d20 = DOT(br2, br0);
                float d21 = DOT(br2, br1);
                byA.y = (d11 * d20 - d01 * d21) * invD;
                byA.z = (d00 * d21 - d01 * d20) * invD;
                byA.x = 1.0f - byA.y - byA.z;

                br2 = vB - v0.pos;
                d20 = DOT(br2, br0);
                d21 = DOT(br2, br1);
                byB.y = (d11 * d20 - d01 * d21) * invD;
                byB.z = (d00 * d21 - d01 * d20) * invD;
                byB.x = 1.0f - byB.y - byB.z;

                VEC3 vDepth(v0.pos.z, v1.pos.z, v2.pos.z);
                float depth =  vDepth.dot(byA);
                float depthB =  vDepth.dot(byB);
                float depth_incr = (depthB-depth)*d;

                float invD = 1.0f/depth;
                float invDB = 1.0f/depthB;

                VEC3 vU(v0.uv.u, v1.uv.u, v2.uv.u);
                float texU = vU.dot(byA) * invD * (float)rs.material.baseColor.width;
                float texUB = vU.dot(byB) * invDB * (float)rs.material.baseColor.width;
                float texU_incr = (texUB-texU)*d;

                VEC3 vV(v0.uv.v, v1.uv.v, v2.uv.v);
                float texV = vV.dot(byA) * invD * (float)rs.material.baseColor.height;
                float texVB = vV.dot(byB) * invDB * (float)rs.material.baseColor.height;
                float texV_incr = (texVB-texV)*d;

                VEC3 vInt(v0.intensity, v1.intensity, v2.intensity);
                float shade = vInt.dot(byA) * invD;
                float shadeB = vInt.dot(byB) * invDB;
                float shade_incr = (shadeB-shade)*d;

                unsigned long incr = (a + y * rs.frameWidth);

                 for (int x = a; x < b; x++)
                {
                    if(x>=0 && x<rs.frameWidth)
                    {
                        if(depth>=rs.depthBuffer[incr]) // Depth comparison GREATER-EQUAL
                        {
                            // Update the depth buffer with the new value
                            rs.depthBuffer[incr] = depth;

                            // Draw the pixel on the screen buffer
                            //rs.colorBuffer[incr] = blendPBR((int)texU&(rs.material.baseColor.width-1), (int)texV&(rs.material.baseColor.height-1));
                            rs.colorBuffer[incr] = blend(incr, (unsigned int)texU&(rs.material.baseColor.width-1), (unsigned int)texV&(rs.material.baseColor.height-1), shade);
                        }
                    }

                    depth += depth_incr;
                    texU += texU_incr;
                    texV += texV_incr;
                    shade += shade_incr;
                    incr++;
                }
            }
        }
    }
}

void transform (VERTEX *v, const unsigned int &numVertices, TR_VERTEX *meshTVB, VEC3 center)
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

///////////////////////////////////
/// External API
///////////////////////////////////

void apiSendVertices (VERTEX *vertices, const unsigned int &numVertices, unsigned short *indices, const unsigned int &numIndices, VEC3 center)
{
    TR_VERTEX *meshTVB = (TR_VERTEX *)malloc(numVertices*sizeof(TR_VERTEX));

    transform (vertices, numVertices, meshTVB, center);
    rasterizeIMR (indices, numIndices, meshTVB);

    free(meshTVB);
}

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
}

void apiRelease()
{
    if(rs.depthBuffer) { free(rs.depthBuffer); rs.depthBuffer = nullptr;}
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

void apiStartRender()
{
}

void apiEndRender()
{
}

void apiSetMaterial(MATERIAL material)
{
    rs.material = material;
}
