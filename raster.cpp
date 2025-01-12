#include "raster.h"
#include <stdlib.h>

unsigned int debug_colors[] = {0xFFFF0000,0xFF00FF00,0xFF0000FF,0xFFFFFF00,0xFFFF00FF,0xFF00FFFF,0xFF800000,0xFF008000,0xFF000080};
unsigned int currColor = 0;
RENDER_STATE rs;

void rasterSetWorldMatrix(MATRIX m) { rs.worldMatrix = m; }

void rasterSetViewMatrix(MATRIX m) { rs.viewMatrix = m; }

void rasterSetProjectionMatrix(MATRIX m) { rs.projectionMatrix = m; }

void rasterSetLight(VEC3 pos) { rs.lightPosition = pos;};
void rasterSetEye(VEC3 pos) { rs.eyePosition = pos;};

void rasterInitialise(const int &width, const int &height, void *debugBuffer)
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

void rasterRelease()
{
    if(rs.depthBuffer) { free(rs.depthBuffer); rs.depthBuffer = nullptr;}
}

void rasterClear(unsigned int color, float depth)
{
    unsigned int i = rs.frameWidth*rs.frameHeight;
    while(i--)
    {
        rs.depthBuffer[i]= depth;
        rs.colorBuffer[i] = color;
    }
}

void rasterSetMaterial(MATERIAL material)
{
    rs.material = material;
}

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

inline bool rasterCulling (VEC3 v0, VEC3 v1, VEC3 v2)
{
    // Backface culling: sign = ab*ac.y - ac.x*ab.y;
    if((v0.x-v1.x)*(v0.y-v2.y) - (v0.x-v2.x)*(v0.y-v1.y) >= 0.0f) return true;

    // Depth culling (front plane only) TODO: Front plane clipping
    if(v0.z<0.0f && v1.z<0.0f && v2.z<0.0f) return true;

    // Bounding box check (out-of-the-screen)
    VEC2INT a, b;
    boundingBox(v0, v1, v2, a, b);
    VEC2INT sc(rs.frameWidth/2, rs.frameHeight/2);
    VEC2INT td((b.x-a.x)/2,(b.y-a.y)/2);
    VEC2INT tc((b.x+a.x)/2,(b.y+a.y)/2);

      if( abs(tc.x-sc.x)>(td.x+sc.x) &&
        abs(tc.y-sc.y)>(td.y+sc.y) ) return true;

    return false;
}

#define MAP(x,y,w,h) (((unsigned int)(x*w)&(w-1)) + ((unsigned int)(y*h)&(h-1)) *w );

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

    // Very faked specular
    float specular = (normal * rs.eyeInvPosition) * 0.5f + 0.5f;
    specular =  (specular < (1.0f-FOCUS)) ? 0.0f : (specular-(1.0f-FOCUS))*(1.0f/FOCUS)*255.0f;

    float shade = (normal * rs.lightInvPosition) * 0.5f + 0.5f;
    shade *= metal.x; // x: ambient occlusion

    float shininess = specular*metal.y*0.2f; // y: roughness, z: metallicity

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

void rasterRasterizeIMR(unsigned short *indices, const unsigned int &numIndices, TR_VERTEX *meshTVB)
{
    for (int indexCount = 0; indexCount < numIndices;)
    {
        unsigned short in0 = indices[indexCount++];
        unsigned short in1 = indices[indexCount++];
        unsigned short in2 = indices[indexCount++];

        if (rasterCulling(meshTVB[in0].pos, meshTVB[in1].pos, meshTVB[in2].pos)) continue;

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

            float alpha = 1.0f, beta = 1.0f;

            int a, b;

            if(total_height>0) alpha = (float)i / (float)total_height;
            a = int(v0.pos.x + (v2.pos.x - v0.pos.x) * alpha);

            if(y > v1.pos.y || v1.pos.y == v0.pos.y) // bottom half of the triangle
            {
                float h = v2.pos.y - v1.pos.y;
                if(h > 0.0f) beta = (float)(y - v1.pos.y) / h;
                b = int(v1.pos.x + (v2.pos.x - v1.pos.x) * beta);
            }
            else
            {
                float h = v1.pos.y - v0.pos.y;
                if(h > 0.0f) beta = (float)(i) / h;
                b = int(v0.pos.x + (v1.pos.x - v0.pos.x) * beta);
            }

            if (a > b) SWAP(a, b);

            float d = 1.0f/float(b-a);

            VEC3 vA(a,y,0.0f), vB(b,y,0.0f), byA, byB;

            // Some marginal increase in performance when splitting
            // the barycentric calculations in two halfs

            //barycentric(v0.pos, v1.pos, v2.pos, vA, byA);
            VEC3 br2 = vA - v0.pos;
            float d20 = DOT(br2, br0);
            float d21 = DOT(br2, br1);
            byA.y = (d11 * d20 - d01 * d21) * invD;
            byA.z = (d00 * d21 - d01 * d20) * invD;
            byA.x = 1.0f - byA.y - byA.z;

            //barycentric(v0.pos, v1.pos, v2.pos, vB, byB);
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
            float texU = vU.dot(byA) * invD * rs.material.baseColor.width;
            float texUB = vU.dot(byB) * invDB * rs.material.baseColor.width;
            float texU_incr = (texUB-texU)*d;

            VEC3 vV(v0.uv.v, v1.uv.v, v2.uv.v);
            float texV = vV.dot(byA) * invD * rs.material.baseColor.height;
            float texVB = vV.dot(byB) * invDB * rs.material.baseColor.height;
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
                        rs.colorBuffer[incr] = blendPBR((unsigned int)texU&(rs.material.baseColor.width-1), (unsigned int)texV&(rs.material.baseColor.height-1));
                        //rs.colorBuffer[incr] = blend(incr, (unsigned int)texU&(rs.material.baseColor.width-1), (unsigned int)texV&(rs.material.baseColor.height-1), shade);
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

void rasterTransform (VERTEX *v, const unsigned int &numVertices, TR_VERTEX *meshTVB, VEC3 center)
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
        meshTVB[i].pos.x = ceil(out.x * div * (rs.frameWidth/2) + rs.frameWidth/2); // ceil = pixel left-top convention
        meshTVB[i].pos.y = ceil(out.y * div * (rs.frameWidth/2) + rs.frameHeight/2);
        meshTVB[i].pos.z = div;

        meshTVB[i].uv.u = v[i].uv.u * div;
        meshTVB[i].uv.v = v[i].uv.v * div;

        meshTVB[i].intensity = shade * div;
    }
}

void rasterSendVertices (VERTEX *vertices, const unsigned int &numVertices, unsigned short *indices, const unsigned int &numIndices, VEC3 center)
{
    TR_VERTEX *meshTVB = (TR_VERTEX *)malloc(numVertices*sizeof(TR_VERTEX));

    rasterTransform (vertices, numVertices, meshTVB, center);
    rasterRasterizeIMR (indices, numIndices, meshTVB);

    free(meshTVB);
}
