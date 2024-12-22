#ifndef VKMATH_H
#define VKMATH_H

#include <math.h>
#include <string.h>
#include <assert.h>

const float PI = 3.14159265359f;

static const float fIdentity[16] = {
    1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f
};

class VEC2
{
public:
    union{
        float x;
        float u;
    };
    union{
        float y;
        float v;
    };

    VEC2() { x= 0.0f; y = 0.0f; }
    VEC2(float vx, float vy) { x=vx; y=vy; }
    VEC2(const VEC2 &inV2) { x= inV2.x; y = inV2.y; }

    VEC2 operator = (const VEC2& inV2) { x = inV2.x; y = inV2.y; return *this;}
    bool operator == (const VEC2& inV2) { return (x == inV2.x && y == inV2.y); }
    VEC2 operator + (const VEC2& inV2)  const { return VEC2(x+inV2.x, y+inV2.y); }
    VEC2 operator - (const VEC2& inV2)  const { return VEC2(x-inV2.x, y-inV2.y); }

    float lenght() { return sqrt(x*x+y*y); }
    VEC2  normalize() { float l = lenght(); if(l==0.0f) { x = x/l;  y=y/l; }; return *this;}

} ;

class VEC3
{
public:
    float x;
    float y;
    float z;

    VEC3() { x= 0.0f; y = 0.0f; z = 0.0f; }
    VEC3(float vx, float vy, float vz) { x=vx; y=vy; z=vz; }
    VEC3(const VEC3 &inV3) { x= inV3.x; y = inV3.y; z = inV3.z; }

    VEC3& operator = (const VEC3& inV3) { x = inV3.x; y = inV3.y; z = inV3.z; return *this;}
    bool operator == (const VEC3& inV3) { return (x == inV3.x && y == inV3.y && z == inV3.z); }
    const VEC3 operator + (const VEC3& inV3) const { return VEC3(x+inV3.x, y+inV3.y, z+inV3.z); }
    const VEC3 operator - (const VEC3& inV3) const { return VEC3(x-inV3.x, y-inV3.y, z-inV3.z); }
    const VEC3 operator * (float const& f) const { return VEC3(x*f, y*f, z*f);}

    float lenght() { return sqrt(x*x+y*y+z*z); }
    VEC3&  normalize() { float l = lenght(); if(l!=0.0f){ x = x/l;  y=y/l; z=z/l; }; return *this;}
    float dotProduct(const VEC3 &inV3) {return (x*inV3.x + y*inV3.y + z*inV3.z);}
    const VEC3  crossProduct(const VEC3 &inV3) { VEC3 vOut; vOut.x = y * inV3.z - z * inV3.y; vOut.y = z * inV3.x - x * inV3.z; vOut.z = x * inV3.y - y * inV3.x; return vOut; }
};

class QUATERNION
{
public:
    float x;
    float y;
    float z;
    float w;

    QUATERNION() { x= 0.0f; y = 0.0f; z = 0.0f; w = 1.0f;}
    QUATERNION(float vx, float vy, float vz, float vw) { x=vx; y=vy; z=vz; w=vw;}
    QUATERNION(const QUATERNION &inQ) { x= inQ.x; y = inQ.y; z = inQ.z; w = inQ.w; }

    void operator = (const QUATERNION &inQ) { x = inQ.x; y = inQ.y; z = inQ.z; w = inQ.w;}
    bool operator == (const QUATERNION &inQ) { return (x == inQ.x && y == inQ.y && z == inQ.z && w == inQ.w); }
    const QUATERNION operator + (const QUATERNION& inQ) const { return QUATERNION(x+inQ.x, y+inQ.y, z+inQ.z, w+inQ.w); }
    const QUATERNION operator - (const QUATERNION& inQ) const { return QUATERNION(x-inQ.x, y-inQ.y, z-inQ.z, w-inQ.w); }

    VEC3 toEuler()
    {
        VEC3 angle;

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        angle.x = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = std::sqrt(1 + 2 * (w * y - x * z));
        double cosp = std::sqrt(1 - 2 * (w * y - x * z));
        angle.y = 2 * std::atan2(sinp, cosp) - M_PI / 2;

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        angle.z = std::atan2(siny_cosp, cosy_cosp);

        return angle;
    }

    QUATERNION fromEuler(VEC3 euler)
    {
        double cy = cos(euler.z * 0.5);
        double sy = sin(euler.z * 0.5);
        double cr = cos(euler.x * 0.5);
        double sr = sin(euler.x * 0.5);
        double cp = cos(euler.y * 0.5);
        double sp = sin(euler.y * 0.5);

        w = cy * cr * cp + sy * sr * sp;
        x = cy * sr * cp - sy * cr * sp;
        y = cy * cr * sp + sy * sr * cp;
        z = sy * cr * cp - cy * sr * sp;

        return *this;
    }
};

class MATRIX
{
public:
    float* operator [] ( const int Row ) { return &f[Row*4]; }
    MATRIX operator*(MATRIX const& m) { multiply(m); return *this;}
    VEC3 operator*(VEC3 const& v) { return vectorMultiply(v);}

    MATRIX() { identity(); }

    float f[16];

public:

void identity()
{
    this->f[ 0]=1.0f;	this->f[ 4]=0.0f;	this->f[ 8]=0.0f;	this->f[12]=0.0f;
    this->f[ 1]=0.0f;	this->f[ 5]=1.0f;	this->f[ 9]=0.0f;	this->f[13]=0.0f;
    this->f[ 2]=0.0f;	this->f[ 6]=0.0f;	this->f[10]=1.0f;	this->f[14]=0.0f;
    this->f[ 3]=0.0f;	this->f[ 7]=0.0f;	this->f[11]=0.0f;	this->f[15]=1.0f;
};

void multiply(const MATRIX &m)
{
    MATRIX mOut;

    /* Perform calculation on a dummy  (mRet) */
    mOut.f[ 0] = this->f[ 0]*m.f[ 0] + this->f[ 1]*m.f[ 4] + this->f[ 2]*m.f[ 8] + this->f[ 3]*m.f[12];
    mOut.f[ 1] = this->f[ 0]*m.f[ 1] + this->f[ 1]*m.f[ 5] + this->f[ 2]*m.f[ 9] + this->f[ 3]*m.f[13];
    mOut.f[ 2] = this->f[ 0]*m.f[ 2] + this->f[ 1]*m.f[ 6] + this->f[ 2]*m.f[10] + this->f[ 3]*m.f[14];
    mOut.f[ 3] = this->f[ 0]*m.f[ 3] + this->f[ 1]*m.f[ 7] + this->f[ 2]*m.f[11] + this->f[ 3]*m.f[15];

    mOut.f[ 4] = this->f[ 4]*m.f[ 0] + this->f[ 5]*m.f[ 4] + this->f[ 6]*m.f[ 8] + this->f[ 7]*m.f[12];
    mOut.f[ 5] = this->f[ 4]*m.f[ 1] + this->f[ 5]*m.f[ 5] + this->f[ 6]*m.f[ 9] + this->f[ 7]*m.f[13];
    mOut.f[ 6] = this->f[ 4]*m.f[ 2] + this->f[ 5]*m.f[ 6] + this->f[ 6]*m.f[10] + this->f[ 7]*m.f[14];
    mOut.f[ 7] = this->f[ 4]*m.f[ 3] + this->f[ 5]*m.f[ 7] + this->f[ 6]*m.f[11] + this->f[ 7]*m.f[15];

    mOut.f[ 8] = this->f[ 8]*m.f[ 0] + this->f[ 9]*m.f[ 4] + this->f[10]*m.f[ 8] + this->f[11]*m.f[12];
    mOut.f[ 9] = this->f[ 8]*m.f[ 1] + this->f[ 9]*m.f[ 5] + this->f[10]*m.f[ 9] + this->f[11]*m.f[13];
    mOut.f[10] = this->f[ 8]*m.f[ 2] + this->f[ 9]*m.f[ 6] + this->f[10]*m.f[10] + this->f[11]*m.f[14];
    mOut.f[11] = this->f[ 8]*m.f[ 3] + this->f[ 9]*m.f[ 7] + this->f[10]*m.f[11] + this->f[11]*m.f[15];

    mOut.f[12] = this->f[12]*m.f[ 0] + this->f[13]*m.f[ 4] + this->f[14]*m.f[ 8] + this->f[15]*m.f[12];
    mOut.f[13] = this->f[12]*m.f[ 1] + this->f[13]*m.f[ 5] + this->f[14]*m.f[ 9] + this->f[15]*m.f[13];
    mOut.f[14] = this->f[12]*m.f[ 2] + this->f[13]*m.f[ 6] + this->f[14]*m.f[10] + this->f[15]*m.f[14];
    mOut.f[15] = this->f[12]*m.f[ 3] + this->f[13]*m.f[ 7] + this->f[14]*m.f[11] + this->f[15]*m.f[15];

    memcpy(this->f, mOut.f, sizeof(mOut.f));
};

QUATERNION quaternionMultiply(const QUATERNION &q)
{
    QUATERNION vRet;

    vRet.x = q.x*this->f[ 0] + q.y*this->f[ 4] + q.z*this->f[ 8] + q.w*this->f[12];
    vRet.y = q.x*this->f[ 1] + q.y*this->f[ 5] + q.z*this->f[ 9] + q.w*this->f[13];
    vRet.z = q.x*this->f[ 2] + q.y*this->f[ 6] + q.z*this->f[10] + q.w*this->f[14];
    vRet.w = q.x*this->f[ 3] + q.y*this->f[ 7] + q.z*this->f[11] + q.w*this->f[15];

    return vRet;
};

VEC3 vectorMultiply(const VEC3 &m)
{
    VEC3 vRet;

    vRet.x = m.x*this->f[ 0] + m.y*this->f[ 4] + m.z*this->f[ 8];
    vRet.y = m.x*this->f[ 1] + m.y*this->f[ 5] + m.z*this->f[ 9];
    vRet.z = m.x*this->f[ 2] + m.y*this->f[ 6] + m.z*this->f[10];

    return vRet;
};

void translation(const float fX, const float fY, const float fZ)
{
    MATRIX mTemp;

    mTemp.f[ 0]=1.0f;	mTemp.f[ 4]=0.0f;	mTemp.f[ 8]=0.0f;	mTemp.f[12]=fX;
    mTemp.f[ 1]=0.0f;	mTemp.f[ 5]=1.0f;	mTemp.f[ 9]=0.0f;	mTemp.f[13]=fY;
    mTemp.f[ 2]=0.0f;	mTemp.f[ 6]=0.0f;	mTemp.f[10]=1.0f;	mTemp.f[14]=fZ;
    mTemp.f[ 3]=0.0f;	mTemp.f[ 7]=0.0f;	mTemp.f[11]=0.0f;	mTemp.f[15]=1.0f;

    multiply(mTemp);
};

void scaling(const float fX, const float fY, const float fZ)
{
    MATRIX mTemp;

    mTemp.f[ 0]=fX;     mTemp.f[ 4]=0.0f;	mTemp.f[ 8]=0.0f;	mTemp.f[12]=0.0f;
    mTemp.f[ 1]=0.0f;	mTemp.f[ 5]=fY;     mTemp.f[ 9]=0.0f;	mTemp.f[13]=0.0f;
    mTemp.f[ 2]=0.0f;	mTemp.f[ 6]=0.0f;	mTemp.f[10]=fZ;     mTemp.f[14]=0.0f;
    mTemp.f[ 3]=0.0f;	mTemp.f[ 7]=0.0f;	mTemp.f[11]=0.0f;	mTemp.f[15]=1.0f;

    multiply(mTemp);
};

void rotationQ(QUATERNION &quaternion)
{
    MATRIX mTemp;

    // GLTF XYZW order (Blender is WXYZ)
    // Disable Y+ Up when exporting from Blender as Blender uses Z+ UP
    float qX = quaternion.x;
    float qY = quaternion.y;
    float qZ = quaternion.z;
    float qW = quaternion.w;

    mTemp.f[ 0] = (1 - (2 * qY * qY) - (2 * qZ * qZ));
    mTemp.f[ 4] = ((2 * qX * qY) - (2 * qZ * qW));
    mTemp.f[ 8] = ((2* qX * qZ) + (2 * qY * qW));
    mTemp.f[12] = 0.0f;

    mTemp.f[ 1] = ((2 * qX * qY) + (2 * qZ * qW));
    mTemp.f[ 5] = (1 - (2 * qX * qX) - (2 * qZ *qZ));
    mTemp.f[ 9] = ((2 * qY * qZ) - (2 * qX * qW));
    mTemp.f[13] = 0.0f;

    mTemp.f[ 2] = ((2 * qX * qZ) - (2 * qY * qW));
    mTemp.f[ 6] = ((2 * qY * qZ) + (2 * qX * qW));
    mTemp.f[10] = (1 - (2 * qX * qX) - (2 * qY * qY));
    mTemp.f[14] = 0.0f;

    mTemp.f[3] = 0.0f;
    mTemp.f[7] = 0.0f;
    mTemp.f[11] = 0.0f;
    mTemp.f[15] = 1.0f;

    multiply(mTemp);
};

void rotationX(const float fAngle)
{
    MATRIX mTemp;
    float fCosine, fSine;

    /* Precompute cos and sin */
#if defined(BUILD_DX)
    fCosine	= (float)cos(-fAngle);
    fSine	= (float)sin(-fAngle);
#else
    fCosine	= (float)cos(fAngle);
    fSine	= (float)sin(fAngle);
#endif

    /* Create the trigonometric  corresponding to X Rotation */
    mTemp.f[ 0]=1.0f;	mTemp.f[ 4]=0.0f;	mTemp.f[ 8]=0.0f;	mTemp.f[12]=0.0f;
    mTemp.f[ 1]=0.0f;	mTemp.f[ 5]=fCosine;mTemp.f[ 9]=fSine;	mTemp.f[13]=0.0f;
    mTemp.f[ 2]=0.0f;	mTemp.f[ 6]=-fSine;	mTemp.f[10]=fCosine;mTemp.f[14]=0.0f;
    mTemp.f[ 3]=0.0f;	mTemp.f[ 7]=0.0f;	mTemp.f[11]=0.0f;	mTemp.f[15]=1.0f;

    multiply(mTemp);
};

void rotationY(const float fAngle)
{
    MATRIX mTemp;
    float fCosine, fSine;

    /* Precompute cos and sin */
#if defined(BUILD_DX)
    fCosine	= (float)cos(-fAngle);
    fSine	= (float)sin(-fAngle);
#else
    fCosine	= (float)cos(fAngle);
    fSine	= (float)sin(fAngle);
#endif

    /* Create the trigonometric  corresponding to Y Rotation */
    mTemp.f[ 0]=fCosine;	mTemp.f[ 4]=0.0f;	mTemp.f[ 8]=-fSine;		mTemp.f[12]=0.0f;
    mTemp.f[ 1]=0.0f;		mTemp.f[ 5]=1.0f;	mTemp.f[ 9]=0.0f;		mTemp.f[13]=0.0f;
    mTemp.f[ 2]=fSine;		mTemp.f[ 6]=0.0f;	mTemp.f[10]=fCosine;	mTemp.f[14]=0.0f;
    mTemp.f[ 3]=0.0f;		mTemp.f[ 7]=0.0f;	mTemp.f[11]=0.0f;		mTemp.f[15]=1.0f;

    multiply(mTemp);
};

void rotationZ(const float fAngle)
{
    MATRIX mTemp;
    float fCosine, fSine;

    /* Precompute cos and sin */
#if defined(BUILD_DX)
    fCosine =	(float)cos(-fAngle);
    fSine =		(float)sin(-fAngle);
#else
    fCosine =	(float)cos(fAngle);
    fSine =		(float)sin(fAngle);
#endif

    /* Create the trigonometric  corresponding to Z Rotation */
    mTemp.f[ 0]=fCosine;	mTemp.f[ 4]=fSine;	mTemp.f[ 8]=0.0f;	mTemp.f[12]=0.0f;
    mTemp.f[ 1]=-fSine;		mTemp.f[ 5]=fCosine;mTemp.f[ 9]=0.0f;	mTemp.f[13]=0.0f;
    mTemp.f[ 2]=0.0f;		mTemp.f[ 6]=0.0f;	mTemp.f[10]=1.0f;	mTemp.f[14]=0.0f;
    mTemp.f[ 3]=0.0f;		mTemp.f[ 7]=0.0f;	mTemp.f[11]=0.0f;	mTemp.f[15]=1.0f;

    multiply(mTemp);
};

void transpose()
{
    MATRIX mOut;

    mOut.f[ 0]=this->f[ 0];	mOut.f[ 4]=this->f[ 1];	mOut.f[ 8]=this->f[ 2];	mOut.f[12]=this->f[ 3];
    mOut.f[ 1]=this->f[ 4];	mOut.f[ 5]=this->f[ 5];	mOut.f[ 9]=this->f[ 6];	mOut.f[13]=this->f[ 7];
    mOut.f[ 2]=this->f[ 8];	mOut.f[ 6]=this->f[ 9];	mOut.f[10]=this->f[10];	mOut.f[14]=this->f[11];
    mOut.f[ 3]=this->f[12];	mOut.f[ 7]=this->f[13];	mOut.f[11]=this->f[14];	mOut.f[15]=this->f[15];

    memcpy(this->f, mOut.f, sizeof(mOut.f));
};

void inverse()
{
    MATRIX mOut;
    double det_1;
    double pos, neg, temp;

    /* Calculate the determinant of submatrix A and determine if the
       the  is singular as limited by the double precision
       floating-point data representation. */
    pos = neg = 0.0;
    temp =  this->f[ 0] * this->f[ 5] * this->f[10];
    if (temp >= 0.0) pos += temp; else neg += temp;
    temp =  this->f[ 4] * this->f[ 9] * this->f[ 2];
    if (temp >= 0.0) pos += temp; else neg += temp;
    temp =  this->f[ 8] * this->f[ 1] * this->f[ 6];
    if (temp >= 0.0) pos += temp; else neg += temp;
    temp = -this->f[ 8] * this->f[ 5] * this->f[ 2];
    if (temp >= 0.0) pos += temp; else neg += temp;
    temp = -this->f[ 4] * this->f[ 1] * this->f[10];
    if (temp >= 0.0) pos += temp; else neg += temp;
    temp = -this->f[ 0] * this->f[ 9] * this->f[ 6];
    if (temp >= 0.0) pos += temp; else neg += temp;
    det_1 = pos + neg;

    /* Is the submatrix A singular? */
    if ((det_1 == 0.0) || (fabs(det_1 / (pos - neg)) < 1.0e-15))
    {
        return;
    }
    else
    {
        /* Calculate inverse(A) = adj(A) / det(A) */
        det_1 = 1.0 / det_1;
        mOut.f[ 0] =   ( this->f[ 5] * this->f[10] - this->f[ 9] * this->f[ 6] ) * (float)det_1;
        mOut.f[ 1] = - ( this->f[ 1] * this->f[10] - this->f[ 9] * this->f[ 2] ) * (float)det_1;
        mOut.f[ 2] =   ( this->f[ 1] * this->f[ 6] - this->f[ 5] * this->f[ 2] ) * (float)det_1;
        mOut.f[ 4] = - ( this->f[ 4] * this->f[10] - this->f[ 8] * this->f[ 6] ) * (float)det_1;
        mOut.f[ 5] =   ( this->f[ 0] * this->f[10] - this->f[ 8] * this->f[ 2] ) * (float)det_1;
        mOut.f[ 6] = - ( this->f[ 0] * this->f[ 6] - this->f[ 4] * this->f[ 2] ) * (float)det_1;
        mOut.f[ 8] =   ( this->f[ 4] * this->f[ 9] - this->f[ 8] * this->f[ 5] ) * (float)det_1;
        mOut.f[ 9] = - ( this->f[ 0] * this->f[ 9] - this->f[ 8] * this->f[ 1] ) * (float)det_1;
        mOut.f[10] =   ( this->f[ 0] * this->f[ 5] - this->f[ 4] * this->f[ 1] ) * (float)det_1;

        /* Calculate -C * inverse(A) */
        mOut.f[12] = - ( this->f[12] * mOut.f[ 0] + this->f[13] * mOut.f[ 4] + this->f[14] * mOut.f[ 8] );
        mOut.f[13] = - ( this->f[12] * mOut.f[ 1] + this->f[13] * mOut.f[ 5] + this->f[14] * mOut.f[ 9] );
        mOut.f[14] = - ( this->f[12] * mOut.f[ 2] + this->f[13] * mOut.f[ 6] + this->f[14] * mOut.f[10] );

        /* Fill in last row */
        mOut.f[ 3] = 0.0f;
        mOut.f[ 7] = 0.0f;
        mOut.f[11] = 0.0f;
        mOut.f[15] = 1.0f;
    }

    memcpy(this->f, mOut.f, sizeof(mOut.f));
};

void lookAtLH(const VEC3 &vEye, const VEC3 &vAt, const VEC3 &vUp)
{
    VEC3 f, s, u;
    MATRIX	mOut;

    f.x = vEye.x - vAt.x;
    f.y = vEye.y - vAt.y;
    f.z = vEye.z - vAt.z;

    f.normalize();
    s = f.crossProduct(vUp);
    s.normalize();
    u = s.crossProduct(f);
    u.normalize();

    mOut.f[ 0] = s.x;
    mOut.f[ 1] = u.x;
    mOut.f[ 2] = -f.x;
    mOut.f[ 3] = 0;

    mOut.f[ 4] = s.y;
    mOut.f[ 5] = u.y;
    mOut.f[ 6] = -f.y;
    mOut.f[ 7] = 0;

    mOut.f[ 8] = s.z;
    mOut.f[ 9] = u.z;
    mOut.f[10] = -f.z;
    mOut.f[11] = 0;

    mOut.f[12] = 0;
    mOut.f[13] = 0;
    mOut.f[14] = 0;
    mOut.f[15] = 1;

    translation(-vEye.x, -vEye.y, -vEye.z);
    multiply(mOut);
};

void lookAtRH(const VEC3 &vEye, const VEC3 &vAt, const VEC3 &vUp)
{
    VEC3 f, s, u;
    MATRIX	mOut, t, p;

    f.x = vAt.x - vEye.x;
    f.y = vAt.y - vEye.y;
    f.z = vAt.z - vEye.z;

    f.normalize();
    s = f.crossProduct(vUp);
    s.normalize();
    u = s.crossProduct(f);
    u.normalize();

    // Fix for when vectors are parallel
    if (s.x==0.0f && s.y==0.0f && s.z==0.0f){
            VEC3 newUp = {vUp.x+0.0000001f,vUp.y+0.0000001f,vUp.z+0.0000001f};
            s = f.crossProduct(newUp);
            s.normalize();
            u = s.crossProduct(f);
            u.normalize();
    }

    mOut.f[ 0] = s.x;
    mOut.f[ 1] = u.x;
    mOut.f[ 2] = -f.x;
    mOut.f[ 3] = 0;

    mOut.f[ 4] = s.y;
    mOut.f[ 5] = u.y;
    mOut.f[ 6] = -f.y;
    mOut.f[ 7] = 0;

    mOut.f[ 8] = s.z;
    mOut.f[ 9] = u.z;
    mOut.f[10] = -f.z;
    mOut.f[11] = 0;

    mOut.f[12] = 0;
    mOut.f[13] = 0;
    mOut.f[14] = 0;
    mOut.f[15] = 1;

    translation(-vEye.x, -vEye.y, -vEye.z);
    multiply(mOut);
};

void perspectiveFovLH(const float fFOVy, const float fAspect, const float fNear, const float fFar, const bool bRotate)
{
    MATRIX mOut;
    float f, n, fRealAspect;

    if (bRotate)
        fRealAspect = 1.0f / fAspect;
    else
        fRealAspect = fAspect;

    // cotangent(a) == 1.0f / tan(a);
    f = 1.0f / (float)tan(fFOVy * 0.5f);
    n = 1.0f / (fFar - fNear);

    mOut.f[ 0] = f / fRealAspect;
    mOut.f[ 1] = 0;
    mOut.f[ 2] = 0;
    mOut.f[ 3] = 0;

    mOut.f[ 4] = 0;
    mOut.f[ 5] = f;
    mOut.f[ 6] = 0;
    mOut.f[ 7] = 0;

    mOut.f[ 8] = 0;
    mOut.f[ 9] = 0;
    mOut.f[10] = fFar * n;
    mOut.f[11] = 1;

    mOut.f[12] = 0;
    mOut.f[13] = 0;
    mOut.f[14] = -fFar * fNear * n;
    mOut.f[15] = 0;

    multiply(mOut);
    if (bRotate) rotationZ(-90.0f*PI/180.0f);
};

void perspectiveFovRH(const float fFOVy, const float fAspect, const float fNear, const float fFar, const bool bRotate)
{
    MATRIX mOut;
    float f, n, fRealAspect;

    if (bRotate)
        fRealAspect = 1.0f / fAspect;
    else
        fRealAspect = fAspect;

    // cotangent(a) == 1.0f / tan(a);
    f = 1.0f / (float)tan(fFOVy * 0.5f);
    n = 1.0f / (fNear - fFar);

    mOut.f[ 0] = f / fRealAspect;
    mOut.f[ 1] = 0;
    mOut.f[ 2] = 0;
    mOut.f[ 3] = 0;

    mOut.f[ 4] = 0;
    mOut.f[ 5] = -f; // TODO: Why negative? Needed for Blender models
    mOut.f[ 6] = 0;
    mOut.f[ 7] = 0;

    mOut.f[ 8] = 0;
    mOut.f[ 9] = 0;
    mOut.f[10] = (fFar + fNear) * n;
    mOut.f[11] = -1;

    mOut.f[12] = 0;
    mOut.f[13] = 0;
    mOut.f[14] = (2 * fFar * fNear) * n;
    mOut.f[15] = 0;

    multiply(mOut);
    if (bRotate) rotationZ(-90.0f*PI/180.0f);
};

void orthoLH(const float w, const float h, const float zn, const float zf, const bool  bRotate)
{
    MATRIX mOut;

    mOut.f[ 0] = 2 / w;
    mOut.f[ 1] = 0;
    mOut.f[ 2] = 0;
    mOut.f[ 3] = 0;

    mOut.f[ 4] = 0;
    mOut.f[ 5] = 2 / h;
    mOut.f[ 6] = 0;
    mOut.f[ 7] = 0;

    mOut.f[ 8] = 0;
    mOut.f[ 9] = 0;
    mOut.f[10] = 1 / (zf - zn);
    mOut.f[11] = zn / (zn - zf);

    mOut.f[12] = 0;
    mOut.f[13] = 0;
    mOut.f[14] = 0;
    mOut.f[15] = 1;

    multiply(mOut);
    if (bRotate) rotationZ (-90.0f*PI/180.0f);
};

void orthoRH(const float w, const float h, const float zn, const float zf, const bool  bRotate)
{
    MATRIX mOut;

    mOut.f[ 0] = 2 / w;
    mOut.f[ 1] = 0;
    mOut.f[ 2] = 0;
    mOut.f[ 3] = 0;

    mOut.f[ 4] = 0;
    mOut.f[ 5] = 2 / h;
    mOut.f[ 6] = 0;
    mOut.f[ 7] = 0;

    mOut.f[ 8] = 0;
    mOut.f[ 9] = 0;
    mOut.f[10] = 1 / (zn - zf);
    mOut.f[11] = zn / (zn - zf);

    mOut.f[12] = 0;
    mOut.f[13] = 0;
    mOut.f[14] = 0;
    mOut.f[15] = 1;

    multiply(mOut);
    if (bRotate) rotationZ(-90.0f*PI/180.0f);
};

VEC3 lerp(const VEC3 &v1, const VEC3 &v2, const float s)
{
    VEC3 vOut;

    vOut.x = v1.x + s * (v2.x - v1.x);
    vOut.y = v1.y + s * (v2.y - v1.y);
    vOut.z = v1.z + s * (v2.z - v1.z);

    return vOut;
};

void linearEqSolve(
    float		* const pRes,
    float		** const pSrc,	// 2D array of floats. 4 Eq linear problem is 5x4 , constants in first column.
    const int	nCnt)
{
    int		i, j, k;
    float	f;

    if(nCnt == 1)
    {
        assert(pSrc[0][1] != 0);
        pRes[0] = pSrc[0][0] / pSrc[0][1];
        return;
    }

    // Loop backwards in an attempt avoid the need to swap rows
    i = nCnt;
    while(i)
    {
        --i;

        if(pSrc[i][nCnt] != 0)
        {
            // Row i can be used to zero the other rows; let's move it to the bottom
            if(i != (nCnt-1))
            {
                for(j = 0; j <= nCnt; ++j)
                {
                    // Swap the two values
                    f = pSrc[nCnt-1][j];
                    pSrc[nCnt-1][j] = pSrc[i][j];
                    pSrc[i][j] = f;
                }
            }

            // Now zero the last columns of the top rows
            for(j = 0; j < (nCnt-1); ++j)
            {
                assert(pSrc[nCnt-1][nCnt] != 0);
                f = pSrc[j][nCnt] / pSrc[nCnt-1][nCnt];

                // No need to actually calculate a zero for the final column
                for(k = 0; k < nCnt; ++k)
                {
                    pSrc[j][k] -= f * pSrc[nCnt-1][k];
                }
            }

            break;
        }
    }

    // Solve the top-left sub
    linearEqSolve(pRes, pSrc, nCnt - 1);

    // Now calc the solution for the bottom row
    f = pSrc[nCnt-1][0];
    for(k = 1; k < nCnt; ++k)
    {
        f -= pSrc[nCnt-1][k] * pRes[k-1];
    }
    assert(pSrc[nCnt-1][nCnt] != 0);
    f /= pSrc[nCnt-1][nCnt];
    pRes[nCnt-1] = f;
};

void inverseEx()
{
    MATRIX mOut;
    float *ppfRows[4];
    float pfRes[4];
    float pfIn[20];
    int   i, j;

    for(i = 0; i < 4; ++i)
        ppfRows[i] = &pfIn[i * 5];

    /* Solve 4 sets of 4 linear equations */
    for(i = 0; i < 4; ++i)
    {
        for(j = 0; j < 4; ++j)
        {
            ppfRows[j][0] = fIdentity[i + 4 * j];
            memcpy(&ppfRows[j][1], &this->f[j * 4], 4 * sizeof(float));
        }

        linearEqSolve(pfRes, (float**)ppfRows, 4);

        for(j = 0; j < 4; ++j)
        {
            mOut.f[i + 4 * j] = pfRes[j];
        }
    }

    memcpy(this->f, mOut.f, sizeof(mOut.f));
};

}; // End of MATRIX

#endif // VKMATH_H
