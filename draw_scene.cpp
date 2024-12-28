#include "draw_scene.h"
#include "raster.h"
#include "load_gltf.h"
#include "log.h"

unsigned int SCREEN_WIDTH = 0;
unsigned int SCREEN_HEIGHT = 0;

extern SCENE gltfScene;
CAMERA camera;

void initializeCamera()
{
    // Get the camera (the first one)
    if(gltfScene.cameras.size()>0)
    {
        camera.transform = gltfScene.cameras[0].transform;
        camera.from = gltfScene.cameras[0].transform.translation;
        camera.to = VEC3(0.0f, 0.0f, 0.0f);
        camera.yfov = gltfScene.cameras[0].yfov;
        camera.zfar = gltfScene.cameras[0].zfar;
        camera.znear = gltfScene.cameras[0].znear;
    }
    else
    {
        camera.transform.rotation = QUATERNION(0.7f,0.0f,0.0f,0.7f);
        camera.transform.translation = VEC3(0.0f,-30.0f,0.0f);
        camera.from = camera.transform.translation;
        camera.to = VEC3(0.0f, 0.0f, 0.0f);
        camera.yfov = 0.39959f;
        camera.zfar = 5000.0f;
        camera.znear = 0.01f;
    }
}

#define ROT_SPEED (0.05f*PI/180.0f)
#define MOV_SPEED 0.3f
void updateCamera(char keyPressed, const bool mousePressed, long mousePointX, long mousePointY)
{
    static VEC3 cameraPosition;
    static long mousePrevX, mousePrevY;
    static bool bFirstTime = true;
    static QUATERNION cameraRotation;

    if (bFirstTime || !mousePressed){
        mousePrevX = mousePointX;
        mousePrevY = mousePointY;
        if(bFirstTime) {
            initializeCamera();
            cameraPosition = camera.transform.translation;
            cameraRotation = camera.transform.rotation;
        }
        bFirstTime = false;
    }

    // Compose mouse movement with camera rotation
    VEC3 angle = cameraRotation.toEuler();
    angle.z += (float)(mousePointX-mousePrevX)*ROT_SPEED;
    angle.x += (float)(mousePointY-mousePrevY)*ROT_SPEED;
    cameraRotation = QUATERNION().fromEuler(angle);

    // Rotate the Blender LookAt vector (0,0,-1) using the Quaternion
    // up = cam.matrix_world.to_quaternion() * Vector((0.0, 1.0, 0.0))
    // cam_direction = cam.matrix_world.to_quaternion() * Vector((0.0, 0.0, -1.0))
    MATRIX mLookAt;
    mLookAt.rotationQ(cameraRotation);
    VEC3 vLookAt = mLookAt * VEC3(0.0f,0.0f,-1.0);

    // Move camera with keyboard
    float zoom = 0.0f, pan = 0.0f;
    if(keyPressed == 'W') zoom = -MOV_SPEED;
    if(keyPressed == 'S') zoom =  MOV_SPEED;
    if(keyPressed == 'A') pan  = -MOV_SPEED;
    if(keyPressed == 'D') pan  =  MOV_SPEED;
    if(zoom!=0.0f)
    {
        cameraPosition = cameraPosition - vLookAt * zoom;
    }
    if(pan!=0.0f)
    {
        VEC3 cross = vLookAt.crossProduct(VEC3(0.0f,0.0f,1.0f));
        cameraPosition = cameraPosition + cross * pan;
    }

    mousePrevX = mousePointX;
    mousePrevY = mousePointY;

    camera.from = cameraPosition;
    camera.to = cameraPosition + vLookAt;
}

void initialise_app(const char* gltfFile, unsigned int frameWidth, unsigned int frameHeight, void *frameBuffer)
{
    load_gltf(gltfFile);

    SCREEN_HEIGHT = frameHeight;
    SCREEN_WIDTH = frameWidth;

    rasterInitialise(frameWidth, frameHeight, frameBuffer);
}
void draw_frame ()
{
    static float angle = 0.0f;
    static int numFrames = 0;
    static clock_t start = clock();
    MATRIX mView, mProjection;
    mView.lookAtRH(camera.from, camera.to, VEC3(0.0f,0.0f,1.0f));

    float aspectRatio = SCREEN_WIDTH / SCREEN_HEIGHT;
    bool isRotated = (SCREEN_WIDTH < SCREEN_HEIGHT);

    mProjection.perspectiveFovRH(camera.yfov, aspectRatio, camera.znear, camera.zfar, isRotated);

    rasterClear();

    for(MESH& mesh : gltfScene.meshes)
    {
        MATRIX mModel, mMVP;
        mModel.scaling(mesh.transform.scale.x, mesh.transform.scale.y, mesh.transform.scale.z);
        mModel.rotationQ(mesh.transform.rotation);
        mModel.translation(mesh.transform.translation.x, mesh.transform.translation.y, mesh.transform.translation.z);
        //mModel.rotationZ(angle); // mModel.rotationX(angle/2.0f);// FOR TESTING

        rasterSetWorldMatrix(mModel);
        rasterSetViewMatrix(mView);
        rasterSetProjectionMatrix(mProjection) ;

        rasterSetLight(gltfScene.lights[0].transform.translation);

        rasterSetMaterial(gltfScene.textures[mesh.textureID]);

        rasterSendVertices (mesh.vertexBuffer, mesh.vertexCount, mesh.indexBuffer, mesh.indexCount);
    }

    angle += 0.01f;
    numFrames++;

    if(clock()-start > 1000L) // 1 seconds
    {
        apiLog(" %d FPS", numFrames);
        numFrames = 0;
        start = clock();
    }
}

void free_all()
{
    free_gltf();
    rasterRelease();
}
