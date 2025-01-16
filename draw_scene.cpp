#include "draw_scene.h"
#include "raster.h"
#include "load_gltf.h"
#include "log.h"
#include "bake.h"

unsigned int SCREEN_WIDTH = 0;
unsigned int SCREEN_HEIGHT = 0;

extern SCENE gltfScene;
CAMERA camera;
TEXTURE reflection;

unsigned int colors[] = {0xFFFF0000,0xFF00FF00,0xFF0000FF,0xFFFFFF00,0xFFFF00FF,0xFF00FFFF,0xFF800000,0xFF008000,0xFF000080};

#define ROT_SPEED (0.5f*PI/180.0f)
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
            camera = gltfScene.cameras[0];
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
        VEC3 cross = vLookAt.cross(VEC3(0.0f,0.0f,1.0f));
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

    for(MESH& mesh : gltfScene.meshes)
    {
        if(mesh.normalTexture>-1)
        {
            bakeNormals (gltfScene.textures[mesh.normalTexture], mesh.indexBuffer, mesh.indexCount, mesh.vertexBuffer);
        }
    }
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

    rasterClear(0xFF200000);

    unsigned int mesh_count = 0;
    for(MESH& mesh : gltfScene.meshes)
    {
        //if(mesh.indexCount<100) continue; // avoid floor box
        MATRIX mModel, mMVP;
        mModel.scaling(mesh.transform.scale.x, mesh.transform.scale.y, mesh.transform.scale.z);
        mModel.rotationQ(mesh.transform.rotation);
        mModel.translation(mesh.transform.translation.x, mesh.transform.translation.y, mesh.transform.translation.z);
        mModel.rotationZ(angle); mModel.rotationX(angle/2.0f);// FOR TESTING

        rasterSetWorldMatrix(mModel);
        rasterSetViewMatrix(mView);
        rasterSetProjectionMatrix(mProjection) ;

        rasterSetLight(gltfScene.lights[0].transform.translation);
        rasterSetEye(gltfScene.cameras[0].transform.translation);

        MATERIAL material;
        if(mesh.baseColorTexture!=-1) material.baseColor = gltfScene.textures[mesh.baseColorTexture];
        else material.baseColor.data = 0;
        if(mesh.metallicRoughnessTexture!=-1) material.metallicRoughness = gltfScene.textures[mesh.metallicRoughnessTexture];
        else material.metallicRoughness.data = 0;
        if(mesh.emissiveTexture!=-1) material.emissive = gltfScene.textures[mesh.emissiveTexture];
        else material.emissive.data = 0;
        if(mesh.normalTexture!=-1) material.normal = gltfScene.textures[mesh.normalTexture];
        else material.normal.data = 0;
        material.reflection = reflection;
        material.blend_mode = NONE;
        material.factor = 0.5f;
        material.color = colors[mesh_count&7];
        material.smooth_shade = true;
        rasterSetMaterial(material);

        rasterSendVertices (mesh.vertexBuffer, mesh.vertexCount, mesh.indexBuffer, mesh.indexCount, mesh.center);

        mesh_count++;
    }

    angle += 0.05f;
    numFrames++;

    if(clock()-start > 1000L) // 1 second
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
