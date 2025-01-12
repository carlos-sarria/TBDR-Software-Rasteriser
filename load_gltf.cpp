#include "load_gltf.h"
#include "dds-ktx.h"
#include "log.h"

SCENE gltfScene;

inline bool isPowerOfTwo(unsigned int x){return (x & (x - 1)) == 0;}

bool loadDDS(const char* textureFileName, TEXTURE& texture)
{
    FILE* textureFile;

    // opening the file in read mode
    textureFile = fopen(textureFileName, "rb");
    if(!textureFile)
    {
        apiLog("Cannot find %s",textureFileName);
        exit(1);
    }

    // File size
    fseek(textureFile, 0L, SEEK_END);
    unsigned int fileSize = ftell(textureFile);
    fseek(textureFile, 0L, SEEK_SET);

    // File data
    char* fileData = (char*)malloc(fileSize);
    fread(fileData, sizeof(char), fileSize, textureFile);

    ddsktx_texture_info tc = {0};

    if (ddsktx_parse(&tc, fileData, fileSize, NULL))
    {
        if (!isPowerOfTwo(tc.height) || !isPowerOfTwo(tc.width))
        {
            apiLog("Sorry only power of two textures % is (%d, %d)\n",textureFileName,tc.width,tc.height);
            return false;
        }
        if(tc.num_mips>0)
        {
            for (int mip = 0; mip < 1/*tc.num_mips*/; mip++)
            {
                ddsktx_sub_data sub_data;
                ddsktx_get_sub(&tc, &sub_data, fileData, fileSize, 0, 0, mip);
                texture.data = (unsigned int *)malloc(sub_data.size_bytes);
                memcpy(texture.data, sub_data.buff, sub_data.size_bytes);
                texture.width = sub_data.width;
                texture.height = sub_data.height;
            }
        }

        apiLog("Loaded %s (%d,%d)\n",textureFileName,tc.width,tc.height);
    }
    else
    {
        apiLog("Texture % not found\n",textureFileName);
    }

    free(fileData);
    fclose (textureFile);

    return true;
}

// Callback function required for tiny_gltf
static bool myTextureLoadingFunction(tinygltf::Image *image, const int image_idx, std::string * err,
                                     std::string * warn, int req_width, int req_height,
                                     const unsigned char * bytes, int size, void* user_data)
{
    // GLTF does not support DDS
    // The data received here is just the RGBA exported by Blender
    // we will need to use the texture name and work out the DDS file

TEXTURE texture;
#if 1
    std::string name = image->name.substr(0, image->name.rfind('.'));
    std::string uri = gltfScene.path+"\\"+name+".dds";
    loadDDS(uri.c_str(), texture);
#else  // PNG Format
    texture.data = (unsigned int *)malloc(size);//bytes;
    memcpy(texture.data, bytes, size);
    texture.height = image->height;
    texture.width = image->width;
#endif
    gltfScene.textures.push_back(texture);
    return true;
}

static void getTransform(TRANSFORM& transform, const tinygltf::Node node)
{
    int size = node.translation.size();
    transform.translation.x = (size>0) ? node.translation[0]: 0.0f;
    transform.translation.y = (size>0) ? node.translation[1]: 0.0f;
    transform.translation.z = (size>0) ? node.translation[2]: 0.0f;

    size = node.rotation.size();
    transform.rotation.x = (size>0) ? node.rotation[0]: 0.0f;
    transform.rotation.y = (size>0) ? node.rotation[1]: 0.0f;
    transform.rotation.z = (size>0) ? node.rotation[2]: 0.0f;
    transform.rotation.w = (size>0) ? node.rotation[3]: 1.0f;

    size = node.scale.size();
    transform.scale.x = (size>0) ? node.scale[0] : 1.0f;
    transform.scale.y = (size>0) ? node.scale[1] : 1.0f;
    transform.scale.z = (size>0) ? node.scale[2] : 1.0f;
}


/// <summary>Defines the vertices of a simple triangle which can be passed to the vertex shader to be rendered on screen</summary>
void load_gltf(const char* fileName)
{
    // Load GLTF
    tinygltf::Model model;
    tinygltf::TinyGLTF gltf_ctx;
    std::string err;
    std::string warn;
    unsigned int textureID = 0;

    std::string fn(fileName);
    gltfScene.path = fn.substr(0, fn.rfind('\\'));

    gltf_ctx.SetImageLoader(myTextureLoadingFunction, (void *)&gltfScene);

    gltf_ctx.SetStoreOriginalJSONForExtrasAndExtensions(false);

    bool ret = false;
    ret = gltf_ctx.LoadBinaryFromFile(&model, &err, &warn, fileName);

    if(!ret){
        apiLog("Cannot find %s",fileName);
        exit(1);
    }

    const tinygltf::Scene &scene = model.scenes[model.defaultScene];
    int numCameras = 0;
    int numLights = 0;
    for(tinygltf::Node &node : model.nodes)
    {
        if (node.camera > -1)
        {
            CAMERA camera;

            getTransform(camera.transform, node);
            camera.type = 0;
            camera.aspectRatio = model.cameras[numCameras].perspective.aspectRatio;
            camera.yfov = model.cameras[numCameras].perspective.yfov;
            camera.zfar = model.cameras[numCameras].perspective.zfar;
            camera.znear = model.cameras[numCameras].perspective.znear;

            gltfScene.cameras.push_back(camera);
            numCameras++;

        }

        if (node.light > -1)
        {
            LIGHT light;

            getTransform(light.transform, node);
            light.type = 0;

            gltfScene.lights.push_back(light);
        }

        if (node.mesh > -1)
        {
            tinygltf::Mesh mesh = model.meshes[node.mesh];
            unsigned short* bufferIndices;
            unsigned int numIndices = 0;
            unsigned int numVertices = 0;

            for (tinygltf::Primitive primitive : mesh.primitives)
            {
                MESH temp_mesh;

                const tinygltf::Accessor accessor_indices = model.accessors[primitive.indices];
                const tinygltf::BufferView bufferView_indices = model.bufferViews[accessor_indices.bufferView];
                bufferIndices = reinterpret_cast<unsigned short*>(&model.buffers[bufferView_indices.buffer].data[0] + bufferView_indices.byteOffset);

                numIndices = accessor_indices.count;
                temp_mesh.vertexBuffer = (VERTEX *)malloc(sizeof(VERTEX)*numIndices);

                const tinygltf::Accessor accessor_pos = model.accessors[primitive.attributes["POSITION"]];
                const tinygltf::Accessor accessor_nor = model.accessors[primitive.attributes["NORMAL"]];
                const tinygltf::Accessor accessor_tex = model.accessors[primitive.attributes["TEXCOORD_0"]];

                const tinygltf::BufferView bufferView_pos = model.bufferViews[accessor_pos.bufferView];
                const tinygltf::BufferView bufferView_nor = model.bufferViews[accessor_nor.bufferView];
                const tinygltf::BufferView bufferView_tex = model.bufferViews[accessor_tex.bufferView];

                const float* buffer_pos = reinterpret_cast<const float*>(&model.buffers[bufferView_pos.buffer].data[0] + bufferView_pos.byteOffset);
                const float* buffer_nor = reinterpret_cast<const float*>(&model.buffers[bufferView_nor.buffer].data[0] + bufferView_nor.byteOffset);
                const float* buffer_tex = reinterpret_cast<const float*>(&model.buffers[bufferView_tex.buffer].data[0] + bufferView_tex.byteOffset);

                numVertices = accessor_pos.count;

                temp_mesh.boundingBoxA = VEC3(buffer_pos[0],buffer_pos[1],buffer_pos[2]);
                temp_mesh.boundingBoxB = temp_mesh.boundingBoxA;

                //for (unsigned int i=0; i<accessor_indices.count; i++)
                for (unsigned int i=0; i<numVertices; i++)
                {
                    float x,y,z;

                    temp_mesh.vertexBuffer[i].pos.x = x = buffer_pos[i * 3 + 0]; // VEC3
                    temp_mesh.vertexBuffer[i].pos.y = y = buffer_pos[i * 3 + 1];
                    temp_mesh.vertexBuffer[i].pos.z = z =buffer_pos[i * 3 + 2];
                    temp_mesh.vertexBuffer[i].nor.x = buffer_nor[i * 3 + 0]; // VEC3
                    temp_mesh.vertexBuffer[i].nor.y = buffer_nor[i * 3 + 1];
                    temp_mesh.vertexBuffer[i].nor.z = buffer_nor[i * 3 + 2];
                    temp_mesh.vertexBuffer[i].uv.u = buffer_tex[i * 2 + 0]; // VEC2
                    temp_mesh.vertexBuffer[i].uv.v = buffer_tex[i * 2 + 1];

                    // Bonding box
                    if(x<temp_mesh.boundingBoxA.x) temp_mesh.boundingBoxA.x=x;
                    if(y<temp_mesh.boundingBoxA.y) temp_mesh.boundingBoxA.y=y;
                    if(z<temp_mesh.boundingBoxA.z) temp_mesh.boundingBoxA.z=z;
                    if(x>temp_mesh.boundingBoxB.x) temp_mesh.boundingBoxB.x=x;
                    if(y>temp_mesh.boundingBoxB.y) temp_mesh.boundingBoxB.y=y;
                    if(z>temp_mesh.boundingBoxB.z) temp_mesh.boundingBoxB.z=z;
                }

                temp_mesh.center = (temp_mesh.boundingBoxA + temp_mesh.boundingBoxB) * 0.5f;

                if(primitive.material != -1)
                {
                    temp_mesh.baseColorTexture = model.materials[primitive.material].pbrMetallicRoughness.baseColorTexture.index;
                    temp_mesh.metallicRoughnessTexture = model.materials[primitive.material].pbrMetallicRoughness.metallicRoughnessTexture.index;
                    temp_mesh.emissiveTexture = model.materials[primitive.material].emissiveTexture.index;
                    temp_mesh.normalTexture = model.materials[primitive.material].normalTexture.index;
                }

                temp_mesh.indexBuffer = (unsigned short*)malloc(sizeof(unsigned short)*numIndices);
                memcpy(temp_mesh.indexBuffer, bufferIndices, sizeof(unsigned short)*numIndices);

                getTransform(temp_mesh.transform, node);

                temp_mesh.indexCount = numIndices;
                temp_mesh.vertexCount = numVertices;
                temp_mesh.vertexBuffer = temp_mesh.vertexBuffer;

                // Calculate the bounding box
                VEC3 center;

                gltfScene.meshes.push_back(temp_mesh);

            }
        }
    }

    if (gltfScene.cameras.size()==0)
    {
        CAMERA camera;

        camera.transform.rotation = QUATERNION(0.7f,0.0f,0.0f,0.7f);
        camera.transform.translation = VEC3(0.0f,-30.0f,0.0f);
        camera.from = camera.transform.translation;
        camera.to = VEC3(0.0f, 0.0f, 0.0f);
        camera.yfov = 0.39959f;
        camera.zfar = 5000.0f;
        camera.znear = 0.01f;

        gltfScene.cameras.push_back(camera);
    }

    if (gltfScene.lights.size()==0)
    {
        LIGHT light;

        light.type = 0;
        light.transform.rotation = QUATERNION(0.57f,0.17f,0.27f,0.75f);
        light.transform.translation = VEC3(4.0f,1.0f,6.0f);
        light.transform.scale = VEC3(1.0f,1.0f,1.0f);

        gltfScene.lights.push_back(light);
    }
}

void free_gltf()
{
    for(MESH &mesh : gltfScene.meshes)
    {
        free(mesh.indexBuffer);
        free(mesh.vertexBuffer);
    }
}
