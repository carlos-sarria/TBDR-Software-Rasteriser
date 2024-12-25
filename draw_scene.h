#ifndef DRAW_SCENE_H
#define DRAW_SCENE_H

#include "load_gltf.h"
#include "api.h"

void updateCamera(char keyPressed, const bool mousePressed, long mousePointX, long mousePointY);
void initialise_app(const char* gltfFile, unsigned int width, unsigned int height);
void draw_frame ();
void free_all();

#endif // DRAW_SCENE_H
