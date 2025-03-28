#ifndef DRAW_SCENE_H
#define DRAW_SCENE_H

void updateCamera(char keyPressed, const bool mousePressed, long mousePointX, long mousePointY);
void initialise_app(const char* gltfFile, unsigned int width, unsigned int height, void *frameBuffer);
void draw_frame ();
void free_all();

#endif // DRAW_SCENE_H
