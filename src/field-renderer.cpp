#include "src/field-renderer.hpp"

#include <raylib.h>
#include <rlgl.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#ifdef PI
#undef PI
#endif

FieldRender::FieldRender(GLuint fbo, GLuint textureColorbuffer, int fboWidth, int fboHeight) : fbo(fbo),
                                                                                               textureColorbuffer(textureColorbuffer),
                                                                                               fboWidth(fboWidth),
                                                                                               fboHeight(fboHeight)
{
}

void FieldRender::render(const Values& values)
{
    RenderTexture2D target = {};
    target.id              = fbo;
    target.texture.id      = textureColorbuffer;
    target.texture.width   = fboWidth;
    target.texture.height  = fboHeight;

    BeginTextureMode(target);
    ClearBackground(BLACK);
    DrawCircle(fboWidth / 2, fboHeight / 2, 50, MAROON);
    DrawText("Drawing via high-level Raylib on custom FBO!", 10, 10, 20, RAYWHITE);
    EndTextureMode();
}