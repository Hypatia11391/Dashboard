#pragma once

#include <include/glad/glad.h>

#include "src/networking-values.hpp"

class FieldRender
{
    public:
        FieldRender(GLuint fbo, GLuint textureColorbuffer, int fboWidth, int fboHeight);

        void render(const Values& values);

        void setFbo(GLuint newFbo, int newFboWidth, int newFboHeight);

    private:
        GLuint fbo, textureColorbuffer;
        int fboWidth, fboHeight;
};