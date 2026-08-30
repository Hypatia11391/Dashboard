#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>

#include "src/dashboard.hpp"

int main()
{
    Dashboard app;

    try
    {
        app.run();
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }

    return 0;
}