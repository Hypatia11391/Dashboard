#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>

#include "src/dashboard.hpp"

int main(int argc, char* argv[])
{
    bool isSimulation = false;

    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--sim")
        {
            isSimulation = true;
            break;
        }
    }

    if (isSimulation)
    {
        std::cout << "Starting in Simulation Mode" << std::endl;
    }
    else
    {
        std::cout << "Starting in Driver Station Mode" << std::endl;
    }

    DashboardCreateInfo dashboardCreateInfo{};
    dashboardCreateInfo.isSimulation = isSimulation;

    Dashboard app = {dashboardCreateInfo};

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