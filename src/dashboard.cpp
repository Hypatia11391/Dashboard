#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "stb_image.h"

#include <GLFW/glfw3.h>

#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>

#include <iostream>
#include <stdexcept>

#include "src/constants.hpp"
#include "src/dashboard.hpp"
#include "src/primitives.hpp"

Dashboard::Dashboard()
{
}

void Dashboard::run()
{
    init();
    mainLoop();
    cleanUp();
}

void Dashboard::init()
{
    inst       = nt::NetworkTableInstance::GetDefault();
    table      = inst.GetTable("datatable");
    xSub       = table->GetDoubleTopic("x").Subscribe(0.0);
    ySub       = table->GetDoubleTopic("y").Subscribe(0.0);
    dxSub      = table->GetDoubleTopic("dx").Subscribe(0.0);
    dySub      = table->GetDoubleTopic("dy").Subscribe(0.0);
    headingSub = table->GetDoubleTopic("heading").Subscribe(0.0);

    intakeLiftPosSub       = table->GetDoubleTopic("intake-lift-pos").Subscribe(0.0);
    intakeLiftTargetPosSub = table->GetDoubleTopic("intake-lift-target-pos").Subscribe(0.0);
    intakeFeedPowerSub     = table->GetDoubleTopic("intake-feed-power").Subscribe(0.0);

    feedPowerSub = table->GetDoubleTopic("feed-power").Subscribe(0.0);

    kickerPowerSub = table->GetDoubleTopic("kicker-power").Subscribe(0.0);

    shooterLeftRPMSub        = table->GetDoubleTopic("shooter-left-rpm").Subscribe(0.0);
    shooterLeftTargetRPMSub  = table->GetDoubleTopic("shooter-left-target-rpm").Subscribe(0.0);
    shooterRightRPMSub       = table->GetDoubleTopic("shooter-right-rpm").Subscribe(0.0);
    shooterRightTargetRPMSub = table->GetDoubleTopic("shooter-right-target-rpm").Subscribe(0.0);

    vSub = table->GetDoubleTopic("v").Subscribe(0.0);

    autoPub     = table->GetStringTopic("selected-auto").Publish();
    autoListSub = table->GetStringArrayTopic("auto-list").Subscribe({});

    inst.StartClient4("Dashboard Client");
    inst.SetServerTeam(11391);
    inst.StartDSClient();

    if (!glfwInit())
    {
        throw std::runtime_error("Failed to init glfw");
    }

    GLFWmonitor* monitor    = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);

    window = glfwCreateWindow(mode->width, mode->height, "Fullscreen Window", monitor, NULL);
    if (!window)
    {
        glfwTerminate();
        throw std::runtime_error("Failed to create window");
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    ImGui::StyleColorsDark();

    int my_image_width  = 0;
    int my_image_height = 0;
    fieldImg            = LoadTextureFromFile("field26.png", &my_image_width, &my_image_height);
}

void Dashboard::mainLoop()
{
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        float x       = xSub.Get();
        float y       = ySub.Get();
        float dx      = dxSub.Get();
        float dy      = dySub.Get();
        float heading = headingSub.Get();
        float v       = vSub.Get();

        float intakeLiftPos       = intakeLiftPosSub.Get();
        float intakeLiftTargetPos = intakeLiftTargetPosSub.Get();
        float intakeFeedPower     = intakeFeedPowerSub.Get();

        float feedPower = feedPowerSub.Get();

        float kickerPower = kickerPowerSub.Get();

        float shooterLeftRPM        = shooterLeftRPMSub.Get();
        float shooterLeftTargetRPM  = shooterLeftTargetRPMSub.Get();
        float shooterRightRPM       = shooterRightRPMSub.Get();
        float shooterRightTargetRPM = shooterRightTargetRPMSub.Get();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Connection status");
        ImGui::Text("NetworkTables Status: %s", inst.IsConnected() ? "Connected" : "Disconnected");
        ImGui::End();

        ImGui::Begin("Telemetry Data");

        ImGui::Text("X Position:        %.3f", x);
        ImGui::Text("Y Position:        %.3f", y);

        ImGui::Separator();

        ImGui::Text("DX Value:          %.3f", dx);
        ImGui::Text("DY Value:          %.3f", dy);

        ImGui::Separator();

        ImGui::Text("Heading Value:     %.3f", heading);

        ImDrawList* drawList = ImGui::GetWindowDrawList();

        ImVec2 topLeft = ImGui::GetCursorScreenPos();

        ImGui::Dummy({consts::fieldW * consts::fieldScaleFactor, consts::fieldH * consts::fieldScaleFactor + consts::robotH * consts::fieldScaleFactor});

        drawList->AddImage((void*) (intptr_t) fieldImg,
                           {topLeft.x, topLeft.y + consts::robotH * consts::fieldScaleFactor / 2},
                           {topLeft.x + consts::fieldW * consts::fieldScaleFactor,
                            topLeft.y + consts::fieldH * consts::fieldScaleFactor + consts::robotH * consts::fieldScaleFactor / 2},
                           {(float) 90 / 3508, (float) 90 / 1814},
                           {1 - (float) 90 / 3508, 1 - (float) 90 / 1814});

        // drawList->AddRectFilled({topLeft.x, topLeft.y + consts::robotH * consts::fieldScaleFactor / 2},
        //                         {topLeft.x + consts::fieldW * consts::fieldScaleFactor,
        //                          topLeft.y + consts::fieldH * consts::fieldScaleFactor + consts::robotH * consts::fieldScaleFactor / 2},
        //                         IM_COL32(25, 40, 60, 255));

        drawRotatedRect(drawList,
                        {topLeft.x + x * consts::fieldScaleFactor,
                         topLeft.y + consts::fieldH * consts::fieldScaleFactor - y * consts::fieldScaleFactor + consts::robotH * consts::fieldScaleFactor / 2},
                        {consts::robotH * consts::fieldScaleFactor, consts::robotW * consts::fieldScaleFactor},
                        heading,
                        IM_COL32(100, 125, 200, 255));

        ImGui::Separator();

        drawCompass(drawList, "Heading:", heading * consts::PI / 180, 200);

        ImGui::End();

        ImGui::Begin("Autos");

        std::vector<std::string> items = autoListSub.Get();
        static int current_item_idx    = 0;

        if (current_item_idx >= static_cast<int>(items.size()))
        {
            current_item_idx = 0;
        }

        std::string combo_preview_value;

        if (items.size())
        {
            combo_preview_value = items[current_item_idx];
        }
        else
        {
            combo_preview_value = "";
        }

        if (ImGui::BeginCombo("Autos", combo_preview_value.c_str()))
        {
            for (int n = 0; n < items.size(); n++)
            {
                const bool is_selected = (current_item_idx == n);

                if (ImGui::Selectable(items[n].c_str(), is_selected))
                {
                    current_item_idx = n;
                }

                if (is_selected)
                {
                    ImGui::SetItemDefaultFocus();
                    autoPub.Set(items[0]);
                }
            }
            ImGui::EndCombo();
        }

        ImGui::End();

        ImGui::Begin("Misc");

        ImGui::BeginDisabled(true);

        if (v < 12)
        {
            ImGui::PushStyleColor(ImGuiCol_SliderGrab, IM_COL32(255, 0, 0, 255));
        }
        else if (v < 12.4)
        {
            ImGui::PushStyleColor(ImGuiCol_SliderGrab, IM_COL32(255, 255, 0, 255));
        }
        else
        {
            ImGui::PushStyleColor(ImGuiCol_SliderGrab, IM_COL32(0, 255, 0, 255));
        }

        ImGui::SliderFloat("Voltage", &v, 0, 14, "%.3f");

        ImGui::PopStyleColor();

        ImGui::EndDisabled();

        ImGui::End();

        ImGui::Begin("Intake State");

        ImGui::Text("Intake Lift Pos:                 %.3f", intakeLiftPos);
        ImGui::Text("Intake Lift Target Pos:          %.3f", intakeLiftTargetPos);

        ImGui::Separator();

        ImGui::Text("Intake Feed Power:               %.3f", intakeFeedPower);

        ImGui::End();

        ImGui::Begin("Feed State");

        ImGui::Text("Feed Power: %.3f", feedPower);

        ImGui::End();

        ImGui::Begin("Kicker State");

        ImGui::Text("Kicker Power: %.3f", kickerPower);

        ImGui::End();

        ImGui::Begin("Shooter State");

        ImGui::Text("Shooter Left RPM:         %.3f", shooterLeftRPM);
        ImGui::Text("Shooter Left Target RPM:  %.3f", shooterLeftTargetRPM);
        ImGui::Text("Shooter Right RPM:        %.3f", shooterRightRPM);
        ImGui::Text("Shooter Right Target RPM: %.3f", shooterRightTargetRPM);

        ImGui::End();

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }
}

void Dashboard::cleanUp()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
}

GLuint Dashboard::LoadTextureFromFile(const char* filename, int* out_width, int* out_height)
{
    // 1. Load the image from disk
    int image_width  = 0;
    int image_height = 0;
    int channels     = 0;

    // We pass 4 to force RGBA channels, which OpenGL expects for simple transparency
    unsigned char* image_data = stbi_load(filename, &image_width, &image_height, &channels, 4);
    if (image_data == NULL)
    {
        throw std::runtime_error("Failed to load image");
        return 0; // Failed to load
    }

    // 2. Create the OpenGL Texture
    GLuint image_texture;
    glGenTextures(1, &image_texture);
    glBindTexture(GL_TEXTURE_2D, image_texture);

    // 3. Setup texture scaling parameters (Linear filtering looks smooth)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Setup wrapping parameters (Clamp to edge prevents artifacts at borders)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // 4. Upload the pixels to the GPU
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);

    // 5. Free the CPU memory since the GPU now has the data
    stbi_image_free(image_data);

    *out_width  = image_width;
    *out_height = image_height;

    return image_texture;
}