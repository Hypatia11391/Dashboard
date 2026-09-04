#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "stb_image.h"

#include <GLFW/glfw3.h>

#include <frc/geometry/Pose2d.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>

#include <iostream>
#include <stdexcept>

#include "src/constants.hpp"
#include "src/dashboard.hpp"
#include "src/primitives.hpp"
#include "units/angle.h"

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
    inst  = nt::NetworkTableInstance::GetDefault();
    table = inst.GetTable("datatable");

    fieldTable       = table->GetSubTable("field");
    robotPoseSub     = fieldTable->GetDoubleArrayTopic("robot").Subscribe({});
    robotVelocitySub = fieldTable->GetDoubleArrayTopic("velocity").Subscribe({});
    trajPoseSub      = fieldTable->GetDoubleArrayTopic("trajectory").Subscribe({});

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

    frontLeftSub  = table->GetDoubleTopic("front-left").Subscribe(0);
    frontRightSub = table->GetDoubleTopic("front-right").Subscribe(0);
    rearLeftSub   = table->GetDoubleTopic("rear-left").Subscribe(0);
    rearRightSub  = table->GetDoubleTopic("rear-right").Subscribe(0);

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

        float v = vSub.Get();

        auto poseData = robotPoseSub.Get();
        frc::Pose2d pose;

        if (poseData.size() >= 3)
        {
            pose = {units::meter_t(poseData[0]),
                    units::meter_t(poseData[1]),
                    {units::degree_t(poseData[2])}};
        }
        else
        {
            pose = {};
        }

        auto velocityData = robotVelocitySub.Get();
        frc::Pose2d velocity;

        if (velocityData.size() >= 3)
        {
            velocity = {units::meter_t(velocityData[0]),
                        units::meter_t(velocityData[1]),
                        {units::degree_t(velocityData[2])}};
        }
        else
        {
            velocity = {};
        }

        auto trajData = trajPoseSub.Get();
        std::vector<frc::Pose2d> trajPoints;

        if (!trajData.empty() && trajData.size() % 3 == 0)
        {
            trajPoints.resize(trajData.size() / 3);

            for (int i = 0; i * 3 < trajData.size(); i++)
            {
                trajPoints[i] = {units::meter_t(trajData[i * 3]),
                                 units::meter_t(trajData[i * 3 + 1]),
                                 {units::degree_t(trajData[i * 3 + 2])}};
            }
        }

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

        ImGui::Text("X Position:    %.3f", (float) pose.X());
        ImGui::Text("Y Position:    %.3f", (float) pose.Y());

        ImGui::Separator();

        ImGui::Text("DX Value:      %.3f", (float) velocity.X());
        ImGui::Text("DY Value:      %.3f", (float) velocity.Y());
        ImGui::Text("Omega Value:   %.3f", (float) velocity.Rotation().Radians());

        ImGui::Separator();

        ImGui::Text("Heading Value: %.3f", (float) pose.Rotation().Degrees());

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

        // drawRotatedRect(drawList,
        //                 {topLeft.x + x * consts::fieldScaleFactor,
        //                  topLeft.y + consts::fieldH * consts::fieldScaleFactor - y * consts::fieldScaleFactor + consts::robotH * consts::fieldScaleFactor / 2},
        //                 {consts::robotH * consts::fieldScaleFactor, consts::robotW * consts::fieldScaleFactor},
        //                 heading,
        //                 IM_COL32(100, 125, 200, 255));

        drawRotatedRect(drawList,
                        {topLeft.x + (float) pose.X() * consts::fieldScaleFactor,
                         topLeft.y + consts::fieldH * consts::fieldScaleFactor - (float) pose.Y() * consts::fieldScaleFactor + consts::robotH * consts::fieldScaleFactor / 2},
                        {consts::robotH * consts::fieldScaleFactor, consts::robotW * consts::fieldScaleFactor},
                        -(float) pose.Rotation().Degrees(),
                        IM_COL32(100, 125, 200, 255));

        std::vector<ImVec2> trajPointsToDraw;
        trajPointsToDraw.resize(trajPoints.size());

        for (int i = 0; i < trajPoints.size(); i++)
        {
            trajPointsToDraw[i] = {topLeft.x + (float) trajPoints[i].X() * consts::fieldScaleFactor,
                                   topLeft.y + consts::fieldH * consts::fieldScaleFactor - (float) trajPoints[i].Y() * consts::fieldScaleFactor};
        }

        drawList->AddPolyline(
            trajPointsToDraw.data(),
            trajPointsToDraw.size(),
            IM_COL32(0, 0, 255, 255),
            ImDrawFlags_None,
            2.0f);

        ImGui::Separator();

        drawCompass(drawList, "Heading:", -(float) pose.Rotation().Degrees() * consts::PI / 180, 200);

        ImGui::End();

        ImGui::Begin("Autos");

        std::vector<std::string> items = autoListSub.Get();

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
                    autoPub.Set(items[n]);
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

        std::cout
            << frontLeftSub.Get()
            << "\n"
            << frontRightSub.Get()
            << "\n"
            << rearLeftSub.Get()
            << "\n"
            << rearRightSub.Get()
            << "\n"
            << std::endl;

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