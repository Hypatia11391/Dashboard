#include <algorithm>
#include <chrono>
#include <cstdio>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#include "networktables/Topic.h"
#include "src/networking-server.hpp"
#include "src/recording-manager.hpp"
#include <stb_image.h>

#include <GLFW/glfw3.h>

#include <frc/geometry/Pose2d.h>
#include <iterator>
#include <memory>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/GenericEntry.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <vendor/json.hpp>

using json = nlohmann::json;

#include "src/constants.hpp"
#include "src/dashboard.hpp"
#include "src/primitives.hpp"

Dashboard::Dashboard(const DashboardCreateInfo& createInfo) : createInfo(createInfo)
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
    NetworkingServerCreateInfo networkServerInfo{};
    networkServerInfo.isSimulation = createInfo.isSimulation;

    networkingServer = std::make_shared<NetworkingServer>(networkServerInfo);

    recordingManager = std::make_shared<RecordingManager>("output.jsonl");

    if (!glfwInit())
    {
        throw std::runtime_error("Failed to init glfw");
    }

    GLFWmonitor* monitor    = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);           // Required on macOS

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
    ImPlot::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 150 core");

    ImGui::StyleColorsDark();

    int my_image_width  = 0;
    int my_image_height = 0;
    fieldImg            = LoadTextureFromFile("field26.png", &my_image_width, &my_image_height);

    autoPlotData = {};

    auto currentTime = std::chrono::high_resolution_clock::now().time_since_epoch();

    startTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime);
}

void Dashboard::mainLoop()
{
    double kp{}, ki{}, kd{};
    std::string selectedAuto{};

    json outputArray = json::array();

    std::vector<int> customSubs{};
    std::vector<int> customFieldSubs{};
    std::vector<int> customRecordingSubs{};

    std::vector<CustomPlotData> customPlots{};

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        Values values = networkingServer->getValues();

        frc::Pose2d pose;

        if (values.robotPose.size() >= 3)
        {
            pose = {units::meter_t(values.robotPose[0]),
                    units::meter_t(values.robotPose[1]),
                    {units::degree_t(values.robotPose[2])}};
        }
        else
        {
            pose = {};
        }

        frc::Pose2d velocity;

        if (values.robotVelocity.size() >= 3)
        {
            velocity = {units::meter_t(values.robotVelocity[0]),
                        units::meter_t(values.robotVelocity[1]),
                        {units::degree_t(values.robotVelocity[2])}};
        }
        else
        {
            velocity = {units::meter_t(0),
                        units::meter_t(0),
                        {units::degree_t(0)}};
        }

        frc::Pose2d targetVelocity;

        if (values.targetVelocity.size() >= 3)
        {

            targetVelocity = {units::meter_t(values.targetVelocity[0]),
                              units::meter_t(values.targetVelocity[1]),
                              {units::degree_t(values.targetVelocity[2])}};
        }
        else
        {
            targetVelocity = {units::meter_t(0),
                              units::meter_t(0),
                              {units::degree_t(0)}};
        }

        std::vector<frc::Pose2d> trajPoints;

        if (!values.trajPose.empty() && values.trajPose.size() % 3 == 0)
        {
            trajPoints.resize(values.trajPose.size() / 3);

            for (int i = 0; i * 3 < values.trajPose.size(); i++)
            {
                trajPoints[i] = {units::meter_t(values.trajPose[i * 3]),
                                 units::meter_t(values.trajPose[i * 3 + 1]),
                                 {units::degree_t(values.trajPose[i * 3 + 2])}};
            }
        }

        SetValues setValues{};
        setValues.selectedAuto = selectedAuto;
        setValues.kp           = kp;
        setValues.ki           = ki;
        setValues.kd           = kd;

        networkingServer->setPubs(setValues);

        auto nsTime = std::chrono::high_resolution_clock::now();

        auto msTime = std::chrono::duration_cast<std::chrono::milliseconds>(nsTime.time_since_epoch());

        if (values.isAuto)
        {
            autoPlotData.append(velocity, targetVelocity, pose, msTime - startTime);
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Telemetry Data");

        ImGui::Text("X Position:    %.3f", (float) pose.X());
        ImGui::Text("Y Position:    %.3f", (float) pose.Y());

        ImGui::Separator();

        ImGui::Text("DX Value:      %.3f", (float) targetVelocity.X());
        ImGui::Text("DY Value:      %.3f", (float) targetVelocity.Y());
        ImGui::Text("Omega Value:   %.3f", (float) targetVelocity.Rotation().Radians());

        ImGui::Separator();

        ImGui::Text("Heading Value: %.3f", (float) pose.Rotation().Degrees());

        ImDrawList* drawList = ImGui::GetWindowDrawList();

        ImVec2 topLeft = ImGui::GetCursorScreenPos();

        ImGui::Dummy({Consts::fieldW * Consts::fieldScaleFactor, Consts::fieldH * Consts::fieldScaleFactor + Consts::robotH * Consts::fieldScaleFactor});

        drawList->AddImage((void*) (intptr_t) fieldImg,
                           {topLeft.x, topLeft.y + Consts::robotH * Consts::fieldScaleFactor / 2},
                           {topLeft.x + Consts::fieldW * Consts::fieldScaleFactor,
                            topLeft.y + Consts::fieldH * Consts::fieldScaleFactor + Consts::robotH * Consts::fieldScaleFactor / 2},
                           {(float) 90 / 3508, (float) 90 / 1814},
                           {1 - (float) 90 / 3508, 1 - (float) 90 / 1814});

        drawRotatedRect(drawList,
                        {topLeft.x + (float) pose.X() * Consts::fieldScaleFactor,
                         topLeft.y + Consts::fieldH * Consts::fieldScaleFactor - (float) pose.Y() * Consts::fieldScaleFactor + Consts::robotH * Consts::fieldScaleFactor / 2},
                        {Consts::robotH * Consts::fieldScaleFactor, Consts::robotW * Consts::fieldScaleFactor},
                        -(float) pose.Rotation().Degrees(),
                        IM_COL32(100, 125, 200, 255));

        for (const auto& customFieldSub : customFieldSubs)
        {
            nt::Value data = networkingServer->getCustomSub(customFieldSub);

            if (data && data.IsDoubleArray())
            {
                if (data.GetDoubleArray().size() == 3)
                {
                    drawRotatedRect(drawList,
                                    {topLeft.x + (float) data.GetDoubleArray()[0] * Consts::fieldScaleFactor,
                                     topLeft.y + Consts::fieldH * Consts::fieldScaleFactor - (float) data.GetDoubleArray()[1] * Consts::fieldScaleFactor + Consts::robotH * Consts::fieldScaleFactor / 2},
                                    {Consts::robotH * Consts::fieldScaleFactor, Consts::robotW * Consts::fieldScaleFactor},
                                    -(float) data.GetDoubleArray()[2],
                                    IM_COL32(255, 125, 200, 255));
                }
            }
        }

        std::vector<ImVec2> trajPointsToDraw;
        trajPointsToDraw.resize(trajPoints.size());

        for (int i = 0; i < trajPoints.size(); i++)
        {
            trajPointsToDraw[i] = {topLeft.x + (float) trajPoints[i].X() * Consts::fieldScaleFactor,
                                   topLeft.y + Consts::fieldH * Consts::fieldScaleFactor - (float) trajPoints[i].Y() * Consts::fieldScaleFactor};
        }

        drawList->AddPolyline(
            trajPointsToDraw.data(),
            trajPointsToDraw.size(),
            IM_COL32(0, 0, 255, 255),
            ImDrawFlags_None,
            2.0f);

        static bool showHistory   = false;
        static bool appendHistory = true;

        if (showHistory)
        {

            std::vector<ImVec2> historyPoints;
            historyPoints.resize(plotData.pose.size());

            for (int i = 0; i < plotData.pose.size(); i++)
            {
                historyPoints[i] = {topLeft.x + (float) plotData.pose[i].X() * Consts::fieldScaleFactor,
                                    topLeft.y + Consts::fieldH * Consts::fieldScaleFactor - (float) plotData.pose[i].Y() * Consts::fieldScaleFactor};
            }

            int numPoints = std::min<int>(historyPoints.size(), 10000);

            drawList->AddPolyline(historyPoints.data() + historyPoints.size() - numPoints,
                                  numPoints,
                                  IM_COL32(0, 255, 0, 255),
                                  2.0f);
        }

        drawCheckBox("Append History", appendHistory);

        if (appendHistory)
        {
            plotData.append(velocity, pose, msTime - startTime);
        }

        drawCheckBox("Show History", showHistory);
        if (ImGui::Button("Clear History"))
        {
            plotData.clear();
        }

        ImGui::Separator();

        static char fieldSearchBuffer[128] = "";
        std::vector<nt::Topic> fieldTopics = networkingServer->getFieldTable()->GetTopics();
        std::vector<std::string> fieldTopicKeys;
        fieldTopicKeys.reserve(fieldTopics.size());

        for (const auto& topic : fieldTopics)
        {
            std::string name = topic.GetName();

            if (name.starts_with(Consts::fieldPrefix))
            {
                name.erase(0, Consts::fieldPrefix.length());
            }

            fieldTopicKeys.push_back(name);
        }

        drawSearch(fieldTopicKeys,
                   "Search for objects to add to field",
                   fieldSearchBuffer,
                   IM_ARRAYSIZE(fieldSearchBuffer),
                   [this, &customFieldSubs](std::string option) mutable
                   { customFieldSubs.push_back(networkingServer->addFieldCustomSub(option)); });

        ImGui::Separator();

        drawCompass(drawList, "Heading:", -(float) pose.Rotation().Degrees() * Consts::PI / 180, 200);

        ImGui::End();

        ImGui::Begin("Autos");

        if (current_item_idx >= static_cast<int>(values.autoList.size()))
        {
            current_item_idx = 0;
        }

        std::string combo_preview_value;

        if (values.autoList.size())
        {
            combo_preview_value = values.autoList[current_item_idx];
        }
        else
        {
            combo_preview_value = "";
        }

        if (ImGui::BeginCombo("Autos", combo_preview_value.c_str()))
        {
            for (int n = 0; n < values.autoList.size(); n++)
            {
                const bool is_selected = (current_item_idx == n);

                if (ImGui::Selectable(values.autoList[n].c_str(), is_selected))
                {
                    current_item_idx = n;
                }

                if (is_selected)
                {
                    ImGui::SetItemDefaultFocus();
                    selectedAuto = values.autoList[n];
                }
            }
            ImGui::EndCombo();
        }

        ImGui::End();

        ImGui::Begin("Misc");

        ImGui::BeginDisabled(true);

        if (values.v < 12)
        {
            ImGui::PushStyleColor(ImGuiCol_SliderGrab, IM_COL32(255, 0, 0, 255));
        }
        else if (values.v < 12.4)
        {
            ImGui::PushStyleColor(ImGuiCol_SliderGrab, IM_COL32(255, 255, 0, 255));
        }
        else
        {
            ImGui::PushStyleColor(ImGuiCol_SliderGrab, IM_COL32(0, 255, 0, 255));
        }

        float vFloat = values.v;

        ImGui::SliderFloat("Voltage", &vFloat, 0, 14, "%.3f");

        ImGui::EndDisabled();

        ImGui::Separator();

        ImGui::InputDouble("KP", &kp, 0.001f, 1.0f, "%.4f");
        ImGui::InputDouble("KI", &ki, 0.001f, 1.0f, "%.4f");
        ImGui::InputDouble("KD", &kd, 0.001f, 1.0f, "%.4f");

        ImGui::Separator();

        ImGui::Text("NetworkTables Status: %s", networkingServer->isConnected() ? "Connected" : "Disconnected");

        ImGui::PopStyleColor();

        ImGui::End();

        ImGui::Begin("Mechanism State");

        ImGui::Text("Intake Lift Pos:                 %.3f", values.intakeLiftPos);
        ImGui::Text("Intake Lift Target Pos:          %.3f", values.intakeLiftTargetPos);

        ImGui::Separator();

        ImGui::Text("Intake Feed Power:               %.3f", values.intakeFeedPower);

        ImGui::Separator();

        ImGui::Text("Feed Power: %.3f", values.feedPower);

        ImGui::Separator();

        ImGui::Text("Kicker Power: %.3f", values.kickerPower);

        ImGui::Text("Shooter Left RPM:         %.3f", values.shooterLeftRPM);
        ImGui::Text("Shooter Left Target RPM:  %.3f", values.shooterLeftTargetRPM);
        ImGui::Text("Shooter Right RPM:        %.3f", values.shooterRightRPM);
        ImGui::Text("Shooter Right Target RPM: %.3f", values.shooterRightTargetRPM);

        ImGui::End();

        ImGui::Begin("Plots");

        if (ImPlot::BeginPlot("DX"))
        {
            ImPlot::SetupAxis(ImAxis_X1, "Time", ImPlotAxisFlags_AutoFit);
            ImPlot::SetupAxis(ImAxis_Y1, "Meters/Second", ImPlotAxisFlags_AutoFit);

            ImPlot::PlotLine("DX", autoPlotData.times.data(), autoPlotData.dx.data(), autoPlotData.dx.size());

            ImPlot::PlotLine("Target DX", autoPlotData.times.data(), autoPlotData.dxTarget.data(), autoPlotData.dxTarget.size());
            ImPlot::EndPlot();
        }

        if (ImPlot::BeginPlot("DY"))
        {
            ImPlot::SetupAxis(ImAxis_X1, "Time", ImPlotAxisFlags_AutoFit);
            ImPlot::SetupAxis(ImAxis_Y1, "Meters/Second", ImPlotAxisFlags_AutoFit);

            ImPlot::PlotLine("DY", autoPlotData.times.data(), autoPlotData.dy.data(), autoPlotData.dy.size());

            ImPlot::PlotLine("Target DY", autoPlotData.times.data(), autoPlotData.dyTarget.data(), autoPlotData.dyTarget.size());
            ImPlot::EndPlot();
        }

        if (ImPlot::BeginPlot("Angular Velocity"))
        {
            ImPlot::SetupAxis(ImAxis_X1, "Time", ImPlotAxisFlags_AutoFit);
            ImPlot::SetupAxis(ImAxis_Y1, "Degs/Second", ImPlotAxisFlags_AutoFit);

            ImPlot::PlotLine("Angular Velocity", autoPlotData.times.data(), autoPlotData.omega.data(), autoPlotData.omegaTarget.size());

            ImPlot::PlotLine("Target Angular Velocity", autoPlotData.times.data(), autoPlotData.omegaTarget.data(), autoPlotData.omegaTarget.size());
            ImPlot::EndPlot();
        }

        ImGui::Separator();

        int customPlotIdx = 0;

        for (auto& customPlot : customPlots)
        {
            ImGui::PushID(customPlotIdx);

            std::string name = networkingServer->getCustomSubName(customPlot.subId);

            if (name.starts_with(Consts::datatablePrefix))
            {
                name.erase(0, Consts::datatablePrefix.length());
            }

            if (ImPlot::BeginPlot(name.c_str()))
            {
                ImPlot::SetupAxis(ImAxis_X1, "Time", ImPlotAxisFlags_AutoFit);
                ImPlot::SetupAxis(ImAxis_Y1, name.c_str(), ImPlotAxisFlags_AutoFit);

                ImPlot::PlotLine(name.c_str(), customPlot.times.data(), customPlot.data.data(), customPlot.data.size());

                ImPlot::EndPlot();
            }

            ImGui::PopID();

            customPlotIdx++;
        }

        if (ImGui::Button("Clear Graphs"))
        {
            autoPlotData.clear();
            for (auto& customPlot : customPlots)
            {
                customPlot.clear();
            }
        }

        static bool appendingCustomPlots;
        drawCheckBox("Appending Custom Plots", appendingCustomPlots);

        if (appendingCustomPlots)
        {
            for (auto& customPlot : customPlots)
            {
                customPlot.update(networkingServer, (msTime - startTime).count());
            }
        }

        ImGui::Separator();

        static char plotSearchBuffer[128] = "";
        std::vector<nt::Topic> plotTopics = networkingServer->getTable()->GetTopics();
        std::vector<std::string> plotTopicKeys;
        plotTopicKeys.reserve(plotTopics.size());

        for (const auto& topic : plotTopics)
        {
            std::string name = topic.GetName();

            if (name.starts_with(Consts::datatablePrefix))
            {
                name.erase(0, Consts::datatablePrefix.length());
            }

            plotTopicKeys.push_back(name);
        }

        drawSearch(plotTopicKeys,
                   "Search for values to plot",
                   plotSearchBuffer,
                   IM_ARRAYSIZE(plotSearchBuffer),
                   [this, &customPlots](std::string option) mutable
                   { customPlots.push_back({{}, {}, networkingServer->addCustomSub(option)}); });

        ImGui::End();

        ImGui::Begin("Recording");

        static bool recording;
        drawCheckBox("Is Recording", recording);

        if (ImGui::Button("Clear recording"))
        {
            recordingManager->clearFile();
        }

        ImGui::Separator();

        static char fileNameBuffer[128] = "output.jsonl";

        ImGui::Text("Output Filename:");

        if (ImGui::InputTextWithHint("##SearchBar", "Type to set file name", fileNameBuffer, IM_ARRAYSIZE(fileNameBuffer)))
        {
            recordingManager->setFilePath(fileNameBuffer);
        }

        static char recordingSearchBuffer[128] = "";
        std::vector<nt::Topic> recordingTopics = networkingServer->getTable()->GetTopics();
        std::vector<std::string> recordingTopicKeys;
        recordingTopicKeys.reserve(recordingTopics.size());

        for (const auto& topic : recordingTopics)
        {
            std::string name = topic.GetName();

            if (name.starts_with(Consts::datatablePrefix))
            {
                name.erase(0, Consts::datatablePrefix.length());
            }

            recordingTopicKeys.push_back(name);
        }

        drawSearch(recordingTopicKeys,
                   "Search for values to record",
                   recordingSearchBuffer,
                   IM_ARRAYSIZE(recordingSearchBuffer),
                   [this, &customRecordingSubs](std::string option) mutable
                   { customRecordingSubs.push_back(networkingServer->addCustomSub(option)); });

        ImGui::End();

        if (recording)
        {
            std::vector<nt::Value> customValues{};
            std::vector<std::string> customValuesKeys{};

            for (const auto& customRecordingSub : customRecordingSubs)
            {
                customValues.push_back(networkingServer->getCustomSub(customRecordingSub));
                customValuesKeys.push_back(networkingServer->getCustomSubName(customRecordingSub));
            }

            recordingManager->appendToFile(values, customValues, customValuesKeys);
        }

        static char searchBuffer[128] = "";
        std::vector<nt::Topic> topics = networkingServer->getTable()->GetTopics();
        std::vector<std::string> topicKeys;
        topicKeys.reserve(topics.size());

        for (const auto& topic : topics)
        {
            std::string name = topic.GetName();

            if (name.starts_with(Consts::datatablePrefix))
            {
                name.erase(0, Consts::datatablePrefix.length());
            }

            topicKeys.push_back(name);
        }

        ImGui::Begin("Custom Viewer");

        drawSearch(topicKeys,
                   "Search for values to view",
                   searchBuffer,
                   IM_ARRAYSIZE(searchBuffer),
                   [this, &customSubs](std::string option) mutable
                   { customSubs.push_back(networkingServer->addCustomSub(option)); });

        ImGui::Separator();

        for (const auto& customSub : customSubs)
        {
            std::string name = networkingServer->getCustomSubName(customSub);

            if (name.starts_with(Consts::datatablePrefix))
            {
                name.erase(0, Consts::datatablePrefix.length());
            }

            displayGeneric(name, networkingServer->getCustomSub(customSub));
        }

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
    ImPlot::DestroyContext();
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
