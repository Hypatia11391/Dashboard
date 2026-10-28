#include <algorithm>
#include <chrono>
#include <cstdio>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#include "networktables/Topic.h"
#include "stb_image.h"

#include <GLFW/glfw3.h>

#include <frc/geometry/Pose2d.h>
#include <iterator>
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
    auto inst = nt::NetworkTableInstance::GetDefault();
    inst.StartClient4("wsl_client");
    inst.SetServer("127.0.0.1");
    table = inst.GetTable("datatable");

    options          = nt::PubSubOptions{};
    options.periodic = 0.02;

    tableSub = {inst, std::vector<std::string_view>{"/datatable/"}};

    appliedOutputSub = table->GetDoubleArrayTopic("drive-applied-output").Subscribe({}, options);

    fieldTable        = table->GetSubTable("field");
    robotPoseSub      = fieldTable->GetDoubleArrayTopic("robot").Subscribe({}, options);
    robotVelocitySub  = fieldTable->GetDoubleArrayTopic("velocity").Subscribe({}, options);
    targetVelocitySub = fieldTable->GetDoubleArrayTopic("target-velocity").Subscribe({}, options);
    trajPoseSub       = fieldTable->GetDoubleArrayTopic("trajectory").Subscribe({}, options);

    intakeLiftPosSub       = table->GetDoubleTopic("intake-lift-pos").Subscribe(0.0, options);
    intakeLiftTargetPosSub = table->GetDoubleTopic("intake-lift-target-pos").Subscribe(0.0, options);
    intakeFeedPowerSub     = table->GetDoubleTopic("intake-feed-power").Subscribe(0.0, options);

    feedPowerSub = table->GetDoubleTopic("feed-power").Subscribe(0.0, options);

    kickerPowerSub = table->GetDoubleTopic("kicker-power").Subscribe(0.0, options);

    shooterLeftRPMSub        = table->GetDoubleTopic("shooter-left-rpm").Subscribe(0.0, options);
    shooterLeftTargetRPMSub  = table->GetDoubleTopic("shooter-left-target-rpm").Subscribe(0.0, options);
    shooterRightRPMSub       = table->GetDoubleTopic("shooter-right-rpm").Subscribe(0.0, options);
    shooterRightTargetRPMSub = table->GetDoubleTopic("shooter-right-target-rpm").Subscribe(0.0, options);

    vSub = table->GetDoubleTopic("v").Subscribe(0.0, options);

    autoPub     = table->GetStringTopic("selected-auto").Publish();
    autoListSub = table->GetStringArrayTopic("auto-list").Subscribe({}, options);

    isAutoSub = table->GetBooleanTopic("is-in-auto").Subscribe(false, options);

    kpPub = table->GetDoubleTopic("pid-kp").Publish(options);
    kiPub = table->GetDoubleTopic("pid-ki").Publish(options);
    kdPub = table->GetDoubleTopic("pid-kd").Publish(options);

    inst.StartClient4("Dashboard Client");
    inst.SetServerTeam(11391);
    inst.StartDSClient();

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
    glfwSwapInterval(0);

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
    double kp = 0, ki = 0, kd = 0;

    json outputArray = json::array();

    std::vector<nt::GenericSubscriber> customSubs{};
    std::vector<nt::DoubleArraySubscriber> customFieldSubs{};

    std::vector<CustomPlotData> customPlots{};

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        auto appliedDriveOutputs = appliedOutputSub.Get();

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
            velocity = {units::meter_t(0),
                        units::meter_t(0),
                        {units::degree_t(0)}};
        }

        auto targetVelocityData = targetVelocitySub.Get();

        frc::Pose2d targetVelocity;

        if (targetVelocityData.size() >= 3)
        {

            targetVelocity = {units::meter_t(targetVelocityData[0]),
                              units::meter_t(targetVelocityData[1]),
                              {units::degree_t(targetVelocityData[2])}};
        }
        else
        {
            targetVelocity = {units::meter_t(0),
                              units::meter_t(0),
                              {units::degree_t(0)}};
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

        bool isAuto = isAutoSub.Get();

        kpPub.Set(kp);
        kiPub.Set(ki);
        kdPub.Set(kd);

        auto nsTime = std::chrono::high_resolution_clock::now();

        auto msTime = std::chrono::duration_cast<std::chrono::milliseconds>(nsTime.time_since_epoch());

        if (isAuto)
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
            std::vector<double> data = customFieldSub.Get();

            if (data.size() == 3)
            {
                drawRotatedRect(drawList,
                                {topLeft.x + (float) data[0] * Consts::fieldScaleFactor,
                                 topLeft.y + Consts::fieldH * Consts::fieldScaleFactor - (float) data[1] * Consts::fieldScaleFactor + Consts::robotH * Consts::fieldScaleFactor / 2},
                                {Consts::robotH * Consts::fieldScaleFactor, Consts::robotW * Consts::fieldScaleFactor},
                                -(float) pose.Rotation().Degrees(),
                                IM_COL32(255, 125, 200, 255));
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
        std::vector<nt::Topic> fieldTopics = fieldTable->GetTopics();
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

        ImGui::Text("Search For Field Objects:");

        if (ImGui::InputTextWithHint("##SearchBar",
                                     "Type to filter...",
                                     fieldSearchBuffer,
                                     IM_ARRAYSIZE(fieldSearchBuffer)))
        {
        }

        ImGui::Separator();

        ImGuiTextFilter fieldFilter(fieldSearchBuffer);

        int foundFieldObjects = 0;

        for (int i = 0; i < fieldTopicKeys.size(); i++)
        {
            const auto& topicKey = fieldTopicKeys[i];

            if (fieldFilter.PassFilter(topicKey.c_str()))
            {
                if (++foundFieldObjects > 5)
                {
                    break;
                }

                if (ImGui::Selectable(topicKey.c_str()))
                {
                    nt::DoubleArrayTopic topic = fieldTable->GetDoubleArrayTopic(topicKey);
                    customFieldSubs.push_back(topic.Subscribe({}, options));
                }
            }
        }

        ImGui::Separator();

        drawCompass(drawList, "Heading:", -(float) pose.Rotation().Degrees() * Consts::PI / 180, 200);

        // Code to have a chassis speed thing not using
        // ImGui::Separator();

        // topLeft = ImGui::GetCursorScreenPos();

        // ImGui::Dummy(ImVec2(200 * 2.0f, 200 * 2.0f));

        // frc::Translation2d norm = velocity.Translation() / velocity.Translation().Norm().value();
        // DrawVectorArrow(drawList,
        //                 {topLeft.x + 200, topLeft.y + 200},
        //                 {topLeft.x + 200 + (float) norm.X(), topLeft.y + 200 + (float) norm.Y()},
        //                 IM_COL32(255, 255, 255, 255));

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

        ImGui::Separator();

        ImGui::InputDouble("KP", &kp, 0.001f, 1.0f, "%.4f");
        ImGui::InputDouble("KI", &ki, 0.001f, 1.0f, "%.4f");
        ImGui::InputDouble("KD", &kd, 0.001f, 1.0f, "%.4f");

        ImGui::Separator();

        ImGui::Text("NetworkTables Status: %s", inst.IsConnected() ? "Connected" : "Disconnected");

        ImGui::PopStyleColor();

        ImGui::EndDisabled();

        ImGui::End();

        ImGui::Begin("Mechanism State");

        ImGui::Text("Intake Lift Pos:                 %.3f", intakeLiftPos);
        ImGui::Text("Intake Lift Target Pos:          %.3f", intakeLiftTargetPos);

        ImGui::Separator();

        ImGui::Text("Intake Feed Power:               %.3f", intakeFeedPower);

        ImGui::Separator();

        ImGui::Text("Feed Power: %.3f", feedPower);

        ImGui::Separator();

        ImGui::Text("Kicker Power: %.3f", kickerPower);

        ImGui::Text("Shooter Left RPM:         %.3f", shooterLeftRPM);
        ImGui::Text("Shooter Left Target RPM:  %.3f", shooterLeftTargetRPM);
        ImGui::Text("Shooter Right RPM:        %.3f", shooterRightRPM);
        ImGui::Text("Shooter Right Target RPM: %.3f", shooterRightTargetRPM);

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

            if (ImPlot::BeginPlot(customPlot.sub.GetTopic().GetName().c_str()))
            {
                ImPlot::SetupAxis(ImAxis_X1, "Time", ImPlotAxisFlags_AutoFit);
                ImPlot::SetupAxis(ImAxis_Y1, customPlot.sub.GetTopic().GetName().c_str(), ImPlotAxisFlags_AutoFit);

                ImPlot::PlotLine(customPlot.sub.GetTopic().GetName().c_str(), customPlot.times.data(), customPlot.data.data(), customPlot.data.size());

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
                customPlot.update((msTime - startTime).count());
            }
        }

        ImGui::Separator();

        static char plotSearchBuffer[128] = "";
        std::vector<nt::Topic> plotTopics = table->GetTopics();
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

        ImGui::Text("Search For Plot Objects:");

        if (ImGui::InputTextWithHint("##SearchBar",
                                     "Type to filter...",
                                     plotSearchBuffer,
                                     IM_ARRAYSIZE(plotSearchBuffer)))
        {
        }

        ImGui::Separator();

        ImGuiTextFilter plotFilter(plotSearchBuffer);

        int foundPlotObjects = 0;

        for (int i = 0; i < plotTopicKeys.size(); i++)
        {
            const auto& topicKey = plotTopicKeys[i];

            if (plotFilter.PassFilter(topicKey.c_str()))
            {
                if (++foundPlotObjects > 5)
                {
                    break;
                }

                if (ImGui::Selectable(topicKey.c_str()))
                {
                    nt::DoubleTopic topic = table->GetDoubleTopic(topicKey);

                    customPlots.push_back({{},
                                           {},
                                           topic.Subscribe({}, options)});
                }
            }
        }

        ImGui::End();

        ImGui::Begin("Recording");

        static bool recording;
        drawCheckBox("Is Recording", recording);

        ImGui::Separator();

        static bool isRecordingTime;
        static bool isRecordingPose;
        static bool isRecordingVelocity;
        static bool isRecordingAppliedOutputs;
        static bool isRecordingTargetVelocity;
        static bool isRecordingTargetVoltage;

        drawCheckBox("Is Recording Time", isRecordingTime);
        drawCheckBox("Is Recording Pose", isRecordingPose);
        drawCheckBox("Is Recording Velocity", isRecordingVelocity);
        drawCheckBox("Is Recording Applied Outputs", isRecordingAppliedOutputs);
        drawCheckBox("Is Recording Target Velocity", isRecordingTargetVelocity);
        drawCheckBox("Is Recording Target Voltage", isRecordingTargetVoltage);

        if (recording)
        {
            json frameData = json::object();

            if (isRecordingTime) frameData["time"] = nt::Now();
            if (isRecordingPose) frameData["pose"] = poseData;
            if (isRecordingVelocity) frameData["velocity"] = velocityData;
            if (isRecordingAppliedOutputs) frameData["applied-outputs"] = appliedDriveOutputs;
            if (isRecordingTargetVelocity) frameData["target-velocity"] = targetVelocityData;
            if (isRecordingTargetVoltage) frameData["voltage"] = v;

            std::ofstream file("output.jsonl", std::ios::app);
            if (file.is_open())
            {
                file << frameData.dump() << std::endl;
                file.close();
            }
        }

        ImGui::Separator();

        static char fileNameBuffer[128] = "output.jsonl";

        ImGui::Text("Output Filename:");

        if (ImGui::InputTextWithHint("##SearchBar", "Type to set file name", fileNameBuffer, IM_ARRAYSIZE(fileNameBuffer)))
        {
        }

        ImGui::End();

        static char searchBuffer[128] = "";
        std::vector<nt::Topic> topics = table->GetTopics();
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

        ImGui::Text("Search:");

        if (ImGui::InputTextWithHint("##SearchBar", "Type to filter...", searchBuffer, IM_ARRAYSIZE(searchBuffer)))
        {
        }

        ImGui::Separator();

        ImGuiTextFilter filter(searchBuffer);

        int found = 0;

        for (int i = 0; i < topicKeys.size(); i++)
        {
            const auto& topicKey = topicKeys[i];

            if (filter.PassFilter(topicKey.c_str()))
            {
                if (++found > 5)
                {
                    break;
                }

                if (ImGui::Selectable(topicKey.c_str()))
                {
                    customSubs.push_back(table->GetTopic(topicKey).GenericSubscribe({}, options));
                }
            }
        }

        ImGui::Separator();

        for (const auto& customSub : customSubs)
        {
            displayGeneric(customSub);
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
