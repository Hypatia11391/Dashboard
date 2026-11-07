#include <include/glad/glad.h>

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

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <frc/geometry/Pose2d.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/GenericEntry.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#include <wpi/json.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

#include "src/constants.hpp"
#include "src/dashboard.hpp"
#include "src/primitives.hpp"
#include "wpi/json_fwd.h"

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

    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress))
    {
        throw std::runtime_error("Failed to initialize GLAD");
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 150 core");

    ImGui::StyleColorsDark();

    int my_image_width  = 0;
    int my_image_height = 0;
    fieldImg            = LoadTextureFromFile("field26.png", &my_image_width, &my_image_height);

    std::chrono::duration<double, std::milli> currentTime(nt::Now());

    startTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime);

    baseImGuiStyle = ImGui::GetStyle();

    loadConfigFile();

    init3D();
}

void Dashboard::init3D()
{
    // 1. Compile Shaders
    shaderProgram = compileShaders();

    // 2. Set up Vertex Data for a Cube
    float vertices[] = {
        // positions (x,y,z)     // colors (r,g,b)
        -0.5f, -0.5f, -0.5f, 1.0f, 0.0f, 0.0f,
        0.5f, -0.5f, -0.5f, 0.0f, 1.0f, 0.0f,
        0.5f, 0.5f, -0.5f, 0.0f, 0.0f, 1.0f,
        0.5f, 0.5f, -0.5f, 0.0f, 0.0f, 1.0f,
        -0.5f, 0.5f, -0.5f, 1.0f, 1.0f, 0.0f,
        -0.5f, -0.5f, -0.5f, 1.0f, 0.0f, 0.0f,

        -0.5f, -0.5f, 0.5f, 1.0f, 0.0f, 0.0f,
        0.5f, -0.5f, 0.5f, 0.0f, 1.0f, 0.0f,
        0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 1.0f,
        0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 1.0f,
        -0.5f, 0.5f, 0.5f, 1.0f, 1.0f, 0.0f,
        -0.5f, -0.5f, 0.5f, 1.0f, 0.0f, 0.0f,

        -0.5f, 0.5f, 0.5f, 1.0f, 0.0f, 0.0f,
        -0.5f, 0.5f, -0.5f, 0.0f, 1.0f, 0.0f,
        -0.5f, -0.5f, -0.5f, 0.0f, 0.0f, 1.0f,
        -0.5f, -0.5f, -0.5f, 0.0f, 0.0f, 1.0f,
        -0.5f, -0.5f, 0.5f, 1.0f, 1.0f, 0.0f,
        -0.5f, 0.5f, 0.5f, 1.0f, 0.0f, 0.0f,

        0.5f, 0.5f, 0.5f, 1.0f, 0.0f, 0.0f,
        0.5f, 0.5f, -0.5f, 0.0f, 1.0f, 0.0f,
        0.5f, -0.5f, -0.5f, 0.0f, 0.0f, 1.0f,
        0.5f, -0.5f, -0.5f, 0.0f, 0.0f, 1.0f,
        0.5f, -0.5f, 0.5f, 1.0f, 1.0f, 0.0f,
        0.5f, 0.5f, 0.5f, 1.0f, 0.0f, 0.0f,

        -0.5f, -0.5f, -0.5f, 1.0f, 0.0f, 0.0f,
        0.5f, -0.5f, -0.5f, 0.0f, 1.0f, 0.0f,
        0.5f, -0.5f, 0.5f, 0.0f, 0.0f, 1.0f,
        0.5f, -0.5f, 0.5f, 0.0f, 0.0f, 1.0f,
        -0.5f, -0.5f, 0.5f, 1.0f, 1.0f, 0.0f,
        -0.5f, -0.5f, -0.5f, 1.0f, 0.0f, 0.0f,

        -0.5f, 0.5f, -0.5f, 1.0f, 0.0f, 0.0f,
        0.5f, 0.5f, -0.5f, 0.0f, 1.0f, 0.0f,
        0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 1.0f,
        0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 1.0f,
        -0.5f, 0.5f, 0.5f, 1.0f, 1.0f, 0.0f,
        -0.5f, 0.5f, -0.5f, 1.0f, 0.0f, 0.0f};

    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*) 0);
    glEnableVertexAttribArray(0);
    // Color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*) (3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // 3. Create Framebuffer Object (FBO) for off-screen rendering
    // Fixed resolution for now. For dynamic resizing, recreate this when the ImGui window resizes.
    const int FBO_WIDTH  = 1280;
    const int FBO_HEIGHT = 720;

    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    // Create a color attachment texture
    glGenTextures(1, &textureColorbuffer);
    glBindTexture(GL_TEXTURE_2D, textureColorbuffer);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, FBO_WIDTH, FBO_HEIGHT, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, textureColorbuffer, 0);

    // Create a renderbuffer object for depth and stencil attachment
    glGenRenderbuffers(1, &rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, FBO_WIDTH, FBO_HEIGHT);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

GLuint Dashboard::compileShaders()
{
    const char* vertexShaderSource = "#version 130\n"
                                     "in vec3 aPos;\n"
                                     "in vec3 aColor;\n"
                                     "out vec3 ourColor;\n"
                                     "uniform mat4 model;\n"
                                     "uniform mat4 view;\n"
                                     "uniform mat4 projection;\n"
                                     "void main()\n"
                                     "{\n"
                                     "   gl_Position = projection * view * model * vec4(aPos, 1.0);\n"
                                     "   ourColor = aColor;\n"
                                     "}\0";

    const char* fragmentShaderSource = "#version 130\n"
                                       "in vec3 ourColor;\n"
                                       "out vec4 FragColor;\n"
                                       "void main()\n"
                                       "{\n"
                                       "   FragColor = vec4(ourColor, 1.0f);\n"
                                       "}\n\0";

    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    GLuint shaderProg = glCreateProgram();
    glAttachShader(shaderProg, vertexShader);
    glAttachShader(shaderProg, fragmentShader);
    glLinkProgram(shaderProg);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return shaderProg;
}

void Dashboard::mainLoop()
{
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        Values values = networkingServer->getValues();

        SetValues setValues{};
        setValues.selectedAuto = selectedAuto;
        setValues.kp           = kp;
        setValues.ki           = ki;
        setValues.kd           = kd;

        networkingServer->setPubs(setValues);

        if (values.isAuto)
        {
            autoData.append(values.robotPose, values.robotVelocity, values.robotTargetVelocity, values.msTime - startTime);
        }

        applyPendingGuiChange();

        render(values);
    }
}

void Dashboard::render(const Values& values)
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    drawWindows(values);

    render3DScene();

    ImGui::Begin("3D Field View");
    ImVec2 viewportSize = ImGui::GetContentRegionAvail();
    ImGui::Image((void*) (intptr_t) textureColorbuffer, viewportSize, ImVec2(0, 1), ImVec2(1, 0));
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

void Dashboard::render3DScene()
{
    // Bind our off-screen framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    // Setup viewport to match FBO dimensions
    glViewport(0, 0, 1280, 720);

    // Enable depth testing (crucial for 3D)
    glEnable(GL_DEPTH_TEST);

    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(shaderProgram);

    // Create transformations
    glm::mat4 model = glm::mat4(1.0f);
    // Rotate cube over time
    model = glm::rotate(model, (float) glfwGetTime(), glm::vec3(0.5f, 1.0f, 0.0f));

    glm::mat4 view = glm::mat4(1.0f);
    // Move camera back a bit
    view = glm::translate(view, glm::vec3(0.0f, 0.0f, -3.0f));

    glm::mat4 projection = glm::perspective(glm::radians(45.0f), (float) 1280 / (float) 720, 0.1f, 100.0f);

    // Pass matrices to shaders
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

    // Render the cube
    glBindVertexArray(vao);
    glDrawArrays(GL_TRIANGLES, 0, 36);

    // Reset state for ImGui
    glDisable(GL_DEPTH_TEST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Dashboard::drawWindows(const Values& values)
{
    drawTelemetryWindow(values);
    drawAutosWindow(values);
    drawMiscWindow(values);
    drawMechanismsWindow(values);
    drawPlotsWindow(values);
    drawRecordingWindow(values);
    drawCustomViewerWindow(values);
    drawConfigWindow();
}

void Dashboard::drawTelemetryWindow(const Values& values)
{
    ImGui::Begin("Telemetry Data");

    ImGui::Text("X Position:    %.3f", (float) values.robotPose.X());
    ImGui::Text("Y Position:    %.3f", (float) values.robotPose.Y());

    ImGui::Separator();

    ImGui::Text("DX Value:      %.3f", (float) values.robotTargetVelocity.X());
    ImGui::Text("DY Value:      %.3f", (float) values.robotTargetVelocity.Y());
    ImGui::Text("Omega Value:   %.3f", (float) values.robotTargetVelocity.Rotation().Radians());

    ImGui::Separator();

    ImGui::Text("Heading Value: %.3f", (float) values.robotPose.Rotation().Degrees());

    ImDrawList* drawList = ImGui::GetWindowDrawList();

    ImVec2 topLeft = ImGui::GetCursorScreenPos();

    ImGui::Dummy({Consts::fieldW * fieldScaleFactor, Consts::fieldH * fieldScaleFactor + Consts::robotH * fieldScaleFactor});

    drawList->AddImage((void*) (intptr_t) fieldImg,
                       {topLeft.x, topLeft.y + Consts::robotH * fieldScaleFactor / 2},
                       {topLeft.x + Consts::fieldW * fieldScaleFactor,
                        topLeft.y + Consts::fieldH * fieldScaleFactor + Consts::robotH * fieldScaleFactor / 2},
                       {(float) 90 / 3508, (float) 90 / 1814},
                       {1 - (float) 90 / 3508, 1 - (float) 90 / 1814});

    drawRotatedRect(drawList,
                    {topLeft.x + (float) values.robotPose.X() * fieldScaleFactor,
                     topLeft.y + Consts::fieldH * fieldScaleFactor - (float) values.robotPose.Y() * fieldScaleFactor + Consts::robotH * fieldScaleFactor / 2},
                    {Consts::robotH * fieldScaleFactor, Consts::robotW * fieldScaleFactor},
                    -(float) values.robotPose.Rotation().Degrees(),
                    IM_COL32(100, 125, 200, 255));

    for (const auto& customFieldSub : customFieldSubs)
    {
        nt::Value data = networkingServer->getCustomSub(customFieldSub);

        if (data && data.IsDoubleArray())
        {
            if (data.GetDoubleArray().size() == 3)
            {
                drawRotatedRect(drawList,
                                {topLeft.x + (float) data.GetDoubleArray()[0] * fieldScaleFactor,
                                 topLeft.y + Consts::fieldH * fieldScaleFactor - (float) data.GetDoubleArray()[1] * fieldScaleFactor + Consts::robotH * fieldScaleFactor / 2},
                                {Consts::robotH * fieldScaleFactor, Consts::robotW * fieldScaleFactor},
                                -(float) data.GetDoubleArray()[2],
                                IM_COL32(255, 125, 200, 255));
            }
        }
    }

    std::vector<ImVec2> trajPointsToDraw;
    trajPointsToDraw.resize(values.robotTraj.size());

    for (int i = 0; i < values.robotTraj.size(); i++)
    {
        trajPointsToDraw[i] = {topLeft.x + (float) values.robotTraj[i].X() * fieldScaleFactor,
                               topLeft.y + Consts::fieldH * fieldScaleFactor - (float) values.robotTraj[i].Y() * fieldScaleFactor};
    }

    drawList->AddPolyline(
        trajPointsToDraw.data(),
        trajPointsToDraw.size(),
        IM_COL32(0, 0, 255, 255),
        ImDrawFlags_None,
        2.0f);

    if (showHistory)
    {

        std::vector<ImVec2> historyPoints;
        historyPoints.resize(poseHistory.pose.size());

        for (int i = 0; i < poseHistory.pose.size(); i++)
        {
            historyPoints[i] = {topLeft.x + (float) poseHistory.pose[i].X() * fieldScaleFactor,
                                topLeft.y + Consts::fieldH * fieldScaleFactor - (float) poseHistory.pose[i].Y() * fieldScaleFactor};
        }

        int numPoints = std::min<int>(historyPoints.size(), Consts::numHistoryPointsShown);

        drawList->AddPolyline(historyPoints.data() + historyPoints.size() - numPoints,
                              numPoints,
                              IM_COL32(0, 255, 0, 255),
                              2.0f);
    }

    drawCheckBox("Append History", appendHistory);

    if (appendHistory)
    {
        poseHistory.append(values.robotPose);

        if (poseHistory.pose.size() > Consts::numHistoryPointsShown * 2)
        {
            poseHistory.pose.erase(poseHistory.pose.begin(), poseHistory.pose.begin() + Consts::numHistoryPointsShown);
        }
    }

    drawCheckBox("Show History", showHistory);
    if (ImGui::Button("Clear History"))
    {
        poseHistory.clear();
    }

    ImGui::Separator();
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
               [this](std::string option) mutable
               { customFieldSubs.push_back(networkingServer->addFieldCustomSub(option)); });

    ImGui::Separator();

    drawCompass(drawList, "Heading:", -(float) values.robotPose.Rotation().Degrees() * Consts::PI / 180, 200 * config.guiScale);

    ImGui::End();
}

void Dashboard::drawAutosWindow(const Values& values)
{
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
}

void Dashboard::drawMiscWindow(const Values& values)
{
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
}

void Dashboard::drawMechanismsWindow(const Values& values)
{
    ImGui::Begin("Mechanisms State");

    ImGui::Text("Intake Lift Pos:          %.3f", values.intakeLiftPos);
    ImGui::Text("Intake Lift Target Pos:   %.3f", values.intakeLiftTargetPos);

    ImGui::Separator();

    ImGui::Text("Intake Feed Power:        %.3f", values.intakeFeedPower);

    ImGui::Separator();

    ImGui::Text("Feed Power:               %.3f", values.feedPower);

    ImGui::Separator();

    ImGui::Text("Kicker Power:             %.3f", values.kickerPower);

    ImGui::Text("Shooter Left RPM:         %.3f", values.shooterLeftRPM);
    ImGui::Text("Shooter Left Target RPM:  %.3f", values.shooterLeftTargetRPM);
    ImGui::Text("Shooter Right RPM:        %.3f", values.shooterRightRPM);
    ImGui::Text("Shooter Right Target RPM: %.3f", values.shooterRightTargetRPM);

    ImGui::End();
}

void Dashboard::drawPlotsWindow(const Values& values)
{
    ImGui::Begin("Plots");

    float plotHeight = ImPlot::GetStyle().PlotDefaultSize.y * config.guiScale;

    std::cout << config.guiScale << " " << plotHeight << " " << ImPlot::GetStyle().PlotDefaultSize.y << std::endl;

    ImVec2 plotSize{-1, plotHeight};

    if (ImPlot::BeginPlot("DX", plotSize))
    {
        ImPlot::SetupAxis(ImAxis_X1, "Time", ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxis(ImAxis_Y1, "Meters/Second", ImPlotAxisFlags_AutoFit);

        ImPlot::PlotLine("DX", autoData.times.data(), autoData.dx.data(), autoData.dx.size());

        ImPlot::PlotLine("Target DX", autoData.times.data(), autoData.dxTarget.data(), autoData.dxTarget.size());
        ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("DY", plotSize))
    {
        ImPlot::SetupAxis(ImAxis_X1, "Time", ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxis(ImAxis_Y1, "Meters/Second", ImPlotAxisFlags_AutoFit);

        ImPlot::PlotLine("DY", autoData.times.data(), autoData.dy.data(), autoData.dy.size());

        ImPlot::PlotLine("Target DY", autoData.times.data(), autoData.dyTarget.data(), autoData.dyTarget.size());
        ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("Angular Velocity", plotSize))
    {
        ImPlot::SetupAxis(ImAxis_X1, "Time", ImPlotAxisFlags_AutoFit);
        ImPlot::SetupAxis(ImAxis_Y1, "Degs/Second", ImPlotAxisFlags_AutoFit);

        ImPlot::PlotLine("Angular Velocity", autoData.times.data(), autoData.omega.data(), autoData.omegaTarget.size());

        ImPlot::PlotLine("Target Angular Velocity", autoData.times.data(), autoData.omegaTarget.data(), autoData.omegaTarget.size());
        ImPlot::EndPlot();
    }

    ImGui::Separator();

    int customPlotIdx = 0;

    for (auto& customPlot : customPlotInfo)
    {
        ImGui::PushID(customPlotIdx);

        std::string name = networkingServer->getCustomSubName(customPlot.subId);

        if (name.starts_with(Consts::datatablePrefix))
        {
            name.erase(0, Consts::datatablePrefix.length());
        }

        if (ImPlot::BeginPlot(name.c_str(), plotSize))
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
        autoData.clear();
        for (auto& customPlot : customPlotInfo)
        {
            customPlot.clear();
        }
    }
    drawCheckBox("Appending Custom Plots", appendingCustomPlots);

    if (appendingCustomPlots)
    {
        for (auto& customPlot : customPlotInfo)
        {
            customPlot.update(networkingServer, (values.msTime - startTime).count());
        }
    }

    ImGui::Separator();
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
               [this](std::string option) mutable
               { customPlotInfo.push_back({{}, {}, networkingServer->addCustomSub(option)}); });

    ImGui::End();
}

void Dashboard::drawRecordingWindow(const Values& values)
{
    ImGui::Begin("Recording");
    drawCheckBox("Is Recording", recording);

    if (ImGui::Button("Clear recording"))
    {
        recordingManager->clearFile();
    }

    ImGui::Separator();

    ImGui::Text("Output Filename:");

    if (ImGui::InputTextWithHint("##SearchBar", "Type to set file name", fileNameBuffer, IM_ARRAYSIZE(fileNameBuffer)))
    {
        recordingManager->setFilePath(fileNameBuffer);
    }
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
               [this](std::string option) mutable
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

        recordingManager->update(values, customValues, customValuesKeys);
    }
}

void Dashboard::drawCustomViewerWindow(const Values& values)
{
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
               [this](std::string option) mutable
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
}

void Dashboard::drawConfigWindow()
{
    ImGui::Begin("Config");

    if (ImGui::InputFloat("GUI Scale", &config.guiScale, 0.01, 0.1))
    {
        config.guiScale = std::min(Consts::maxGuiScale, std::max(Consts::minGuiScale, config.guiScale));

        pendingGuiChange = true;
    }

    if (ImGui::Button("Save Config To File"))
    {
        saveConfigFile();
    }

    ImGui::End();
}

void Dashboard::cleanUp()
{
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteProgram(shaderProgram);
    glDeleteFramebuffers(1, &fbo);
    glDeleteTextures(1, &textureColorbuffer);
    glDeleteRenderbuffers(1, &rbo);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
}

void Dashboard::applyPendingGuiChange()
{
    if (pendingGuiChange)
    {
        ImGuiIO& io        = ImGui::GetIO();
        float scale_factor = config.guiScale;

        fieldScaleFactor = Consts::defaultFieldScaleValue * scale_factor;

        io.Fonts->Clear();

        ImFontConfig config;
        config.OversampleH = 4;
        config.OversampleV = 4;

        float base_font_size = 14.0f;
        io.Fonts->AddFontFromFileTTF(std::string(Consts::fontPath).c_str(), base_font_size * scale_factor, &config);

        ImGui_ImplOpenGL3_DestroyDeviceObjects();

        io.Fonts->Build();

        ImGui_ImplOpenGL3_CreateDeviceObjects();

        ImGui::GetStyle() = baseImGuiStyle;
        ImGui::GetStyle().ScaleAllSizes(scale_factor);

        pendingGuiChange = false;
    }
}

void Dashboard::saveConfigFile()
{
    wpi::json j{};

    j["gui"]["scale"] = config.guiScale;

    std::ofstream configFile(Consts::configFilePath.data());
    if (configFile.is_open())
    {
        configFile << j.dump(4) << std::endl;
        configFile.close();
    }
}

void Dashboard::loadConfigFile()
{
    std::ifstream file(Consts::configFilePath.data());
    if (!file.is_open())
    {
        std::cerr << "WARNING: Failed to open config file, NOTE: Will always happen if just deleted config file or first time running and config file doesn't exist yet, try saving config to file and restarting dashboard" << std::endl;
        return;
    }

    try
    {
        wpi::json j = wpi::json::parse(file);

        config.guiScale  = j["gui"]["scale"];
        pendingGuiChange = true;
    }
    catch (const wpi::json::parse_error& e)
    {
        std::cerr << "WARNING: Failed to parse config file, JSON Parse Error: " << e.what() << std::endl;
    }
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
