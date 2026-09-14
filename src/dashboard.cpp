#include <include/glad/glad.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "stb_image.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

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

    kpPub = table->GetDoubleTopic("pid-kp").Publish();
    kiPub = table->GetDoubleTopic("pid-ki").Publish();
    kdPub = table->GetDoubleTopic("pid-kd").Publish();

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

    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress))
    {
        throw std::runtime_error("Failed to initialize GLAD");
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    ImGui::StyleColorsDark();

    int my_image_width  = 0;
    int my_image_height = 0;
    fieldImg            = LoadTextureFromFile("field26.png", &my_image_width, &my_image_height);

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
    double kp = 0, ki = 0, kd = 0;

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

        kpPub.Set(kp);
        kiPub.Set(ki);
        kdPub.Set(kd);

        render3DScene();

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

        ImGui::Begin("PID");

        ImGui::InputDouble("KP", &kp, 0.001f, 1.0f, "%.4f");
        ImGui::InputDouble("KI", &ki, 0.001f, 1.0f, "%.4f");
        ImGui::InputDouble("KD", &kd, 0.001f, 1.0f, "%.4f");

        ImGui::End();

        ImGui::Begin("3D Field View");
        // Get available space in this window
        ImVec2 viewportSize = ImGui::GetContentRegionAvail();
        // Render the FBO texture.
        // Note: OpenGL textures are upside down in ImGui by default. Passing uv0=(0,1) and uv1=(1,0) flips it.
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