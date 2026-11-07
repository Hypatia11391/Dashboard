#pragma once

#include "src/constants.hpp"
#include "src/networking-server.hpp"
#include "src/recording-manager.hpp"

#include <GLFW/glfw3.h>

#include <imgui/imgui.h>

#include <frc/geometry/Pose2d.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/MultiSubscriber.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringArrayTopic.h>
#include <networktables/StringTopic.h>

#include <chrono>
#include <memory>
#include <optional>

struct PoseHistory
{
    public:
        std::vector<frc::Pose2d> pose{};

    public:
        void append(frc::Pose2d currentPose)
        {
            pose.push_back(currentPose);
        }

        void clear()
        {
            pose.clear();
        }
};

struct CustomPlotInfo
{
    public:
        std::vector<double> data{};
        std::vector<double> times{};
        int subId;

    public:
        void update(std::shared_ptr<NetworkingServer> networkingServer, double time)
        {
            nt::Value gotData = networkingServer->getCustomSub(subId);

            if (gotData && gotData.IsDouble())
            {
                times.push_back(time);
                data.push_back(gotData.GetDouble());
            }
        }

        void clear()
        {
            data.clear();
            times.clear();
        }
};

struct RobotData
{
    public:
        std::vector<double> dx{};
        std::vector<double> dy{};
        std::vector<double> omega{};
        std::vector<double> dxTarget{};
        std::vector<double> dyTarget{};
        std::vector<double> omegaTarget{};
        std::vector<double> x{};
        std::vector<double> y{};
        std::vector<double> heading{};
        std::vector<double> times{};

    public:
        void append(frc::Pose2d pose, frc::Pose2d velocity, frc::Pose2d targetVelocity, std::chrono::milliseconds time)
        {
            dx.push_back(velocity.X().value());
            dy.push_back(velocity.Y().value());
            omega.push_back(velocity.Rotation().Degrees().value());

            dxTarget.push_back(targetVelocity.X().value());
            dyTarget.push_back(targetVelocity.Y().value());
            omegaTarget.push_back(targetVelocity.Rotation().Degrees().value());

            double doubleTime = std::chrono::duration<double, std::milli>(time).count();

            x.push_back(pose.X().value());
            y.push_back(pose.Y().value());
            heading.push_back(pose.Rotation().Degrees().value());

            times.push_back(doubleTime);
        }

        void clear()
        {
            dx.clear();
            dy.clear();
            omega.clear();
            dxTarget.clear();
            dyTarget.clear();
            omegaTarget.clear();
            times.clear();
        }
};

struct DashboardCreateInfo
{
    public:
        bool isSimulation = false;
};

struct GuiConfig
{
    public:
        float guiScale = 1;
};

class Dashboard
{
    public:
        Dashboard(const DashboardCreateInfo& createInfo);
        void run();

    private:
        void init();
        void mainLoop();
        void cleanUp();

        void render(const Values& values);
        void drawWindows(const Values& values);

        void drawTelemetryWindow(const Values& values);
        void drawAutosWindow(const Values& values);
        void drawMiscWindow(const Values& values);
        void drawMechanismsWindow(const Values& values);
        void drawPlotsWindow(const Values& values);
        void drawRecordingWindow(const Values& values);
        void drawCustomViewerWindow(const Values& values);

        void drawConfigWindow();

        void applyPendingGuiChange();
        void saveConfigFile();
        void loadConfigFile();

        GLuint LoadTextureFromFile(const char* filename, int* out_width, int* out_height);

    private:
        GLFWwindow* window;

        GLuint fieldImg;

        int current_item_idx = 0;

        GLuint fbo                = 0;
        GLuint textureColorbuffer = 0;
        GLuint rbo                = 0;
        GLuint shaderProgram      = 0;
        GLuint vao                = 0;
        GLuint vbo                = 0;

        void init3D();
        void render3DScene();
        GLuint compileShaders();

        std::chrono::milliseconds startTime;

    private:
        double kp{}, ki{}, kd{};
        std::string selectedAuto{};

        std::vector<int> customSubs{};
        std::vector<int> customFieldSubs{};
        std::vector<int> customRecordingSubs{};

        DashboardCreateInfo createInfo;

        std::shared_ptr<NetworkingServer> networkingServer;
        std::shared_ptr<RecordingManager> recordingManager;

        PoseHistory poseHistory{};
        RobotData autoData{};
        std::vector<CustomPlotInfo> customPlotInfo{};

        GuiConfig config{};
        bool pendingGuiChange = false;

        ImGuiStyle baseImGuiStyle;
        float fieldScaleFactor = Consts::defaultFieldScaleValue;

        bool showHistory                = false;
        bool appendHistory              = true;
        char fieldSearchBuffer[128]     = "";
        bool appendingCustomPlots       = false;
        char plotSearchBuffer[128]      = "";
        bool recording                  = false;
        char fileNameBuffer[128]        = "output.jsonl";
        char recordingSearchBuffer[128] = "";
        char searchBuffer[128]          = "";
};