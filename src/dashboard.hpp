#pragma once

#include "src/networking-server.hpp"
#include "src/recording-manager.hpp"

#include <GLFW/glfw3.h>

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

struct PlotData
{
    public:
        std::vector<double> dx{};
        std::vector<double> dy{};
        std::vector<double> omega{};
        std::vector<frc::Pose2d> pose{};
        std::vector<double> times{};

    public:
        void append(frc::Pose2d velocity, frc::Pose2d currentPose, std::chrono::milliseconds time)
        {
            dx.push_back(velocity.X().value());
            dy.push_back(velocity.Y().value());
            omega.push_back(velocity.Rotation().Degrees().value());

            double doubleTime = std::chrono::duration<double, std::milli>(time).count();

            pose.push_back(currentPose);

            times.push_back(doubleTime);
        }

        void clear()
        {
            dx.clear();
            dy.clear();
            omega.clear();
            pose.clear();
            times.clear();
        }
};

struct CustomPlotData
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

struct AutoPlotData
{
    public:
        std::vector<double> dx{};
        std::vector<double> dy{};
        std::vector<double> omega{};
        std::vector<double> dxTarget{};
        std::vector<double> dyTarget{};
        std::vector<double> omegaTarget{};
        std::vector<frc::Pose2d> pose{};
        std::vector<double> times{};

    public:
        void append(frc::Pose2d velocity, frc::Pose2d targetVelocity, frc::Pose2d currentPose, std::chrono::milliseconds time)
        {
            dx.push_back(velocity.X().value());
            dy.push_back(velocity.Y().value());
            omega.push_back(velocity.Rotation().Degrees().value());

            dxTarget.push_back(targetVelocity.X().value());
            dyTarget.push_back(targetVelocity.Y().value());
            omegaTarget.push_back(targetVelocity.Rotation().Degrees().value());

            double doubleTime = std::chrono::duration<double, std::milli>(time).count();

            pose.push_back(currentPose);

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

class Dashboard
{
    public:
        Dashboard(const DashboardCreateInfo& createInfo);
        void run();

    private:
        void init();
        void mainLoop();
        void cleanUp();

        GLuint LoadTextureFromFile(const char* filename, int* out_width, int* out_height);

    private:
        GLFWwindow* window;

        GLuint fieldImg;

        int current_item_idx = 0;

        std::chrono::milliseconds startTime;

    private:
        DashboardCreateInfo createInfo;

        std::shared_ptr<NetworkingServer> networkingServer;
        std::shared_ptr<RecordingManager> recordingManager;

        PlotData plotData;
        AutoPlotData autoPlotData;
};