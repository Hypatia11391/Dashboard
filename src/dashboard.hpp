#pragma once

#include <GLFW/glfw3.h>

#include <frc/geometry/Pose2d.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringArrayTopic.h>
#include <networktables/StringTopic.h>

#include <chrono>

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

class Dashboard
{
    public:
        Dashboard();
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
        nt::NetworkTableInstance inst;
        std::shared_ptr<nt::NetworkTable> table;

        nt::DoubleArraySubscriber appliedOutputSub;

        std::shared_ptr<nt::NetworkTable> fieldTable;
        nt::DoubleArraySubscriber robotPoseSub;
        nt::DoubleArraySubscriber trajPoseSub;
        nt::DoubleArraySubscriber robotVelocitySub;
        nt::DoubleArraySubscriber targetVelocitySub;

        nt::DoubleSubscriber intakeLiftPosSub;
        nt::DoubleSubscriber intakeLiftTargetPosSub;
        nt::DoubleSubscriber intakeFeedPowerSub;

        nt::DoubleSubscriber feedPowerSub;

        nt::DoubleSubscriber kickerPowerSub;

        nt::DoubleSubscriber shooterLeftRPMSub;
        nt::DoubleSubscriber shooterLeftTargetRPMSub;
        nt::DoubleSubscriber shooterRightRPMSub;
        nt::DoubleSubscriber shooterRightTargetRPMSub;

        nt::DoubleSubscriber vSub;

        nt::StringArraySubscriber autoListSub;
        nt::StringPublisher autoPub;

        nt::DoublePublisher kpPub;
        nt::DoublePublisher kiPub;
        nt::DoublePublisher kdPub;

        nt::BooleanSubscriber isAutoSub;

        PlotData plotData;
        AutoPlotData autoPlotData;
};