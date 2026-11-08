#pragma once

#include <frc/geometry/Pose2d.h>

#include <string>
#include <vector>

struct Values
{
    public:
        std::vector<double> appliedOutput;

        frc::Pose2d robotPose;
        std::vector<frc::Pose2d> robotTraj;
        frc::Pose2d robotVelocity;
        frc::Pose2d robotTargetVelocity;

        double intakeLiftPos;
        double intakeLiftTargetPos;
        double intakeFeedPower;

        double feedPower;

        double kickerPower;

        double shooterLeftRPM;
        double shooterLeftTargetRPM;
        double shooterRightRPM;
        double shooterRightTargetRPM;

        double v;

        std::vector<std::string> autoList;

        bool isAuto;

        std::chrono::milliseconds msTime;
};