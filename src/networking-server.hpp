#pragma once

#include "src/constants.hpp"
#include "src/networking-values.hpp"

#include <frc/geometry/Pose2d.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/GenericEntry.h>
#include <networktables/MultiSubscriber.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringArrayTopic.h>
#include <networktables/StringTopic.h>
#include <wpi/json.h>

#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

#include <vendor/json.hpp>

struct SetValues
{
    public:
        std::string_view selectedAuto;

        double kp;
        double ki;
        double kd;
};

struct NetworkingServerCreateInfo
{
    public:
        bool isSimulation = false;

        nt::PubSubOptions options = {.periodic = Consts::pubSubRate};
};

class NetworkingServer
{
    public:
        NetworkingServer(const NetworkingServerCreateInfo& createInfo);

        Values getValues() const;
        void setPubs(const SetValues& setValues);
        int addCustomSub(std::string key);
        int addFieldCustomSub(std::string key);
        nt::Value getCustomSub(int id) const;

        frc::Pose2d getRobotPose() const;
        frc::Pose2d getRobotVelocity() const;
        std::vector<frc::Pose2d> getRobotTraj() const;
        frc::Pose2d getRobotTargetVelocity() const;

        const std::shared_ptr<nt::NetworkTable> getTable() const { return table; }
        const std::shared_ptr<nt::NetworkTable> getFieldTable() const { return fieldTable; }
        const bool isConnected() const { return inst.IsConnected(); }
        const std::string getCustomSubName(int id) const;

    private:
        NetworkingServerCreateInfo createInfo;

        nt::NetworkTableInstance inst;
        std::shared_ptr<nt::NetworkTable> table;
        nt::MultiSubscriber tableSub;

        nt::DoubleArraySubscriber appliedOutputSub;

        std::shared_ptr<nt::NetworkTable> fieldTable;
        nt::DoubleArraySubscriber robotPoseSub;
        nt::DoubleArraySubscriber robotTrajSub;
        nt::DoubleArraySubscriber robotVelocitySub;
        nt::DoubleArraySubscriber robotTargetVelocitySub;

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

        std::vector<nt::GenericSubscriber> customSubs;
};