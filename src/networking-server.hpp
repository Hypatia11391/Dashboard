#pragma once

#include "src/constants.hpp"

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

#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

#include <vendor/json.hpp>

struct Values
{
    public:
        std::vector<double> appliedOutput;

        std::vector<double> robotPose;
        std::vector<double> trajPose;
        std::vector<double> robotVelocity;
        std::vector<double> targetVelocity;

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
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Values,
                                   appliedOutput,
                                   robotPose,
                                   trajPose,
                                   robotVelocity,
                                   targetVelocity,
                                   intakeLiftPos,
                                   intakeLiftTargetPos,
                                   intakeFeedPower,
                                   feedPower,
                                   kickerPower,
                                   shooterLeftRPM,
                                   shooterLeftTargetRPM,
                                   shooterRightRPM,
                                   shooterRightTargetRPM,
                                   v,
                                   autoList,
                                   isAuto)

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

        std::vector<nt::GenericSubscriber> customSubs;
};