#include "src/networking-server.hpp"
#include "networktables/NetworkTableValue.h"
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

#include <string>
#include <vector>

NetworkingServer::NetworkingServer(const NetworkingServerCreateInfo& createInfo) : createInfo(createInfo)
{
    inst = nt::NetworkTableInstance::GetDefault();

    inst.SetServerTeam(Consts::teamNum);
    inst.StartClient4("Dashboard Client");

    if (createInfo.isSimulation)
    {
        inst.SetServer("127.0.0.1");
    }
    else
    {
        inst.StartDSClient();
    }

    table = inst.GetTable("datatable");

    tableSub = nt::MultiSubscriber(inst, std::vector<std::string_view>{"/datatable/"});

    appliedOutputSub = table->GetDoubleArrayTopic("drive-applied-output").Subscribe({}, createInfo.options);

    fieldTable             = table->GetSubTable("field");
    robotPoseSub           = fieldTable->GetDoubleArrayTopic("robot").Subscribe({}, createInfo.options);
    robotVelocitySub       = fieldTable->GetDoubleArrayTopic("velocity").Subscribe({}, createInfo.options);
    robotTargetVelocitySub = fieldTable->GetDoubleArrayTopic("target-velocity").Subscribe({}, createInfo.options);
    robotTrajSub           = fieldTable->GetDoubleArrayTopic("trajectory").Subscribe({}, createInfo.options);

    intakeLiftPosSub       = table->GetDoubleTopic("intake-lift-pos").Subscribe({}, createInfo.options);
    intakeLiftTargetPosSub = table->GetDoubleTopic("intake-lift-target-pos").Subscribe({}, createInfo.options);
    intakeFeedPowerSub     = table->GetDoubleTopic("intake-feed-power").Subscribe({}, createInfo.options);

    feedPowerSub   = table->GetDoubleTopic("feed-power").Subscribe({}, createInfo.options);
    kickerPowerSub = table->GetDoubleTopic("kicker-power").Subscribe({}, createInfo.options);

    shooterLeftRPMSub        = table->GetDoubleTopic("shooter-left-rpm").Subscribe({}, createInfo.options);
    shooterLeftTargetRPMSub  = table->GetDoubleTopic("shooter-left-target-rpm").Subscribe({}, createInfo.options);
    shooterRightRPMSub       = table->GetDoubleTopic("shooter-right-rpm").Subscribe({}, createInfo.options);
    shooterRightTargetRPMSub = table->GetDoubleTopic("shooter-right-target-rpm").Subscribe({}, createInfo.options);

    vSub = table->GetDoubleTopic("v").Subscribe({}, createInfo.options);

    autoPub     = table->GetStringTopic("selected-auto").Publish(createInfo.options);
    autoListSub = table->GetStringArrayTopic("auto-list").Subscribe({}, createInfo.options);

    isAutoSub = table->GetBooleanTopic("is-in-auto").Subscribe({}, createInfo.options);

    kpPub = table->GetDoubleTopic("pid-kp").Publish(createInfo.options);
    kiPub = table->GetDoubleTopic("pid-ki").Publish(createInfo.options);
    kdPub = table->GetDoubleTopic("pid-kd").Publish(createInfo.options);
}

Values NetworkingServer::getValues() const
{
    Values values{};

    values.appliedOutput = appliedOutputSub.Get();

    values.robotPose           = getRobotPose();
    values.robotTraj           = getRobotTraj();
    values.robotVelocity       = getRobotVelocity();
    values.robotTargetVelocity = getRobotTargetVelocity();

    values.intakeLiftPos       = intakeLiftPosSub.Get();
    values.intakeLiftTargetPos = intakeLiftTargetPosSub.Get();
    values.intakeFeedPower     = intakeFeedPowerSub.Get();

    values.feedPower = feedPowerSub.Get();

    values.kickerPower = kickerPowerSub.Get();

    values.shooterLeftRPM        = shooterLeftRPMSub.Get();
    values.shooterLeftTargetRPM  = shooterLeftTargetRPMSub.Get();
    values.shooterRightRPM       = shooterRightRPMSub.Get();
    values.shooterRightTargetRPM = shooterRightTargetRPMSub.Get();

    values.v = vSub.Get();

    values.autoList = std::move(autoListSub.Get());
    values.isAuto   = isAutoSub.Get();

    std::chrono::duration<double, std::milli> double_ms(nt::Now());

    values.msTime = std::chrono::duration_cast<std::chrono::milliseconds>(double_ms);

    return values;
}

void NetworkingServer::setPubs(const SetValues& setValues)
{
    autoPub.Set(setValues.selectedAuto);

    kpPub.Set(setValues.kp);
    kiPub.Set(setValues.ki);
    kdPub.Set(setValues.kd);
}

int NetworkingServer::addCustomSub(std::string key)
{
    customSubs.push_back(table->GetEntry(key).GetTopic().GenericSubscribe(createInfo.options));

    return customSubs.size() - 1;
}

int NetworkingServer::addFieldCustomSub(std::string key)
{
    customSubs.push_back(fieldTable->GetEntry(key).GetTopic().GenericSubscribe(createInfo.options));

    return customSubs.size() - 1;
}

nt::Value NetworkingServer::getCustomSub(int id) const
{
    if (id >= 0 && id < customSubs.size())
    {
        return customSubs[id].Get();
    }
    return nt::Value();
}

const std::string NetworkingServer::getCustomSubName(int id) const
{
    if (id >= 0 && id < customSubs.size())
    {
        return customSubs[id].GetTopic().GetName();
    }
    return {};
}

frc::Pose2d NetworkingServer::getRobotPose() const
{
    std::vector<double> data = robotPoseSub.Get();

    if (data.size() == 3)
    {
        return {units::meter_t(data[0]),
                units::meter_t(data[1]),
                {units::degree_t(data[2])}};
    }
    else
    {
        return {};
    }
}

frc::Pose2d NetworkingServer::getRobotVelocity() const
{
    std::vector<double> data = robotVelocitySub.Get();

    if (data.size() == 3)
    {
        return {units::meter_t(data[0]),
                units::meter_t(data[1]),
                {units::degree_t(data[2])}};
    }
    else
    {
        return {};
    }
}

std::vector<frc::Pose2d> NetworkingServer::getRobotTraj() const
{
    std::vector<double> data = robotTrajSub.Get();

    if (data.size() % 3 == 0)
    {
        std::vector<frc::Pose2d> traj;
        traj.reserve(data.size() / 3);

        for (int i = 0; i < data.size() / 3; i++)
        {
            traj.push_back({units::meter_t(data[i * 3 + 0]),
                            units::meter_t(data[i * 3 + 1]),
                            {units::degree_t(data[i * 3 + 2])}});
        }

        return traj;
    }

    else
    {
        return {};
    }
}

frc::Pose2d NetworkingServer::getRobotTargetVelocity() const
{
    std::vector<double> data = robotTargetVelocitySub.Get();

    if (data.size() == 3)
    {
        return {units::meter_t(data[0]),
                units::meter_t(data[1]),
                {units::degree_t(data[2])}};
    }
    else
    {
        return {};
    }
}