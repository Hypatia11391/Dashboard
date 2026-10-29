#include "src/networking-server.hpp"
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

    fieldTable        = table->GetSubTable("field");
    robotPoseSub      = fieldTable->GetDoubleArrayTopic("robot").Subscribe({}, createInfo.options);
    robotVelocitySub  = fieldTable->GetDoubleArrayTopic("velocity").Subscribe({}, createInfo.options);
    targetVelocitySub = fieldTable->GetDoubleArrayTopic("target-velocity").Subscribe({}, createInfo.options);
    trajPoseSub       = fieldTable->GetDoubleArrayTopic("trajectory").Subscribe({}, createInfo.options);

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

    values.robotPose      = robotPoseSub.Get();
    values.trajPose       = trajPoseSub.Get();
    values.robotVelocity  = robotVelocitySub.Get();
    values.targetVelocity = targetVelocitySub.Get();

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