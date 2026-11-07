#include "src/recording-manager.hpp"

#include <fstream>
#include <iostream>
#include <string>

#include <networktables/NetworkTableValue.h>
#include <wpi/json.h>

RecordingManager::RecordingManager(std::string filePath) : filePath(filePath) {}

wpi::json nt_to_json(const nt::Value& value)
{
    if (!value || value.type() == NT_UNASSIGNED)
    {
        return nullptr;
    }

    switch (value.type())
    {
    case NT_BOOLEAN:
        return value.GetBoolean();

    case NT_DOUBLE:
        return value.GetDouble();

    case NT_FLOAT:
        return value.GetFloat();

    case NT_INTEGER:
        return value.GetInteger();

    case NT_STRING:
        return value.GetString();

    case NT_BOOLEAN_ARRAY:
        return value.GetBooleanArray();

    case NT_DOUBLE_ARRAY:
        return value.GetDoubleArray();

    case NT_FLOAT_ARRAY:
        return value.GetFloatArray();

    case NT_INTEGER_ARRAY:
        return value.GetIntegerArray();

    case NT_STRING_ARRAY:
        return value.GetStringArray();

    case NT_RAW:
        return value.GetRaw();

    case NT_RPC:
        return "[NT_RPC Procedure]";

    case NT_UNASSIGNED:
    default:
        return nullptr;
    }
}

void RecordingManager::update(Values values, std::vector<nt::Value> customValues, std::vector<std::string> customValuesKeys)
{
    wpi::json frameData = {
        {"appliedOutput", values.appliedOutput},
        {"robotPose", values.robotPose},
        {"robotTraj", values.robotTraj},
        {"robotVelocity", values.robotVelocity},
        {"robotTargetVelocity", values.robotTargetVelocity},
        {"intakeLiftPos", values.intakeLiftPos},
        {"intakeLiftTargetPos", values.intakeLiftTargetPos},
        {"intakeFeedPower", values.intakeFeedPower},
        {"feedPower", values.feedPower},
        {"kickerPower", values.kickerPower},
        {"shooterLeftRPM", values.shooterLeftRPM},
        {"shooterLeftTargetRPM", values.shooterLeftTargetRPM},
        {"shooterRightRPM", values.shooterRightRPM},
        {"shooterRightTargetRPM", values.shooterRightTargetRPM},
        {"v", values.v},
        {"autoList", values.autoList},
        {"isAuto", values.isAuto},
        {"time", values.msTime.count()}};

    assert(customValues.size() == customValuesKeys.size());

    for (int i = 0; i < customValues.size(); i++)
    {
        frameData[customValuesKeys[i]] = nt_to_json(customValues[i]);
    }

    std::ofstream file(filePath, std::ios::app);
    if (file.is_open())
    {
        file << frameData.dump() << std::endl;
        file.close();
    }
}

void RecordingManager::clearFile()
{
    std::ofstream file(filePath, std::ios::out | std::ios::trunc);

    if (file.is_open())
    {
        file.close();
    }
}