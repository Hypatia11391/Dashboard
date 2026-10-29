#include "src/recording-manager.hpp"

#include <fstream>
#include <iostream>
#include <string>

#include <networktables/NetworkTableValue.h>

#include <vendor/json.hpp>

using json = nlohmann::json;

RecordingManager::RecordingManager(std::string filePath) : filePath(filePath) {}

json nt_to_json(const nt::Value& value)
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

void RecordingManager::appendToFile(Values values, std::vector<nt::Value> customValues, std::vector<std::string> customValuesKeys)
{
    json frameData = values;

    frameData["time"] = nt::Now();

    assert(customValues.size() == customValuesKeys.size());

    for (int i = 0; i < customValues.size(); i++)
    {
        frameData[customValuesKeys[i]] = nt_to_json(customValues[i]);
    }

    bool isAuto;

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