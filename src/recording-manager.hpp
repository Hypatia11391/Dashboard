#pragma once

#include "src/networking-server.hpp"

#include <string>

class RecordingManager
{
    public:
        RecordingManager(std::string filePath);

        void appendToFile(Values values, std::vector<nt::Value> customValues, std::vector<std::string> customValuesKeys);
        void clearFile();

        void setFilePath(std::string newFilePath) { filePath = newFilePath; }

    private:
        std::string filePath;
};