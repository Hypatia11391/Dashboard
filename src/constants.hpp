#pragma once

#include <string_view>

namespace Consts
{
inline constexpr float PI = 3.14159265358979323846;

inline constexpr float fieldW = 16.54;
inline constexpr float fieldH = 8;

inline constexpr float robotW = 1;
inline constexpr float robotH = 0.85;

inline constexpr std::string_view fontPath = "assets/fonts/UbuntuMono-Regular.ttf";

inline constexpr std::string_view configFilePath = "dashboard-config.json";

inline constexpr float minGuiScale = 0.25;
inline constexpr float maxGuiScale = 10.0;

inline constexpr std::string_view datatablePrefix = "/datatable/";
inline constexpr std::string_view fieldPrefix     = "/datatable/field/";

inline constexpr double pubSubRate = 0.01;

inline constexpr int teamNum = 11391;

inline constexpr int numHistoryPointsShown = 1e3;
} // namespace Consts