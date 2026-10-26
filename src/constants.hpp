#pragma once

#include <string_view>

namespace Consts
{
inline constexpr float PI = 3.14159265358979323846;

inline constexpr float fieldW           = 16.54;
inline constexpr float fieldH           = 8;
inline constexpr float fieldScaleFactor = 30;

inline constexpr float robotW = 1;
inline constexpr float robotH = 0.85;

inline constexpr std::string_view datatablePrefix = "/datatable/";
inline constexpr std::string_view fieldPrefix     = "/datatable/field/";
} // namespace Consts