#pragma once

#include <networktables/GenericEntry.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

void drawRotatedRect(ImDrawList* drawList, ImVec2 center, ImVec2 size, float rot, ImU32 col);
void drawCompass(ImDrawList* draw_list, const char* label, float angle_radians, float radius = 50.0f);
void DrawVectorArrow(ImDrawList* draw_list, ImVec2 start, ImVec2 end, ImU32 color, float thickness = 2.0f, float arrowhead_size = 10.0f);
void drawCheckBox(const char* label, bool& state);
void displayGeneric(const std::string& name, const nt::Value& value);
void drawSearch(std::vector<std::string> options, std::string label, char* searchBuf, size_t bufLen, std::function<void(std::string)> fn);