#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

void drawRotatedRect(ImDrawList* drawList, ImVec2 center, ImVec2 size, float rot, ImU32 col);
void drawCompass(ImDrawList* draw_list, const char* label, float angle_radians, float radius = 50.0f);