#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <cmath>

#include "src/constants.hpp"
#include "src/primitives.hpp"

void drawRotatedRect(ImDrawList* drawList, ImVec2 center, ImVec2 size, float rot, ImU32 col)
{
    float rad = rot * consts::PI / 180.0f;
    float c   = std::cos(rad);
    float s   = std::sin(rad);

    float hx = size.x / 2.0f;
    float hy = size.y / 2.0f;

    float dx1 = -hx * c - (-hy) * s;
    float dy1 = -hx * s + (-hy) * c;

    float dx2 = hx * c - (-hy) * s;
    float dy2 = hx * s + (-hy) * c;

    float dx3 = hx * c - hy * s;
    float dy3 = hx * s + hy * c;

    float dx4 = -hx * c - hy * s;
    float dy4 = -hx * s + hy * c;

    ImVec2 p1 = {center.x + dx1, center.y + dy1};
    ImVec2 p2 = {center.x + dx2, center.y + dy2};
    ImVec2 p3 = {center.x + dx3, center.y + dy3};
    ImVec2 p4 = {center.x + dx4, center.y + dy4};

    drawList->AddQuad(p1, p2, p3, p4, col);
}

void drawCompass(ImDrawList* draw_list, const char* label, float angle_radians, float radius)
{
    ImGui::BeginGroup();
    ImGui::Text("%s", label);

    ImVec2 center = ImGui::GetCursorScreenPos();

    center.x += radius;
    center.y += radius;

    ImGui::Dummy(ImVec2(radius * 2.0f, radius * 2.0f));

    ImU32 color_north = ImGui::GetColorU32(ImVec4(1.0f, 0.2f, 0.2f, 1.0f));

    draw_list->AddCircleFilled(center, radius, IM_COL32(25, 40, 60, 255), 64);

    float angle_offset = angle_radians;
    float cos_a        = std::cos(angle_offset);
    float sin_a        = std::sin(angle_offset);

    ImVec2 needle_tip(center.x + cos_a * radius * 0.85f, center.y + sin_a * radius * 0.85f);

    ImVec2 needle_tail(center.x - cos_a * radius * 0.85f, center.y - sin_a * radius * 0.85f);

    float perp_cos = std::cos(angle_offset + (3.14159265f / 2.0f));
    float perp_sin = std::sin(angle_offset + (3.14159265f / 2.0f));
    ImVec2 needle_left(center.x + perp_cos * radius * 0.2f, center.y + perp_sin * radius * 0.2f);
    ImVec2 needle_right(center.x - perp_cos * radius * 0.2f, center.y - perp_sin * radius * 0.2f);

    draw_list->AddTriangleFilled(needle_right, needle_left, needle_tip, color_north);

    draw_list->AddLine({center.x - radius, center.y},
                       {center.x + 10 - radius, center.y},
                       color_north,
                       2);
    draw_list->AddLine({center.x + radius, center.y},
                       {center.x - 10 + radius, center.y},
                       color_north,
                       2);
    draw_list->AddLine({center.x, center.y - radius},
                       {center.x, center.y + 10 - radius},
                       color_north,
                       2);
    draw_list->AddLine({center.x, center.y + radius},
                       {center.x, center.y - 10 + radius},
                       color_north,
                       2);

    float degrees = angle_radians * (180.0f / 3.14159265f);
    if (degrees < 0.0f)
        degrees += 360.0f;

    ImGui::EndGroup();
}