#pragma once

struct ImFont;
struct GLFWwindow;

namespace viewer::imgui_theme
{
void apply_kit_dark_style();
bool load_ui_font(GLFWwindow *window);
ImFont *ui_bold_font();
void init_ui_scale();
float ui_scale();
float ui(float value);

} // namespace viewer::imgui_theme
