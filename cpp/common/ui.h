#pragma once

// 画布内的即时模式控件（immediate-mode UI）：在 Demo::drawUi() 中调用，
// 每个控件同时负责绘制和处理输入，返回值表示"本帧是否被操作"。

#include "raylib.h"

namespace ga::ui {

// 普通按钮；selected 为 true 时以选中样式显示（用作分段选择器 / 标签页）
bool button(Rectangle r, const char* label, bool selected = false);

// 开关按钮：点击时翻转 *value
bool toggle(Rectangle r, const char* label, bool* value);

// 横向滑块：左侧标签、中间轨道、右侧数值。step > 0 时按步长取整
bool slider(Rectangle r, const char* label, float* value, float min, float max, const char* format = "%.0f",
            float step = 0.0f);

// 鼠标是否落在控件上或正在拖动控件。演示在 update() 中据此跳过对场景的鼠标操作
bool wantsMouse();

namespace detail {
void beginFrame();  // 由演示框架每帧调用
}

}  // namespace ga::ui
