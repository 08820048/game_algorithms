#include "ui.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "demo.h"

namespace ga::ui {
namespace {

constexpr float kTextSize = 11.0f;

struct State {
    bool hoveredLastFrame = false;
    bool hoveredThisFrame = false;
    const void* active = nullptr;  // 正在拖动的滑块
};

State s;

bool hovering(Rectangle r) {
    bool h = CheckCollisionPointRec(GetMousePosition(), r);
    if (h) s.hoveredThisFrame = true;
    return h;
}

void frameRect(Rectangle r, Color c) {
    float w = hairline();
    DrawRectangleLinesEx(r, w, c);
}

}  // namespace

bool button(Rectangle r, const char* label, bool selected) {
    const Palette& p = palette();
    bool hover = hovering(r);
    bool clicked = hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);

    if (selected) {
        DrawRectangleRec(r, p.text);
    } else {
        DrawRectangleRec(r, hover ? p.panel : p.background);
    }
    frameRect(r, selected ? p.text : p.gridLine);
    drawTextCentered(label, {r.x + r.width * 0.5f, r.y + r.height * 0.5f}, kTextSize,
                     selected ? p.background : p.text);
    return clicked;
}

bool toggle(Rectangle r, const char* label, bool* value) {
    const Palette& p = palette();
    bool hover = hovering(r);
    bool clicked = hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
    if (clicked) *value = !*value;

    DrawRectangleRec(r, hover ? p.panel : p.background);
    frameRect(r, p.gridLine);
    // 左侧小方块表示开关状态
    float box = 9.0f;
    Rectangle b{r.x + 10, r.y + (r.height - box) * 0.5f, box, box};
    if (*value) {
        DrawRectangleRec(b, p.goal);
    } else {
        frameRect(b, p.textMuted);
    }
    drawText(label, {b.x + box + 8, r.y + (r.height - kTextSize) * 0.5f - 1}, kTextSize, p.text);
    return clicked;
}

bool slider(Rectangle r, const char* label, float* value, float min, float max, const char* format, float step) {
    const Palette& p = palette();
    const float labelW = measureText(label, kTextSize).x + 12;
    const float valueW = 52;
    Rectangle track{r.x + labelW, r.y, r.width - labelW - valueW, r.height};

    bool hover = hovering(r);
    if (hover && IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) s.active = value;

    bool changed = false;
    if (s.active == value) {
        s.hoveredThisFrame = true;
        float t = std::clamp((GetMousePosition().x - track.x) / track.width, 0.0f, 1.0f);
        float v = min + (max - min) * t;
        if (step > 0) v = std::round(v / step) * step;
        v = std::clamp(v, min, max);
        if (v != *value) {
            *value = v;
            changed = true;
        }
    }

    float cy = r.y + r.height * 0.5f;
    drawText(label, {r.x, cy - kTextSize * 0.5f - 1}, kTextSize, p.textMuted);
    DrawLineEx({track.x, cy}, {track.x + track.width, cy}, 1.0f, p.text);
    float t = (*value - min) / (max - min);
    float tx = track.x + track.width * t;
    float k = s.active == value ? 11.0f : 9.0f;
    DrawRectangleRec({tx - k * 0.5f, cy - k * 0.5f, k, k}, p.goal);

    char buf[32];
    std::snprintf(buf, sizeof buf, format, *value);
    drawText(buf, {track.x + track.width + 10, cy - kTextSize * 0.5f - 1}, kTextSize, p.text);
    return changed;
}

bool wantsMouse() { return s.hoveredLastFrame || s.active != nullptr; }

namespace detail {
void beginFrame() {
    s.hoveredLastFrame = s.hoveredThisFrame;
    s.hoveredThisFrame = false;
    if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) s.active = nullptr;
}
}  // namespace detail

}  // namespace ga::ui
