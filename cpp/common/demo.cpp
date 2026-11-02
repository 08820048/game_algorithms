#include "demo.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "ga_font_data.h"

#if defined(PLATFORM_WEB)
#include <emscripten/emscripten.h>
#include <emscripten/html5.h>
#define GA_EXPORT extern "C" EMSCRIPTEN_KEEPALIVE
#else
#define GA_EXPORT extern "C"
#endif

namespace ga {
namespace {

// 与网站 src/styles/custom.css 中的 --ga-* 变量保持一致
constexpr Palette kLight{
    /*background*/ {251, 249, 244, 255},  // #FBF9F4 纸
    /*gridLine*/ {231, 226, 214, 255},
    /*wall*/ {38, 36, 32, 255},           // 墨
    /*text*/ {26, 25, 21, 255},
    /*textMuted*/ {150, 143, 131, 255},
    /*panel*/ {244, 241, 234, 255},
    /*start*/ {26, 25, 21, 255},
    /*startLabel*/ {251, 249, 244, 255},
    /*goal*/ {200, 64, 42, 255},          // 朱红
    /*goalLabel*/ {251, 249, 244, 255},
    /*frontier*/ {228, 186, 96, 255},     // 赭黄
    /*visitedNear*/ {226, 216, 194, 255},
    /*visitedFar*/ {242, 238, 228, 255},
    /*current*/ {26, 25, 21, 255},
    /*path*/ {200, 64, 42, 255},
    /*hover*/ {26, 25, 21, 20},
};

constexpr Palette kDark{
    /*background*/ {27, 26, 23, 255},     // #1B1A17
    /*gridLine*/ {44, 42, 38, 255},
    /*wall*/ {214, 207, 192, 255},
    /*text*/ {236, 231, 220, 255},
    /*textMuted*/ {128, 122, 112, 255},
    /*panel*/ {20, 19, 17, 255},
    /*start*/ {236, 231, 220, 255},
    /*startLabel*/ {27, 26, 23, 255},
    /*goal*/ {232, 102, 74, 255},
    /*goalLabel*/ {27, 26, 23, 255},
    /*frontier*/ {176, 136, 58, 255},
    /*visitedNear*/ {72, 64, 50, 255},
    /*visitedFar*/ {38, 36, 32, 255},
    /*current*/ {236, 231, 220, 255},
    /*path*/ {232, 102, 74, 255},
    /*hover*/ {255, 255, 255, 22},
};

struct Runner {
    Demo* demo = nullptr;
    DemoConfig config;
    bool playing = false;
    bool finished = false;
    bool dark = false;
    float speed = 1.0f;
    float accumulator = 0.0f;
    float scale = 1.0f;  // 设备像素比
    Font font{};
    bool fontLoaded = false;
    // 上次通知网页时的状态，避免每帧重复回调
    int notifiedPlaying = -1;
    int notifiedFinished = -1;
    std::string notifiedStatus = "\x01";
};

Runner g;

void loadFont() {
    // 字体按物理像素加载，保证高分屏下文字清晰
    int baseSize = static_cast<int>(std::ceil(28.0f * g.scale));
    int codepoints[96 + 3];
    int n = 0;
    for (int c = 32; c < 127; ++c) codepoints[n++] = c;
    codepoints[n++] = 0x00D7;  // ×
    codepoints[n++] = 0x00B7;  // ·
    codepoints[n++] = 0x2192;  // →
    g.font = LoadFontFromMemory(".ttf", kGaFontData, static_cast<int>(sizeof kGaFontData), baseSize,
                                codepoints, n);
    SetTextureFilter(g.font.texture, TEXTURE_FILTER_BILINEAR);
    g.fontLoaded = g.font.texture.id != 0;
}

void notifyHost() {
#if defined(PLATFORM_WEB)
    int p = g.playing ? 1 : 0;
    int f = g.finished ? 1 : 0;
    if (p != g.notifiedPlaying || f != g.notifiedFinished) {
        g.notifiedPlaying = p;
        g.notifiedFinished = f;
        EM_ASM({ if (Module.onDemoState) Module.onDemoState($0 === 1, $1 === 1); }, p, f);
    }
    std::string s = g.demo->status();
    if (s != g.notifiedStatus) {
        g.notifiedStatus = s;
        EM_ASM({ if (Module.onDemoStatus) Module.onDemoStatus(UTF8ToString($0)); }, s.c_str());
    }
#endif
}

void stepOnce() {
    if (g.finished) return;
    if (!g.demo->step()) {
        g.finished = true;
        g.playing = false;
    }
}

void resetDemo(bool play) {
    g.demo->reset();
    g.finished = false;
    g.accumulator = 0.0f;
    g.playing = play;
}

void togglePlay() {
    if (g.finished) {
        resetDemo(true);
    } else {
        g.playing = !g.playing;
    }
}

void setSpeed(float s) { g.speed = std::clamp(s, 0.1f, 64.0f); }

#if !defined(PLATFORM_WEB)
void handleDesktopKeys() {
    if (IsKeyPressed(KEY_SPACE)) togglePlay();
    if (IsKeyPressed(KEY_N) || IsKeyPressed(KEY_RIGHT)) {
        g.playing = false;
        stepOnce();
    }
    if (IsKeyPressed(KEY_R)) resetDemo(false);
    if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD)) setSpeed(g.speed * 2.0f);
    if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) setSpeed(g.speed * 0.5f);
    if (IsKeyPressed(KEY_T)) g.dark = !g.dark;
}

void drawDesktopHud() {
    const Palette& p = palette();
    char buf[200];
    std::snprintf(buf, sizeof buf, "%s  %s   |   space play  n step  r reset  +/- speed %.3gx  t theme",
                  g.finished ? "DONE" : (g.playing ? "PLAYING" : "PAUSED"), g.demo->status().c_str(), g.speed);
    float h = 24;
    float y = static_cast<float>(g.config.height) - h;
    DrawRectangleRec({0, y, static_cast<float>(g.config.width), h}, p.panel);
    DrawLineEx({0, y}, {static_cast<float>(g.config.width), y}, hairline(), p.gridLine);
    drawText(buf, {10, y + 6}, 11, p.textMuted);
}
#endif

void frame() {
    g.demo->update();

#if !defined(PLATFORM_WEB)
    handleDesktopKeys();
#endif

    if (g.playing) {
        g.accumulator += GetFrameTime() * g.config.stepsPerSecond * g.speed;
        // 防止切回标签页时一次性补太多步
        g.accumulator = std::min(g.accumulator, 2000.0f);
        while (g.accumulator >= 1.0f && !g.finished) {
            g.accumulator -= 1.0f;
            stepOnce();
        }
    }

    BeginDrawing();
    ClearBackground(palette().background);
    Camera2D camera{};
    camera.zoom = g.scale;
    BeginMode2D(camera);
    g.demo->draw();
#if !defined(PLATFORM_WEB)
    drawDesktopHud();
#endif
    EndMode2D();
    EndDrawing();

    notifyHost();
}

}  // namespace

const Palette& palette() { return g.dark ? kDark : kLight; }
bool isDarkTheme() { return g.dark; }

float hairline() { return 1.0f / g.scale; }

void drawText(const char* text, Vector2 pos, float size, Color color) {
    if (g.fontLoaded) {
        DrawTextEx(g.font, text, pos, size, 0.0f, color);
    } else {
        DrawText(text, static_cast<int>(pos.x), static_cast<int>(pos.y), static_cast<int>(size), color);
    }
}

Vector2 measureText(const char* text, float size) {
    if (g.fontLoaded) return MeasureTextEx(g.font, text, size, 0.0f);
    return {static_cast<float>(MeasureText(text, static_cast<int>(size))), size};
}

void drawTextCentered(const char* text, Vector2 center, float size, Color color) {
    Vector2 m = measureText(text, size);
    drawText(text, {std::round((center.x - m.x * 0.5f) * g.scale) / g.scale,
                    std::round((center.y - m.y * 0.5f) * g.scale) / g.scale},
             size, color);
}

void notifyProblemChanged() {
    bool wasFinished = g.finished;
    bool wasPlaying = g.playing;
    resetDemo(wasPlaying);
    if (wasFinished) {
        // 结果已经展示过：直接重新算完，让画面随编辑即时更新
        for (int guard = 0; guard < 1'000'000 && !g.finished; ++guard) stepOnce();
    }
}

int runDemo(Demo& demo, const DemoConfig& config) {
    g.demo = &demo;
    g.config = config;

#if defined(PLATFORM_WEB)
    g.scale = std::clamp(static_cast<float>(emscripten_get_device_pixel_ratio()), 1.0f, 2.0f);
#endif

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(static_cast<int>(config.width * g.scale), static_cast<int>(config.height * g.scale), config.title);
    SetMouseScale(1.0f / g.scale, 1.0f / g.scale);
    loadFont();
    resetDemo(config.autoPlay);

#if defined(PLATFORM_WEB)
    emscripten_set_main_loop(frame, 0, 1);
#else
    SetTargetFPS(60);
    while (!WindowShouldClose()) frame();
    if (g.fontLoaded) UnloadFont(g.font);
    CloseWindow();
#endif
    return 0;
}

}  // namespace ga

// ---- 供网页工具栏调用的接口（Web 端通过 Module._demo_xxx() 调用） ----

GA_EXPORT void demo_toggle_play() { ga::togglePlay(); }

GA_EXPORT void demo_step() {
    ga::g.playing = false;
    ga::stepOnce();
}

GA_EXPORT void demo_reset() { ga::resetDemo(false); }

GA_EXPORT void demo_set_speed(float speed) { ga::setSpeed(speed); }

GA_EXPORT void demo_set_theme(int dark) { ga::g.dark = dark != 0; }

GA_EXPORT void demo_set_visible(int visible) {
#if defined(PLATFORM_WEB)
    // 滚动到视口外时暂停主循环，节省 CPU / 电量
    if (visible) {
        emscripten_resume_main_loop();
    } else {
        emscripten_pause_main_loop();
    }
#else
    (void)visible;
#endif
}
