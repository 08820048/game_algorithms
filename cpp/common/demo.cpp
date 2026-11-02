#include "demo.h"

#include <algorithm>
#include <cstdio>

#if defined(PLATFORM_WEB)
#include <emscripten/emscripten.h>
#define GA_EXPORT extern "C" EMSCRIPTEN_KEEPALIVE
#else
#define GA_EXPORT extern "C"
#endif

namespace ga {
namespace {

constexpr Palette kLight{
    /*background*/ {250, 250, 247, 255},
    /*gridLine*/ {226, 226, 220, 255},
    /*wall*/ {58, 63, 74, 255},
    /*text*/ {32, 34, 40, 255},
    /*textMuted*/ {120, 122, 130, 255},
    /*panel*/ {255, 255, 255, 230},
    /*start*/ {34, 160, 107, 255},
    /*goal*/ {219, 68, 85, 255},
    /*frontier*/ {255, 196, 87, 255},
    /*visitedNear*/ {178, 214, 247, 255},
    /*visitedFar*/ {214, 226, 240, 255},
    /*current*/ {232, 120, 30, 255},
    /*path*/ {219, 68, 85, 255},
    /*hover*/ {0, 0, 0, 40},
};

constexpr Palette kDark{
    /*background*/ {23, 25, 31, 255},
    /*gridLine*/ {40, 43, 52, 255},
    /*wall*/ {150, 156, 170, 255},
    /*text*/ {230, 232, 238, 255},
    /*textMuted*/ {140, 145, 158, 255},
    /*panel*/ {33, 36, 44, 230},
    /*start*/ {64, 199, 140, 255},
    /*goal*/ {255, 107, 122, 255},
    /*frontier*/ {226, 168, 64, 255},
    /*visitedNear*/ {44, 86, 140, 255},
    /*visitedFar*/ {36, 52, 76, 255},
    /*current*/ {255, 150, 60, 255},
    /*path*/ {255, 107, 122, 255},
    /*hover*/ {255, 255, 255, 40},
};

struct Runner {
    Demo* demo = nullptr;
    DemoConfig config;
    bool playing = false;
    bool finished = false;
    bool dark = false;
    float speed = 1.0f;
    float accumulator = 0.0f;
    // 上次通知网页时的状态，避免每帧重复回调
    int notifiedPlaying = -1;
    int notifiedFinished = -1;
};

Runner g;

void notifyHost() {
#if defined(PLATFORM_WEB)
    int p = g.playing ? 1 : 0;
    int f = g.finished ? 1 : 0;
    if (p == g.notifiedPlaying && f == g.notifiedFinished) return;
    g.notifiedPlaying = p;
    g.notifiedFinished = f;
    EM_ASM({ if (Module.onDemoState) Module.onDemoState($0 === 1, $1 === 1); }, p, f);
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
    char buf[160];
    std::snprintf(buf, sizeof buf, "%s  |  Space play/pause  N step  R reset  +/- speed (%.2gx)  T theme",
                  g.finished ? "DONE" : (g.playing ? "PLAYING" : "PAUSED"), g.speed);
    int h = 22;
    DrawRectangle(0, g.config.height - h, g.config.width, h, p.panel);
    DrawText(buf, 8, g.config.height - h + 6, 10, p.textMuted);
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
    g.demo->draw();
#if !defined(PLATFORM_WEB)
    drawDesktopHud();
#endif
    EndDrawing();

    notifyHost();
}

}  // namespace

const Palette& palette() { return g.dark ? kDark : kLight; }
bool isDarkTheme() { return g.dark; }

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

    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(config.width, config.height, config.title);
    resetDemo(config.autoPlay);

#if defined(PLATFORM_WEB)
    emscripten_set_main_loop(frame, 0, 1);
#else
    SetTargetFPS(60);
    while (!WindowShouldClose()) frame();
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
