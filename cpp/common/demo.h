#pragma once

// 所有演示共享的小框架：
//   - 统一的播放 / 暂停 / 单步 / 重置 / 调速逻辑
//   - 桌面端：键盘控制 + 屏幕提示
//   - Web 端：由网页工具栏通过导出的 C 函数控制（见 cpp/web/shell.html）
//   - 亮 / 暗两套配色，跟随网站主题切换
//
// 一个演示只需继承 ga::Demo，实现 reset / step / draw，然后调用 ga::runDemo。

#include "raylib.h"

namespace ga {

struct Palette {
    Color background;
    Color gridLine;
    Color wall;
    Color text;
    Color textMuted;
    Color panel;
    Color start;
    Color goal;
    Color frontier;
    Color visitedNear;  // 距离起点近的已访问格子
    Color visitedFar;   // 距离起点远的已访问格子
    Color current;
    Color path;
    Color hover;
};

const Palette& palette();
bool isDarkTheme();

class Demo {
public:
    virtual ~Demo() = default;
    // 把算法恢复到初始状态（保留用户对地图的编辑）
    virtual void reset() = 0;
    // 执行算法的一步；算法结束时返回 false
    virtual bool step() = 0;
    // 每帧调用一次，处理鼠标编辑等交互
    virtual void update() {}
    // 绘制当前状态（已处于 BeginDrawing/EndDrawing 之间）
    virtual void draw() const = 0;
};

struct DemoConfig {
    const char* title = "Game Algorithms Demo";
    int width = 800;
    int height = 480;
    float stepsPerSecond = 20.0f;  // 1x 速度下每秒执行的步数
    bool autoPlay = true;
};

// 当用户修改了问题本身（比如画墙、拖动起点），演示调用此函数：
// 算法会被重置；如果之前已经运行结束，则立即重新跑完，让结果"实时"跟随编辑。
void notifyProblemChanged();

int runDemo(Demo& demo, const DemoConfig& config);

}  // namespace ga
