#pragma once

// 所有演示共享的小框架：
//   - 统一的播放 / 暂停 / 单步 / 重置 / 调速逻辑
//   - 桌面端：键盘控制 + 屏幕提示
//   - Web 端：由网页工具栏通过导出的 C 函数控制（见 cpp/web/shell.html）
//   - 亮 / 暗两套"技术手稿"配色，跟随网站主题切换
//   - 高分屏下按设备像素比渲染；演示代码始终使用逻辑坐标
//
// 一个演示只需继承 ga::Demo，实现 reset / step / draw，然后调用 ga::runDemo。

#include <string>

#include "raylib.h"

namespace ga {

struct Palette {
    Color background;   // 画布底色（纸）
    Color gridLine;     // 网格细线
    Color wall;         // 障碍（墨）
    Color text;
    Color textMuted;
    Color panel;        // 桌面端提示栏
    Color start;        // 起点（墨色圆）
    Color startLabel;
    Color goal;         // 终点（朱红圆）
    Color goalLabel;
    Color frontier;     // 边界 / 待处理
    Color visitedNear;  // 距离起点近的已访问格子
    Color visitedFar;   // 距离起点远的已访问格子
    Color current;      // 当前正在处理的元素
    Color path;         // 结果路径（朱红）
    Color hover;
};

const Palette& palette();
bool isDarkTheme();

// 等宽字体文字（JetBrains Mono），比 raylib 默认像素字体清晰得多
void drawText(const char* text, Vector2 pos, float size, Color color);
void drawTextCentered(const char* text, Vector2 center, float size, Color color);
Vector2 measureText(const char* text, float size);

// 一个物理像素对应的逻辑宽度，用于绘制真正的"细线"
float hairline();

class Demo {
public:
    virtual ~Demo() = default;
    // 把算法恢复到初始状态（保留用户对地图的编辑）
    virtual void reset() = 0;
    // 执行算法的一步；算法结束时返回 false
    virtual bool step() = 0;
    // 每帧调用一次，处理鼠标编辑等交互
    virtual void update() {}
    // 绘制当前状态（已处于 BeginDrawing/EndDrawing 之间，坐标为逻辑坐标）
    virtual void draw() const = 0;
    // 一行状态文字，显示在网页工具栏 / 桌面提示栏，如 "expanded 12 · frontier 4"
    virtual std::string status() const { return {}; }
};

struct DemoConfig {
    const char* title = "Game Algorithms Demo";
    int width = 800;   // 逻辑尺寸
    int height = 480;
    float stepsPerSecond = 20.0f;  // 1x 速度下每秒执行的步数
    bool autoPlay = true;
};

// 当用户修改了问题本身（比如画墙、拖动起点），演示调用此函数：
// 算法会被重置；如果之前已经运行结束，则立即重新跑完，让结果"实时"跟随编辑。
void notifyProblemChanged();

int runDemo(Demo& demo, const DemoConfig& config);

}  // namespace ga
