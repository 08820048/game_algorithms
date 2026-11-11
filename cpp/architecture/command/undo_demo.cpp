// 撤销与重做：瓦片编辑器
//   - 在网格上拖动画画（四种颜色），undo / redo 撤销、重做
//   - 右侧是历史：每条是一条命令（这一笔改了几个格子、占多少字节）。箭头是当前位置，下面灰色的是可以重做的
//   - 撤销之后再画一笔：灰色的重做分支被丢弃
//   - merge 关掉：每个格子一条命令，一笔要撤销几十次
//   - 底部：同样的历史，三种做法各要多少内存（命令记旧值 / 每步完整快照 / 只存命令 + 每 8 步一个检查点）

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "command.h"
#include "demo.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::command;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80, kX0 = 12, kY0 = 92, kCell = 14;
constexpr int kCols = 38, kRows = 24;

class UndoDemo final : public Demo {
public:
    UndoDemo() { reset(); }
    void reset() override {
        grid_ = Grid(kCols, kRows);
        stack_ = UndoStack{};
        // 预先画几笔，再撤销两次，让历史里有可以重做的条目
        auto line = [&](int x0, int y0, int x1, int y1, uint8_t v) {
            std::vector<Edit> e;
            int n = std::max(std::abs(x1 - x0), std::abs(y1 - y0));
            for (int i = 0; i <= n; ++i) {
                int x = x0 + (x1 - x0) * i / std::max(n, 1), y = y0 + (y1 - y0) * i / std::max(n, 1);
                e.push_back({uint32_t(y * kCols + x), v});
            }
            commit(e);
        };
        line(3, 3, 20, 3, 1), line(3, 3, 3, 18, 2), line(6, 16, 30, 6, 3), line(10, 20, 34, 20, 4), line(34, 4, 34, 20, 1);
        stack_.undo(grid_), stack_.undo(grid_);
    }
    bool step() override { return true; }
    void update() override {
        Vector2 m = GetMousePosition();
        int cx = int((m.x - kX0) / kCell), cy = int((m.y - kY0) / kCell);
        bool inside = m.x >= kX0 && m.y >= kY0 && cx < kCols && cy < kRows && !ui::wantsMouse();
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && inside) {
            uint32_t c = uint32_t(cy * kCols + cx);
            if (stroke_.empty()) before_ = grid_.cells;  // 这一笔开始前的样子
            if (stroke_.empty() || stroke_.back().cell != c) {
                stroke_.push_back({c, uint8_t(color_ + 1)});
                if (merge_) grid_.cells[c] = uint8_t(color_ + 1);  // 先画上，松开时作为一条命令入栈
                else commit({stroke_.back()});                      // 每个格子一条命令
            }
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && !stroke_.empty()) {
            if (merge_) grid_.cells = before_, commit(stroke_);  // 恢复后整笔执行：命令记下的是真正的旧值
            stroke_.clear();
        }
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        const Color colors[5] = {pal.background, pal.text, pal.path, Color{60, 130, 220, 255}, Color{60, 150, 90, 255}};
        for (int y = 0; y < kRows; ++y)
            for (int x = 0; x < kCols; ++x) {
                uint8_t v = grid_.cells[size_t(y * kCols + x)];
                Rectangle r{kX0 + x * kCell, kY0 + y * kCell, kCell, kCell};
                if (v) DrawRectangleRec(r, colors[v]);
                DrawRectangleLinesEx(r, hl, Fade(pal.gridLine, 0.5f));
            }
        // 历史
        const float hx = kX0 + kCols * kCell + 16;
        drawText("history", {hx, kY0 - 2}, 11, pal.textMuted);
        const auto& h = stack_.history();
        size_t first = h.size() > 18 ? h.size() - 18 : 0;
        for (size_t i = first; i < h.size(); ++i) {
            float y = kY0 + 16 + float(i - first) * 16;
            bool done = i < stack_.size();
            char b[64];
            std::snprintf(b, sizeof b, "#%zu  %3zu cells  %4zu B", i + 1, h[i].cells.size(), h[i].bytes());
            drawText(b, {hx + 14, y}, 10, done ? pal.text : Fade(pal.textMuted, 0.6f));
            if (i + 1 == stack_.size()) drawText(">", {hx, y}, 10, pal.path);
        }
        if (stack_.size() == 0) drawText(">", {hx, kY0 + 2}, 10, pal.path);
        // 内存
        size_t cmd = stack_.bytes(), snap = (stack_.total() + 1) * grid_.cells.size();
        size_t replay = 0;
        for (const auto& s : h) replay += s.cells.size() * sizeof(Edit);
        replay += (stack_.total() / 8 + 1) * grid_.cells.size();
        char buf[200];
        std::snprintf(buf, sizeof buf, "memory for this history:  commands %zu B   snapshots %zu B   commands + checkpoint/8 %zu B", cmd, snap, replay);
        drawText(buf, {12, kHeight - 18.0f}, 11, pal.textMuted);
        std::snprintf(buf, sizeof buf, "%zu commands, %zu can be redone", stack_.size(), stack_.total() - stack_.size());
        drawText(buf, {12, kTop + 6}, 11, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        if (ui::button({12, 10, 70, 26}, "undo")) stack_.undo(grid_);
        if (ui::button({86, 10, 70, 26}, "redo")) stack_.redo(grid_);
        const char* names[4] = {"ink", "red", "blue", "green"};
        for (int i = 0; i < 4; ++i)
            if (ui::button({168.0f + i * 64, 10, 60, 26}, names[i], color_ == i)) color_ = i;
        ui::toggle({430, 10, 80, 26}, "merge", &merge_);
    }

    std::string status() const override {
        char buf[64];
        std::snprintf(buf, sizeof buf, "%zu commands · %zu B", stack_.size(), stack_.bytes());
        return buf;
    }

private:
    void commit(const std::vector<Edit>& e) { stack_.apply(grid_, e, true); }
    Grid grid_{kCols, kRows};
    UndoStack stack_;
    std::vector<Edit> stroke_;
    std::vector<uint8_t> before_;
    int color_ = 1;
    bool merge_ = true;
};

}  // namespace

int main() {
    static UndoDemo demo;
    ga::DemoConfig config;
    config.title = "Undo and redo";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
