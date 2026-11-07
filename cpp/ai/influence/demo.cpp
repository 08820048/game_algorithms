// 影响力图演示
//   - 黑色是己方单位，红色是敌方单位，都可以拖动。右侧选择点击场地时做什么：画墙、放置 / 移除单位
//   - 上方切换显示：己方、敌方、势力（差）、紧张度（和）、薄弱度
//   - stamp：按直线距离盖章，穿墙；propagate：每帧传播一轮，绕墙，momentum 控制平滑程度
//   - path：己方侦察兵从 S 到 G，danger 越大越愿意绕路避开敌方影响力。S、G 可以拖动
//   - move：单位在场地里缓慢游走，观察每帧只传播一轮时影响力如何"拖尾"

#include <cmath>
#include <cstdio>
#include <string>

#include "../../foundations/random/random.h"
#include "demo.h"
#include "influence.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::influence;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 52;
constexpr int kCols = 40, kRows = 28;
constexpr float kCell = 15;
constexpr float kOx = 12, kOy = kTop + 12;
constexpr float kPanelX = kOx + kCols * kCell + 14;

struct Unit {
    float x, y;    // 连续坐标（以格子为单位），所在格子取整
    float gx, gy;  // 游走的目标点
    bool enemy;
};

enum class Edit { Walls, Ally, Enemy };

class InfluenceDemo final : public Demo {
public:
    InfluenceDemo() : map_(kCols, kRows), stamper_(10) {
        auto wall = [&](int x0, int y0, int x1, int y1) {
            for (int y = y0; y <= y1; ++y)
                for (int x = x0; x <= x1; ++x) map_.wall[map_.index(x, y)] = 1;
        };
        wall(19, 0, 20, 9), wall(19, 15, 20, 27), wall(8, 18, 14, 18), wall(26, 7, 31, 7), wall(30, 17, 30, 23);
        reset();
    }

    void reset() override {
        units_ = {{5, 5, 5, 5, false}, {9, 12, 9, 12, false}, {4, 21, 4, 21, false},
                  {32, 4, 32, 4, true}, {28, 13, 28, 13, true}, {35, 22, 35, 22, true}};
        rng_ = Pcg32(5);
        for (int f = 0; f < 2; ++f) field_[f].assign(map_.wall.size(), 0);
        recompute(true);
    }

    bool step() override {
        if (move_)
            for (Unit& u : units_) {
                float dx = u.gx - u.x, dy = u.gy - u.y, d = std::sqrt(dx * dx + dy * dy);
                if (d < 0.2f) {  // 换一个附近的目标点
                    for (int k = 0; k < 20; ++k) {
                        float nx = std::clamp(u.x + (rng_.nextFloat() * 2 - 1) * 10, 0.5f, kCols - 0.5f);
                        float ny = std::clamp(u.y + (rng_.nextFloat() * 2 - 1) * 10, 0.5f, kRows - 0.5f);
                        if (map_.open(int(nx), int(ny))) {
                            u.gx = nx, u.gy = ny;
                            break;
                        }
                    }
                } else {
                    float nx = u.x + dx / d * 0.08f, ny = u.y + dy / d * 0.08f;  // 每帧 0.08 格
                    if (map_.open(int(nx), int(ny))) u.x = nx, u.y = ny;
                    else u.gx = u.x, u.gy = u.y;
                }
            }
        recompute(false);
        return true;
    }

    void update() override {
        Vector2 m = GetMousePosition();
        int cx = int(std::floor((m.x - kOx) / kCell)), cy = int(std::floor((m.y - kOy) / kCell));
        bool inMap = cx >= 0 && cy >= 0 && cx < kCols && cy < kRows && !ui::wantsMouse();
        hover_ = inMap ? map_.index(cx, cy) : -1;
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && inMap) {
            float fx = (m.x - kOx) / kCell, fy = (m.y - kOy) / kCell;
            dragUnit_ = -1, dragMarker_ = -1, painting_ = -1;
            for (int i = 0; i < int(units_.size()); ++i)
                if (std::hypot(units_[i].x - fx, units_[i].y - fy) < 0.9f) dragUnit_ = i;
            if (showPath_ && dragUnit_ < 0)
                for (int k = 0; k < 2; ++k)
                    if (markers_[k] == map_.index(cx, cy)) dragMarker_ = k;
            if (dragUnit_ < 0 && dragMarker_ < 0) {
                if (edit_ == Edit::Walls) painting_ = map_.wall[map_.index(cx, cy)] ? 0 : 1;
                else if (map_.open(cx, cy)) {
                    units_.push_back({cx + 0.5f, cy + 0.5f, cx + 0.5f, cy + 0.5f, edit_ == Edit::Enemy});
                    dragUnit_ = int(units_.size()) - 1;
                }
            }
            // 点击已有单位而没有拖动：松开时移除（仅在放置单位的模式下）
            pressX_ = m.x, pressY_ = m.y, removeOnRelease_ = dragUnit_ >= 0 && edit_ != Edit::Walls && dragUnit_ != int(units_.size()) - 1;
        }
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            if (std::hypot(m.x - pressX_, m.y - pressY_) > 4) removeOnRelease_ = false;
            if (inMap && dragUnit_ >= 0 && map_.open(cx, cy)) {
                Unit& u = units_[dragUnit_];
                u.x = u.gx = cx + 0.5f, u.y = u.gy = cy + 0.5f;
            }
            if (inMap && dragMarker_ >= 0 && map_.open(cx, cy)) markers_[dragMarker_] = map_.index(cx, cy);
            if (inMap && painting_ >= 0) {
                bool occupied = markers_[0] == map_.index(cx, cy) || markers_[1] == map_.index(cx, cy);
                for (const Unit& u : units_) occupied |= int(u.x) == cx && int(u.y) == cy;
                if (!occupied) map_.wall[map_.index(cx, cy)] = uint8_t(painting_);
            }
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            if (removeOnRelease_ && dragUnit_ >= 0) units_.erase(units_.begin() + dragUnit_);
            dragUnit_ = dragMarker_ = painting_ = -1;
            removeOnRelease_ = false;
        }
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        const float scale = method_ == 0 ? 1.6f : 1.0f;  // 盖章的值会叠加，用更大的量程
        for (int y = 0; y < kRows; ++y)
            for (int x = 0; x < kCols; ++x) {
                int i = map_.index(x, y);
                Rectangle r{kOx + x * kCell, kOy + y * kCell, kCell, kCell};
                if (map_.wall[i]) {
                    DrawRectangleRec(r, pal.wall);
                    continue;
                }
                float v = combine(field_[0][i], field_[1][i], view_) / scale;
                Color c = view_ == View::Ally ? pal.text
                        : view_ == View::Enemy ? pal.path
                        : view_ == View::Influence ? (v >= 0 ? pal.text : pal.path)
                                                   : pal.frontier;
                float a = std::clamp(std::abs(v) / (view_ == View::Tension || view_ == View::Vulnerability ? 1.5f : 1.0f), 0.0f, 1.0f);
                if (a > 0.003f) DrawRectangleRec(r, Fade(c, 0.08f + 0.72f * a));
            }
        for (int x = 0; x <= kCols; ++x) DrawLineEx({kOx + x * kCell, kOy}, {kOx + x * kCell, kOy + kRows * kCell}, hl, Fade(pal.gridLine, 0.6f));
        for (int y = 0; y <= kRows; ++y) DrawLineEx({kOx, kOy + y * kCell}, {kOx + kCols * kCell, kOy + y * kCell}, hl, Fade(pal.gridLine, 0.6f));
        if (view_ == View::Influence) drawFrontLine();
        if (showPath_) drawPath();
        for (const Unit& u : units_) {
            Vector2 p{kOx + u.x * kCell, kOy + u.y * kCell};
            DrawCircleV(p, 6.5f, u.enemy ? pal.path : pal.text);
            DrawCircleLinesV(p, 6.5f, pal.background);
        }
        DrawRectangleLinesEx({kOx, kOy, kCols * kCell, kRows * kCell}, hl * 1.5f, pal.textMuted);
        drawPanel();
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        const char* views[] = {"ally", "enemy", "influence", "tension", "vulnerability"};
        const float xs[] = {12, 66, 128, 216, 294}, ws[] = {50, 58, 84, 74, 116};
        for (int v = 0; v < 5; ++v)
            if (ui::button({xs[v], 13, ws[v], 26}, views[v], view_ == View(v))) view_ = View(v);
        if (ui::button({436, 13, 66, 26}, "stamp", method_ == 0)) method_ = 0, recompute(true);
        if (ui::button({506, 13, 90, 26}, "propagate", method_ == 1)) method_ = 1, recompute(true);
        ui::toggle({606, 13, 70, 26}, "move", &move_);
        ui::toggle({684, 13, 104, 26}, "path", &showPath_);

        float x = kPanelX, w = kWidth - 12 - kPanelX;
        const char* edits[] = {"walls", "ally", "enemy"};
        for (int e = 0; e < 3; ++e)
            if (ui::button({x + e * (w / 3), kOy + 18, w / 3 - 4, 24}, edits[e], edit_ == Edit(e))) edit_ = Edit(e);
        ui::slider({x, kOy + 76, w, 24}, "mom.", &momentum_, 0.05f, 1, "%.2f", 0.05f);
        if (showPath_) ui::slider({x, kOy + 124, w, 24}, "danger", &danger_, 0, 30, "%.0f", 1);
    }

    std::string status() const override {
        char buf[160];
        int n = std::snprintf(buf, sizeof buf, "%s", method_ == 0 ? "stamp" : "propagate");
        if (method_ == 1) n += std::snprintf(buf + n, sizeof buf - n, " · change %.3f", change_);
        if (showPath_ && !path_.cells.empty())
            std::snprintf(buf + n, sizeof buf - n, " · path %.1f · exposure %.2f", pathLength_, pathExposure_);
        return buf;
    }

private:
    void recompute(bool full) {
        std::vector<Source> s[2];
        for (const Unit& u : units_) s[u.enemy].push_back({int(u.x), int(u.y), 1});
        for (int f = 0; f < 2; ++f) {
            if (method_ == 0) stamper_.stamp(map_, s[f], field_[f]);
            else if (full) propagate(map_, s[f], decay_, field_[f]);
            else {
                Field seeds;
                seed(map_, s[f], seeds);
                float c = propagateStep(map_, seeds, decay_, momentum_, field_[f], scratch_);
                change_ = f == 0 ? c : std::max(change_, c);
            }
            for (size_t i = 0; i < map_.wall.size(); ++i)
                if (map_.wall[i]) field_[f][i] = 0;
        }
        if (showPath_) {
            path_ = map_.wall[markers_[0]] || map_.wall[markers_[1]] ? PathResult{} : safePath(map_, field_[1], markers_[0], markers_[1], danger_);
            pathLength_ = pathExposure_ = 0;
            for (size_t k = 1; k < path_.cells.size(); ++k) {
                int a = path_.cells[k - 1], b = path_.cells[k];
                float step = (a % kCols != b % kCols && a / kCols != b / kCols) ? std::sqrt(2.0f) : 1.0f;
                pathLength_ += step, pathExposure_ += step * field_[1][b];
            }
        }
    }

    Vector2 center(int i) const { return {kOx + (i % kCols + 0.5f) * kCell, kOy + (i / kCols + 0.5f) * kCell}; }

    // 前线：势力的符号在相邻两格之间改变的地方
    void drawFrontLine() const {
        const Palette& pal = palette();
        auto sign = [&](int i) {
            float v = field_[0][i] - field_[1][i];
            return map_.wall[i] || std::abs(v) < 1e-4f ? 0 : v > 0 ? 1 : -1;
        };
        for (int y = 0; y < kRows; ++y)
            for (int x = 0; x < kCols; ++x) {
                int s = sign(map_.index(x, y));
                if (s <= 0) continue;
                float px = kOx + x * kCell, py = kOy + y * kCell;
                if (x + 1 < kCols && sign(map_.index(x + 1, y)) < 0) DrawLineEx({px + kCell, py}, {px + kCell, py + kCell}, 3.0f, pal.background);
                if (x > 0 && sign(map_.index(x - 1, y)) < 0) DrawLineEx({px, py}, {px, py + kCell}, 3.0f, pal.background);
                if (y + 1 < kRows && sign(map_.index(x, y + 1)) < 0) DrawLineEx({px, py + kCell}, {px + kCell, py + kCell}, 3.0f, pal.background);
                if (y > 0 && sign(map_.index(x, y - 1)) < 0) DrawLineEx({px, py}, {px + kCell, py}, 3.0f, pal.background);
            }
    }

    void drawPath() const {
        const Palette& pal = palette();
        for (size_t k = 1; k < path_.cells.size(); ++k) DrawLineEx(center(path_.cells[k - 1]), center(path_.cells[k]), 3, pal.frontier);
        const char* labels[] = {"S", "G"};
        for (int k = 0; k < 2; ++k) {
            Vector2 c = center(markers_[k]);
            DrawRectangleRec({c.x - 7, c.y - 7, 14, 14}, pal.frontier);
            drawTextCentered(labels[k], c, 11, pal.text);
        }
    }

    void drawPanel() const {
        const Palette& pal = palette();
        float x = kPanelX, y = kOy;
        drawText("click on the map", {x, y}, 11, pal.textMuted);
        drawText(method_ == 1 ? "momentum (propagate)" : "momentum (propagate only)", {x, y + 58}, 11, pal.textMuted);
        if (showPath_) drawText("path: avoid enemy", {x, y + 106}, 11, pal.textMuted);
        y += 170;
        DrawLineEx({x, y - 8}, {kWidth - 12.0f, y - 8}, hairline(), pal.gridLine);
        if (hover_ >= 0 && !map_.wall[hover_]) {
            float a = field_[0][hover_], e = field_[1][hover_];
            char buf[64];
            std::snprintf(buf, sizeof buf, "cell %d, %d", hover_ % kCols, hover_ / kCols);
            drawText(buf, {x, y}, 11, pal.text);
            const char* names[] = {"ally", "enemy", "influence", "tension", "vulnerab."};
            for (int v = 0; v < 5; ++v) {
                std::snprintf(buf, sizeof buf, "%-10s %6.2f", names[v], combine(a, e, View(v)));
                drawText(buf, {x, y + 18 + v * 16.0f}, 11, View(v) == view_ ? pal.text : pal.textMuted);
            }
        } else {
            drawText("hover a cell to read", {x, y}, 11, pal.textMuted);
            drawText("its values", {x, y + 16}, 11, pal.textMuted);
        }
        y += 112;
        const char* help[] = {"drag units to move them", "ally / enemy: click a", "unit to remove it",
                              view_ == View::Influence ? "blank seam: front line" : ""};
        for (const char* h : help) drawText(h, {x, y}, 10, pal.textMuted), y += 15;
    }

    Map map_;
    Stamper stamper_;
    Decay decay_{0.85f, 0.03f};
    Field field_[2], scratch_;
    std::vector<Unit> units_;
    Pcg32 rng_{5};
    View view_ = View::Influence;
    int method_ = 1;
    Edit edit_ = Edit::Walls;
    bool move_ = false, showPath_ = true, removeOnRelease_ = false;
    float momentum_ = 1, danger_ = 10, change_ = 0;
    int markers_[2] = {map_.index(1, 26), map_.index(38, 1)};
    PathResult path_;
    float pathLength_ = 0, pathExposure_ = 0;
    int hover_ = -1, dragUnit_ = -1, dragMarker_ = -1, painting_ = -1;
    float pressX_ = 0, pressY_ = 0;
};

}  // namespace

int main() {
    static InfluenceDemo demo;
    ga::DemoConfig config;
    config.title = "Influence map";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 30.0f;
    return ga::runDemo(demo, config);
}
