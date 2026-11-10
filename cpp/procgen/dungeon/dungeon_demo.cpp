// 三种地牢生成器与它们的"可玩性"指标
//   - walk：随机游走，coverage 是地面比例；restart 打开后每 20 步跳到一个随机的已有地面重新开始
//   - bsp：见图 4.4a；closest 决定连接方式
//   - rooms：随机放置房间（300 次尝试），最小生成树连通，loops 是额外加回的边数占房间数的比例
//   - distance：从直径的一端出发的 BFS 距离；红线是最长的最短路（直径）
//   - dead ends：只有一个地面邻居的格子
//   - 右侧：精确直径（每个格子做一次 BFS）与两次 BFS 的结果；两者不同时，两次 BFS 找到的端点用空心圆标出

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "demo.h"
#include "dungeon.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::dungeon;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80;
constexpr int kW = 80, kH = 50;
constexpr float kCell = 7;
constexpr float kMx = 16, kMy = kTop + 10;
constexpr float kPx = 598;

class DungeonDemo final : public Demo {
public:
    void reset() override {}
    void update() override {
        if (dirty_) generate();  // 不在构造函数里生成：那时窗口还没创建，GetTime 不可用
    }
    bool step() override { return false; }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        DrawRectangleRec({kMx, kMy, kW * kCell, kH * kCell}, pal.wall);
        for (int y = 0; y < kH; ++y)
            for (int x = 0; x < kW; ++x) {
                int i = y * kW + x;
                uint8_t t = map_.t[i];
                if (t == Wall) continue;
                Color c = t == Room ? pal.background : pal.visitedNear;
                if (showDist_ && dist_[i] >= 0) {  // 近处赭黄，远处纸色
                    float f = float(dist_[i]) / std::max(1, diam_.length);
                    c = ColorLerp(pal.frontier, pal.visitedFar, f);
                }
                DrawRectangleRec({kMx + x * kCell, kMy + y * kCell, kCell, kCell}, c);
            }
        if (showDead_)
            for (int y = 0; y < kH; ++y)
                for (int x = 0; x < kW; ++x) {
                    if (!map_.floor(x, y)) continue;
                    int k = 0;
                    for (int d = 0; d < 4; ++d) k += map_.floor(x + kDx4[d], y + kDy4[d]);
                    if (k == 1) DrawRectangleLinesEx({kMx + x * kCell - 1, kMy + y * kCell - 1, kCell + 2, kCell + 2}, 1.6f, pal.path);
                }
        // 直径：沿 BFS 距离递减的方向从 b 走回 a
        if (showDist_ && diam_.a >= 0) {
            int c = diam_.b;
            while (dist_[c] > 0) {
                int x = c % kW, y = c / kW, next = c;
                for (int d = 0; d < 4; ++d) {
                    int nx = x + kDx4[d], ny = y + kDy4[d];
                    if (map_.floor(nx, ny) && dist_[ny * kW + nx] == dist_[c] - 1) {
                        next = ny * kW + nx;
                        break;
                    }
                }
                DrawLineEx(center(c), center(next), 2.0f, pal.path);
                c = next;
            }
            DrawCircleV(center(diam_.a), 4.5f, pal.path);
            DrawCircleV(center(diam_.b), 4.5f, pal.path);
            if (sweep_.length != diam_.length) {
                DrawCircleLinesV(center(sweep_.a), 7, pal.text);
                DrawCircleLinesV(center(sweep_.b), 7, pal.text);
            }
        }

        char buf[128];
        float y = kMy;
        auto line = [&](const char* s, Color c, float size = 12) {
            drawText(s, {kPx, y}, size, c);
            y += size + 7;
        };
        std::snprintf(buf, sizeof buf, "floor %.1f%%", 100.0 * map_.floorCount() / ((kW - 2) * (kH - 2)));
        line(buf, pal.text);
        if (gen_ > 0) {
            std::snprintf(buf, sizeof buf, "rooms %d  corridors %d", rooms_, links_);
            line(buf, pal.text);
        }
        std::snprintf(buf, sizeof buf, "dead ends %d", dead_);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "detour ratio %.2f", detour_);
        line(buf, pal.text);
        y += 8;
        line("diameter (longest shortest path)", pal.textMuted, 11);
        std::snprintf(buf, sizeof buf, "exact        %d  (%.1f ms)", diam_.length, exactMs_);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "double sweep %d", sweep_.length);
        line(buf, sweep_.length == diam_.length ? pal.text : pal.path);
        if (sweep_.length != diam_.length) line("open circles: sweep endpoints", pal.textMuted, 11);
        if (gen_ == 0) {
            y += 8;
            std::snprintf(buf, sizeof buf, "steps %lld  (%.1f per cell)", steps_, double(steps_) / std::max(1, map_.floorCount()));
            line(buf, pal.textMuted, 11);
        }
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        const char* names[] = {"walk", "bsp", "rooms"};
        for (int k = 0; k < 3; ++k)
            if (ui::button({12.0f + k * 70, 10, 66, 26}, names[k], gen_ == k)) gen_ = k, ch = true;
        ch |= ui::slider({232, 10, 160, 26}, "seed", &seed_, 1, 50, "%.0f", 1);
        if (gen_ == 0) {
            ch |= ui::slider({12, 44, 220, 26}, "coverage", &coverage_, 0.1f, 0.6f, "%.2f", 0.05f);
            ch |= ui::toggle({244, 44, 96, 26}, "restart", &restart_);
        } else if (gen_ == 1) {
            ch |= ui::toggle({12, 44, 96, 26}, "closest", &closest_);
        } else {
            ch |= ui::slider({12, 44, 220, 26}, "loops", &loops_, 0.0f, 1.0f, "%.2f", 0.05f);
        }
        ui::toggle({560, 44, 104, 26}, "distance", &showDist_);
        ui::toggle({672, 44, 116, 26}, "dead ends", &showDead_);
        if (ch) dirty_ = true;
    }

    std::string status() const override {
        const char* names[] = {"walk", "bsp", "rooms"};
        char buf[128];
        std::snprintf(buf, sizeof buf, "%s · dead ends %d · detour %.2f · diameter %d", names[gen_], dead_, detour_, diam_.length);
        return buf;
    }

private:
    static Vector2 center(int c) { return {kMx + (c % kW + 0.5f) * kCell, kMy + (c / kW + 0.5f) * kCell}; }

    void generate() {
        dirty_ = false;
        map_ = Map(kW, kH);
        Pcg32 rng{uint64_t(seed_)};
        rooms_ = links_ = 0, steps_ = 0;
        if (gen_ == 0) {
            WalkParams p;
            p.coverage = coverage_, p.restartEvery = restart_ ? 20 : 0;
            steps_ = randomWalk(map_, rng, p);
        } else if (gen_ == 1) {
            BspParams p;
            p.closest = closest_;
            BspResult r = generateBsp(map_, rng, p);
            rooms_ = int(r.leaves.size()), links_ = int(r.corridors.size());
        } else {
            RoomsParams p;
            p.loops = loops_;
            RoomsResult r = generateRooms(map_, rng, p);
            rooms_ = int(r.rooms.size()), links_ = int(r.tree.size() + r.extra.size());
        }
        dead_ = deadEnds(map_);
        Pcg32 r2(1);
        detour_ = float(detourRatio(map_, r2, 30, 30, 10));
        double t0 = GetTime();
        diam_ = exactDiameter(map_);
        exactMs_ = float((GetTime() - t0) * 1000);
        // 两次 BFS 从一个固定的"随机"地面格子出发
        std::vector<int> cells;
        for (int i = 0; i < kW * kH; ++i)
            if (map_.t[i] != Wall) cells.push_back(i);
        sweep_ = doubleSweep(map_, cells[r2.below(uint32_t(cells.size()))]);
        dist_ = bfs(map_, diam_.a);
    }

    int gen_ = 2;
    float seed_ = 4, coverage_ = 0.35f, loops_ = 0.15f;
    bool dirty_ = true, restart_ = false, closest_ = true, showDist_ = true, showDead_ = true;
    Map map_{kW, kH};
    std::vector<int> dist_;
    Diameter diam_, sweep_;
    int rooms_ = 0, links_ = 0, dead_ = 0;
    long long steps_ = 0;
    float detour_ = 0, exactMs_ = 0;
};

}  // namespace

int main() {
    static DungeonDemo demo;
    ga::DemoConfig config;
    config.title = "Dungeon generators";
    config.width = kWidth;
    config.height = kHeight;
    config.autoPlay = false;
    return ga::runDemo(demo, config);
}
