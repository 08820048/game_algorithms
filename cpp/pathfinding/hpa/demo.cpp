// HPA* 演示：地图被分成 5×5 的簇
//   - 粗线：簇的边界；圆点：过渡点；赭黄短线：跨越边界的簇间边
//   - graph：显示簇内边（同一簇内过渡点之间的连线）
//   - 单步：① 把 S、G 接入所在簇的过渡点 ② 在抽象图上找到路径（朱红虚线）③ 逐段换成格子路径
//   - 编辑地形后，抽象图立即重建

#include <cstdio>
#include <string>

#include "../astar_opt/pathfinder.h"
#include "../terrain_map.h"
#include "hpa.h"
#include "raymath.h"

namespace {

using namespace ga;
using namespace ga::terrain;

constexpr int kCluster = 5;

class HpaDemo final : public Demo {
public:
    HpaDemo() : grid_(kCols, kRows) {
        wallRect(grid_, 0, 7, 20, 7);
        grid_.setWall({3, 7}, false);
        grid_.setWall({12, 7}, false);
        wallRect(grid_, 10, 0, 10, 6);
        grid_.setWall({10, 2}, false);
        wallRect(grid_, 17, 8, 17, 13);
        grid_.setWall({17, 11}, false);
        fillRect(grid_, 13, 1, 16, 4, kSwampCost);
        fillRect(grid_, 4, 9, 7, 12, kForestCost);
        fillRect(grid_, 20, 2, 23, 5, kForestCost);
        reset();
    }

    void reset() override {
        hpa_.build(grid_, kCluster);
        HpaStats st;
        path_ = hpa_.findPath(start_, goal_, &st, &abstract_);
        stats_ = st;
        Pathfinder pf(grid_);
        auto best = pf.find(start_, goal_);
        optimal_ = best.empty() ? -1 : pathCost(grid_, best);
        // 抽象路径上每个节点在完整路径中的位置，用来逐段显示
        marks_.clear();
        size_t j = 0;
        for (Cell c : abstract_) {
            while (j < path_.size() && !(path_[j] == c)) ++j;
            marks_.push_back(j);
        }
        phase_ = 0;
    }

    bool step() override {
        int last = path_.empty() ? 1 : 1 + static_cast<int>(abstract_.size());
        if (phase_ < last) ++phase_;
        return phase_ < last;
    }

    void update() override {
        if (editor_.update(grid_, start_, goal_)) notifyProblemChanged();
    }

    void draw() const override {
        const Palette& p = palette();
        const auto& nodes = hpa_.nodes();

        // 起点、终点所在的簇（插入阶段高亮）
        if (phase_ >= 1) {
            for (int c : {hpa_.clusterOf(start_), hpa_.clusterOf(goal_)}) {
                Rect r = hpa_.clusterRectOf(c);
                DrawRectangleRec({float(r.x0 * kCell), kTop + r.y0 * kCell, float(r.width() * kCell),
                                  float((r.y1 - r.y0) * kCell)},
                                 p.visitedFar);
            }
        }
        drawTerrain(grid_);
        drawGridAndWalls(grid_);

        // 簇边界
        Color border = Fade(p.text, 0.55f);
        for (int x = kCluster; x < kCols; x += kCluster) {
            DrawLineEx({float(x * kCell), kTop}, {float(x * kCell), float(kHeight)}, 2, border);
        }
        for (int y = kCluster; y < kRows; y += kCluster) {
            DrawLineEx({0, kTop + y * kCell}, {float(kWidth), kTop + y * kCell}, 2, border);
        }

        // 抽象图：簇内边（可选）、簇间边、节点
        if (showGraph_) {
            for (const auto& n : nodes) {
                for (const auto& e : n.edges) {
                    if (e.path >= 0) DrawLineEx(center(n.cell), center(nodes[e.to].cell), 1, Fade(p.text, 0.22f));
                }
            }
        }
        for (const auto& n : nodes) {
            for (const auto& e : n.edges) {
                if (e.path < 0) DrawLineEx(center(n.cell), center(nodes[e.to].cell), 4, p.frontier);
            }
        }
        for (const auto& n : nodes) {
            DrawCircleV(center(n.cell), 5, p.background);
            DrawCircleLinesV(center(n.cell), 5, p.text);
        }

        // ① 插入：S、G 与所在簇过渡点之间的连线
        if (phase_ == 1) {
            for (const auto& n : nodes) {
                if (n.cluster == hpa_.clusterOf(start_)) drawDashedLine(center(start_), center(n.cell), 1.5f, p.text, 4, 3);
                if (n.cluster == hpa_.clusterOf(goal_)) drawDashedLine(center(n.cell), center(goal_), 1.5f, p.text, 4, 3);
            }
        }
        // ② 抽象路径
        if (phase_ >= 2) {
            for (size_t i = 1; i < abstract_.size(); ++i) {
                drawDashedLine(center(abstract_[i - 1]), center(abstract_[i]), 2, p.path, 7, 4);
            }
        }
        // ③ 逐段细化
        if (phase_ >= 3 && !path_.empty()) {
            size_t segs = static_cast<size_t>(phase_ - 2);
            size_t end = segs < marks_.size() ? marks_[segs] : path_.size() - 1;
            std::vector<Cell> prefix(path_.begin(), path_.begin() + static_cast<long>(end) + 1);
            drawPath(prefix, p.path);
        }
        drawOverlay(editor_.hover(), start_, goal_);

        char buf[48];
        float x = kWidth - 16.0f;
        if (phase_ >= 1 + static_cast<int>(abstract_.size()) && !path_.empty()) {
            std::snprintf(buf, sizeof buf, "hpa* %d", pathCost(grid_, path_));
            x = drawRightLabel(x, buf, p.path);
            std::snprintf(buf, sizeof buf, "optimal %d", optimal_);
            drawRightLabel(x, buf, p.text);
        }
    }

    void drawUi() override {
        float x = editor_.drawBrushes(16) + 20;
        ui::toggle({x, 13, 74, 26}, "graph", &showGraph_);
    }

    std::string status() const override {
        char buf[128];
        int segments = static_cast<int>(abstract_.size()) - 1;
        if (path_.empty()) {
            std::snprintf(buf, sizeof buf, "nodes %zu · unreachable", hpa_.nodes().size());
        } else if (phase_ == 0) {
            std::snprintf(buf, sizeof buf, "clusters %d · nodes %zu", hpa_.clusterCount(), hpa_.nodes().size());
        } else if (phase_ == 1) {
            std::snprintf(buf, sizeof buf, "insert S, G into their clusters");
        } else if (phase_ == 2) {
            std::snprintf(buf, sizeof buf, "abstract path · expanded %d nodes", stats_.abstractExpanded);
        } else {
            std::snprintf(buf, sizeof buf, "refine %d / %d segments", std::min(phase_ - 2, segments), segments);
        }
        return buf;
    }

private:
    CostGrid grid_;
    HpaGraph hpa_;
    Cell start_{1, 1};
    Cell goal_{23, 12};
    Editor editor_;
    std::vector<Cell> path_, abstract_;
    std::vector<size_t> marks_;
    HpaStats stats_;
    int optimal_ = 0;
    int phase_ = 0;
    bool showGraph_ = false;
};

}  // namespace

int main() {
    static HpaDemo demo;
    ga::DemoConfig config;
    config.title = "HPA* - Hierarchical Pathfinding";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 1.5f;
    return ga::runDemo(demo, config);
}
