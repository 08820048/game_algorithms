// 四叉树演示
//   - 盒子在场地里运动，每一步重建四叉树；细线是节点的分割
//   - 严格树：跨越分割线的物体留在上层节点（黄色描边），它们要和整棵子树里的物体比较
//   - 松散树：节点边界向外扩大一倍，物体按中心放进孩子，几乎所有物体都在叶子里
//   - 鼠标处是一个区域查询：浅色的是访问过的节点，黄色描边的物体做了盒子测试，红色的是命中
//   - 悬停时，鼠标所在的最深节点以实线框出，松散树下再用虚线画出它的松散边界

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../../foundations/random/random.h"
#include "demo.h"
#include "quadtree.h"
#include "ui.h"

namespace {

using namespace ga;

constexpr int kWidth = 800;
constexpr int kHeight = 500;
constexpr float kTop = 52;
constexpr Vec2 kOrigin{16, kTop + 12};
constexpr float kSize = kHeight - kTop - 24;  // 正方形场地的边长
constexpr float kPanelX = kOrigin.x + kSize + 24;
constexpr float kDt = 1.0f / 60.0f;
constexpr float kQuery = 70;  // 鼠标处查询区域的边长

inline Vector2 screen(float x, float y) { return {x + kOrigin.x, y + kOrigin.y}; }
inline Rectangle rect(const BoxN<2>& b) {
    return {b.min[0] + kOrigin.x, b.min[1] + kOrigin.y, b.max[0] - b.min[0], b.max[1] - b.min[1]};
}

struct Body {
    Vec2 pos, size, vel;
};

class QuadtreeDemo final : public Demo {
public:
    QuadtreeDemo() : tree_(world(), {}) { reset(); }

    void reset() override {
        Pcg32 r(4);
        auto rnd = [&](float lo, float hi) { return lo + r.nextFloat() * (hi - lo); };
        bodies_.clear();
        for (int i = 0; i < static_cast<int>(count_); ++i) {
            Vec2 size{rnd(6, 18), rnd(6, 18)};
            bodies_.push_back({{rnd(0, kSize - size.x), rnd(0, kSize - size.y)}, size, fromAngle(rnd(0, 2 * kPi)) * rnd(15, 55)});
        }
        if (big_) {
            bodies_.push_back({{40, 60}, {120, 80}, {16, 10}});
            bodies_.push_back({{250, 230}, {90, 110}, {-12, -15}});
        }
        rebuild();
    }

    bool step() override {
        for (Body& b : bodies_) {
            b.pos += b.vel * kDt;
            if (b.pos.x < 0) b.pos.x = 0, b.vel.x = std::abs(b.vel.x);
            if (b.pos.y < 0) b.pos.y = 0, b.vel.y = std::abs(b.vel.y);
            if (b.pos.x + b.size.x > kSize) b.pos.x = kSize - b.size.x, b.vel.x = -std::abs(b.vel.x);
            if (b.pos.y + b.size.y > kSize) b.pos.y = kSize - b.size.y, b.vel.y = -std::abs(b.vel.y);
        }
        rebuild();
        return true;
    }

    void update() override {
        Vector2 m = GetMousePosition();
        mouse_ = Vec2{m.x, m.y} - kOrigin;
        hover_ = !ui::wantsMouse() && mouse_.x >= 0 && mouse_.y >= 0 && mouse_.x < kSize && mouse_.y < kSize;
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        const auto& nodes = tree_.nodes();
        bool loose = loose_;

        // 区域查询：照着 Tree::query 的流程走一遍，记下访问过的节点和测试过的物体
        std::vector<char> visited(nodes.size(), 0), tested(bodies_.size(), 0), hit(bodies_.size(), 0);
        int nodesVisited = 0, tests = 0, hits = 0;
        BoxN<2> region{{mouse_.x - kQuery / 2, mouse_.y - kQuery / 2}, {mouse_.x + kQuery / 2, mouse_.y + kQuery / 2}};
        if (hover_) {
            std::vector<int> stack{0};
            while (!stack.empty()) {
                int n = stack.back();
                stack.pop_back();
                visited[n] = 1, ++nodesVisited;
                for (int j = nodes[n].head; j >= 0; j = tree_.next(j)) {
                    tested[j] = 1, ++tests;
                    if (overlaps(boxes_[j], region)) hit[j] = 1, ++hits;
                }
                if (nodes[n].firstChild < 0) continue;
                for (int c = nodes[n].firstChild; c < nodes[n].firstChild + 4; ++c)
                    if ((nodes[c].count > 0 || nodes[c].firstChild >= 0) && overlaps(tree_.looseBounds(c), region)) stack.push_back(c);
            }
        }
        stats_ = {nodesVisited, tests, hits};

        BeginScissorMode(int(kOrigin.x), int(kOrigin.y), int(kSize), int(kSize));
        for (size_t n = 1; n < nodes.size(); ++n)
            if (visited[n] && nodes[n].firstChild < 0) DrawRectangleRec(rect(nodes[n].cell), Fade(pal.frontier, 0.12f));
        // 节点的分割线：每个内部节点画一个十字
        for (const auto& node : nodes) {
            if (node.firstChild < 0) continue;
            float mx = (node.cell.min[0] + node.cell.max[0]) / 2, my = (node.cell.min[1] + node.cell.max[1]) / 2;
            float w = node.depth == 0 ? 1.6f : node.depth == 1 ? 1.2f : hl * 1.5f;
            Color c = Fade(pal.textMuted, node.depth <= 1 ? 0.8f : 0.5f);
            DrawLineEx(screen(mx, node.cell.min[1]), screen(mx, node.cell.max[1]), w, c);
            DrawLineEx(screen(node.cell.min[0], my), screen(node.cell.max[0], my), w, c);
        }

        // 物体
        std::vector<char> hot(bodies_.size(), 0);
        for (TreePair p : pairs_) hot[p.a] = hot[p.b] = 1;
        for (size_t i = 0; i < bodies_.size(); ++i) {
            Rectangle r = rect(boxes_[i]);
            bool stuck = nodes[tree_.nodeOf(i)].firstChild >= 0;  // 不在叶子里
            Color ink = hot[i] ? pal.path : pal.text;
            DrawRectangleRec(r, Fade(ink, hot[i] ? 0.16f : 0.07f));
            if (hover_ && hit[i]) DrawRectangleRec(r, Fade(pal.path, 0.3f));
            // 平时：黄色粗框 = 卡在上层（不在叶子里）。悬停查询时：黄色粗框 = 做了盒子测试
            Color edge = ink;
            float w = hl * 1.5f;
            if (!hover_ && stuck) edge = pal.frontier, w = 2.2f;
            if (hover_) {
                if (tested[i]) edge = hit[i] ? pal.path : pal.frontier, w = 2.2f;
                else edge = Fade(ink, 0.4f);
            }
            DrawRectangleLinesEx(r, w, edge);
        }

        // 鼠标所在的最深节点，及其松散边界
        if (hover_) {
            int n = 0;
            while (nodes[n].firstChild >= 0) {
                const auto& cell = nodes[n].cell;
                int c = (mouse_.x >= (cell.min[0] + cell.max[0]) / 2 ? 1 : 0) | (mouse_.y >= (cell.min[1] + cell.max[1]) / 2 ? 2 : 0);
                n = nodes[n].firstChild + c;
            }
            DrawRectangleLinesEx(rect(nodes[n].cell), 2, pal.text);
            if (loose && n != 0) {
                Rectangle lb = rect(tree_.looseBounds(n));
                dashedRect(lb, pal.text);
            }
            DrawRectangleLinesEx(rect(region), 1.5f, pal.path);
        }
        EndScissorMode();
        DrawRectangleLinesEx({kOrigin.x, kOrigin.y, kSize, kSize}, hl * 1.5f, pal.textMuted);

        drawPanel();
        DrawRectangleRec({0, 0, float(kWidth), kTop}, pal.background);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool changed = false;
        if (ui::button({16, 13, 64, 26}, "strict", !loose_) && loose_) loose_ = false, changed = true;
        if (ui::button({86, 13, 64, 26}, "loose", loose_) && !loose_) loose_ = true, changed = true;
        changed |= ui::slider({164, 13, 180, 26}, "split", &split_, 1, 16, "%.0f", 1);
        if (ui::slider({356, 13, 190, 26}, "count", &count_, 10, 300, "%.0f", 10)) reset();
        if (ui::toggle({558, 13, 52, 26}, "big", &big_)) reset();
        if (changed) rebuild();
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%ld tests · %zu pairs · %d above leaves", tests_, pairs_.size(), stuck_);
        return buf;
    }

private:
    static BoxN<2> world() { return {{0, 0}, {kSize, kSize}}; }

    void rebuild() {
        boxes_.clear();
        for (const Body& b : bodies_) boxes_.push_back({{b.pos.x, b.pos.y}, {b.pos.x + b.size.x, b.pos.y + b.size.y}});
        tree_ = Quadtree(world(), {6, static_cast<int>(split_), loose_ ? 2.0f : 1.0f});
        tree_.build(boxes_);
        tests_ = tree_.findPairs(pairs_);
        stuck_ = 0;
        for (size_t i = 0; i < bodies_.size(); ++i) stuck_ += tree_.nodes()[tree_.nodeOf(int(i))].firstChild >= 0;
    }

    static void dashedRect(Rectangle r, Color c) {
        drawDashedLine({r.x, r.y}, {r.x + r.width, r.y}, 1.5f, c, 5, 4);
        drawDashedLine({r.x + r.width, r.y}, {r.x + r.width, r.y + r.height}, 1.5f, c, 5, 4);
        drawDashedLine({r.x + r.width, r.y + r.height}, {r.x, r.y + r.height}, 1.5f, c, 5, 4);
        drawDashedLine({r.x, r.y + r.height}, {r.x, r.y}, 1.5f, c, 5, 4);
    }

    void drawPanel() const {
        const Palette& pal = palette();
        DrawLineEx({kPanelX - 12, kTop}, {kPanelX - 12, float(kHeight)}, hairline(), pal.gridLine);
        float x = kPanelX, y = kTop + 16, right = kWidth - 16.0f;
        char buf[32];
        auto row = [&](const char* label, long v, Color c) {
            drawText(label, {x, y}, 12, pal.textMuted);
            std::snprintf(buf, sizeof buf, "%ld", v);
            drawText(buf, {right - measureText(buf, 12).x, y}, 12, c);
            y += 20;
        };
        long n = static_cast<long>(bodies_.size());
        row("objects", n, pal.text);
        row("nodes", static_cast<long>(tree_.nodes().size()), pal.text);
        row("above leaves", stuck_, stuck_ ? pal.frontier : pal.text);
        y += 8;
        row("brute-force tests", n * (n - 1) / 2, pal.text);
        row("tree tests", tests_, pal.frontier);
        row("overlapping pairs", static_cast<long>(pairs_.size()), pal.path);
        y += 8;
        DrawLineEx({x, y}, {right, y}, hairline(), pal.gridLine);
        y += 12;
        if (!hover_) {
            drawText("hover: region query", {x, y}, 12, pal.textMuted);
            return;
        }
        drawText("region query", {x, y}, 12, pal.text);
        y += 20;
        row("nodes visited", stats_.nodes, pal.text);
        row("box tests", stats_.tests, pal.frontier);
        row("hits", stats_.hits, pal.path);
    }

    struct QueryStats {
        int nodes = 0, tests = 0, hits = 0;
    };

    std::vector<Body> bodies_;
    std::vector<BoxN<2>> boxes_;
    std::vector<TreePair> pairs_;
    Quadtree tree_;
    long tests_ = 0;
    int stuck_ = 0;
    bool loose_ = false;
    bool big_ = false;
    float split_ = 4;
    float count_ = 120;
    Vec2 mouse_{0, 0};
    bool hover_ = false;
    mutable QueryStats stats_;
};

}  // namespace

int main() {
    static QuadtreeDemo demo;
    ga::DemoConfig config;
    config.title = "Quadtree";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
