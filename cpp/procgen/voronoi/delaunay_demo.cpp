// Bowyer–Watson 逐点插入
//   - 每一步插入一个点：黄色是从上一个新三角形"走"到新点穿过的三角形，红色是外接圆包含新点的三角形（空腔），删掉后把空腔边界和新点连起来
//   - circles：画出空腔里每个三角形的外接圆——新点都在圆内
//   - order：random 随机顺序，走路很长；hilbert 按希尔伯特曲线排序，几乎每次都在隔壁；sort-x 从左到右，每个新点都在凸包外
//   - voronoi：叠加当前已插入点的 Voronoi 图（Delaunay 的对偶：每条 Voronoi 边垂直平分一条 Delaunay 边）

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "delaunay.h"
#include "demo.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::delaunay;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80;
constexpr float kMx = 12, kMy = kTop + 10, kDw = 580, kDh = 380;
constexpr float kPx = 606;

class DelaunayDemo final : public Demo {
public:
    DelaunayDemo() { generate(); }
    void reset() override {
        tr_ = Triangulation(pts_, Precision::Double);
        k_ = 0;
        trace_ = {};
        walkTotal_ = 0;
        cells_.clear();
    }
    bool step() override {
        if (k_ >= int(order_.size())) return false;
        trace_ = {};
        long long w0 = tr_.stats().walkSteps;
        tr_.insert(order_[k_++], &trace_);
        lastWalk_ = tr_.stats().walkSteps - w0;
        walkTotal_ = tr_.stats().walkSteps;
        if (showVoronoi_) rebuildCells();
        return k_ < int(order_.size());
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        DrawRectangleLinesEx({kMx - 1, kMy - 1, kDw + 2, kDh + 2}, hl, pal.gridLine);
        BeginScissorMode(int(kMx), int(kMy), int(kDw), int(kDh));
        int cur = k_ > 0 ? order_[k_ - 1] : -1;
        for (const auto& t : trace_.walk) fillTri(t, Fade(pal.frontier, 0.35f));
        for (const auto& t : trace_.cavity) fillTri(t, Fade(pal.path, 0.16f));
        // 当前三角网
        for (const auto& t : tr_.triangles())
            for (int e = 0; e < 3; ++e) {
                int a = t[e], b = t[(e + 1) % 3];
                bool fresh = a == cur || b == cur;
                DrawLineEx(sp(pts_[a]), sp(pts_[b]), fresh ? 1.6f : hl * 1.3f, fresh ? pal.path : Fade(pal.text, 0.55f));
            }
        if (showCircles_)
            for (const auto& t : trace_.cavity) {
                auto [c, r] = circumcircle(t);
                DrawCircleLinesV(sp(c), r, Fade(pal.path, 0.7f));
            }
        if (showVoronoi_)
            for (size_t i = 0; i < cells_.size(); ++i) {
                const Poly& c = cells_[i];
                for (size_t k = 0; k < c.size(); ++k) {
                    P2 a = c[k], b = c[(k + 1) % c.size()];
                    DrawLineEx({kMx + float(a.x), kMy + float(a.y)}, {kMx + float(b.x), kMy + float(b.y)}, 1.2f, Fade(pal.goal, 0.75f));
                }
            }
        EndScissorMode();
        for (size_t j = 0; j < order_.size(); ++j) {
            int i = order_[j];
            if (int(j) < k_) DrawCircleV(sp(pts_[i]), 2.6f, pal.text);
            else DrawCircleLinesV(sp(pts_[i]), 2.2f, Fade(pal.textMuted, 0.7f));
        }
        if (cur >= 0) DrawCircleV(sp(pts_[cur]), 4.5f, pal.path);

        char buf[128];
        float y = kMy;
        auto line = [&](const char* s, Color col, float size = 12) {
            drawText(s, {kPx, y}, size, col);
            y += size + 7;
        };
        std::snprintf(buf, sizeof buf, "inserted %d / %d", k_, int(pts_.size()));
        line(buf, pal.text, 13);
        std::snprintf(buf, sizeof buf, "triangles %d", int(tr_.triangles().size()));
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "hull points %d", int(tr_.hull().size()));
        line(buf, pal.text);
        y += 6;
        line("this insertion", pal.textMuted, 11);
        std::snprintf(buf, sizeof buf, "walk %lld triangles", lastWalk_);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "cavity %d triangles", int(trace_.cavity.size()));
        line(buf, pal.text);
        y += 6;
        line("average per point", pal.textMuted, 11);
        int located = std::max<long long>(1, tr_.stats().located);
        std::snprintf(buf, sizeof buf, "walk %.1f", double(walkTotal_) / located);
        line(buf, pal.text);
        std::snprintf(buf, sizeof buf, "removed %.2f", double(tr_.stats().cavity) / located);
        line(buf, pal.text);
        y += 10;
        if (tr_.started()) {
            long long n = tr_.stats().located, h = (long long)tr_.hull().size();
            std::snprintf(buf, sizeof buf, "2n - 2 - h = %lld", 2 * n - 2 - h);
            line(buf, pal.textMuted, 11);
        }
        if (k_ >= int(order_.size())) line("done", pal.textMuted, 11);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        ch |= ui::slider({12, 10, 150, 26}, "seed", &seed_, 1, 50, "%.0f", 1);
        ch |= ui::slider({172, 10, 190, 26}, "points", &n_, 10, 400, "%.0f", 1);
        const char* names[] = {"random", "hilbert", "sort-x"};
        for (int i = 0; i < 3; ++i)
            if (ui::button({12.0f + i * 80, 44, 76, 26}, names[i], ord_ == i)) ord_ = i, ch = true;
        ui::toggle({262, 44, 90, 26}, "circles", &showCircles_);
        if (ui::toggle({360, 44, 96, 26}, "voronoi", &showVoronoi_) && showVoronoi_) rebuildCells();
        if (ch) generate(), notifyProblemChanged();
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "%d / %d points · walk %.1f per point", k_, int(pts_.size()), double(walkTotal_) / std::max<long long>(1, tr_.stats().located));
        return buf;
    }

private:
    Vector2 sp(Vec2 p) const { return {kMx + p.x, kMy + p.y}; }
    void fillTri(const std::array<int, 3>& t, Color c) const {
        // 三角形按 orient > 0 存储，在 y 朝下的屏幕上是顺时针；raylib 要求逆时针，所以反过来画
        DrawTriangle(sp(pts_[t[0]]), sp(pts_[t[2]]), sp(pts_[t[1]]), c);
    }
    std::pair<Vec2, float> circumcircle(const std::array<int, 3>& t) const {
        Vec2 a = pts_[t[0]], b = pts_[t[1]] - a, c = pts_[t[2]] - a;
        float d = 2 * (b.x * c.y - b.y * c.x);
        Vec2 u{(c.y * lengthSq(b) - b.y * lengthSq(c)) / d, (b.x * lengthSq(c) - c.x * lengthSq(b)) / d};
        return {a + u, length(u)};
    }
    void rebuildCells() {
        cells_.clear();
        auto nb = tr_.neighbors();
        for (int j = 0; j < k_; ++j) {
            int i = order_[j];
            Poly c = boxPoly(kDw, kDh);
            for (int o : nb[i]) c = clipBisector(c, pts_[i], pts_[o]);
            cells_.push_back(std::move(c));
        }
    }
    void generate() {
        Pcg32 rng{uint64_t(seed_)};
        pts_.assign(int(n_), {});
        for (Vec2& p : pts_) p = {10 + rng.nextFloat() * (kDw - 20), 10 + rng.nextFloat() * (kDh - 20)};
        Order o = ord_ == 0 ? Order::Random : ord_ == 1 ? Order::Hilbert : Order::SortX;
        order_ = insertionOrder(pts_, o, rng);
        reset();
    }

    float seed_ = 4, n_ = 120;
    int ord_ = 0;
    bool showCircles_ = true, showVoronoi_ = false;
    std::vector<Vec2> pts_;
    std::vector<int> order_;
    Triangulation tr_{{}};
    Trace trace_;
    int k_ = 0;
    long long lastWalk_ = 0, walkTotal_ = 0;
    std::vector<Poly> cells_;
};

}  // namespace

int main() {
    static DelaunayDemo demo;
    ga::DemoConfig config;
    config.title = "Bowyer-Watson Delaunay triangulation";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 4.0f;
    return ga::runDemo(demo, config);
}
