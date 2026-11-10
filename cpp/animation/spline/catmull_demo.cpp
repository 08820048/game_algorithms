// Catmull-Rom 样条
//   - 曲线经过每一个控制点。拖动点；点空白处在最近的一段中间插入新点；右键删除
//   - alpha：节点间距 = 两点距离的 alpha 次方。0 均匀，0.5 向心，1 弦长。
//     点的间距相差悬殊时，均匀参数化在急弯处冲出很远，甚至打圈（红色标出有自交或尖点的段），向心参数化不会
//   - 小车沿曲线走：constant speed 打开时按弧长匀速，关闭时每段用同样的时间（t 匀速）

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "demo.h"
#include "spline.h"
#include "ui.h"

namespace {

using namespace ga;
using namespace ga::spline;

constexpr int kWidth = 800, kHeight = 480;
constexpr float kTop = 80;

Vector2 rv(Vec2 v) { return {v.x, v.y}; }

std::vector<Vec2> defaultPoints() {
    // 一个发夹弯（弯顶两点相距 40 px，两条直道各长约 400 px），底边还有两个只隔 16 px 的点
    return {{100, 280}, {230, 134}, {600, 130}, {720, 210}, {330, 245}, {330, 285}, {720, 348}, {600, 440}, {420, 437}, {404, 437}, {230, 440}};
}

class CatmullDemo final : public Demo {
public:
    CatmullDemo() : pts_(defaultPoints()) { rebuild(); }
    void reset() override { pos_ = 0; }
    bool step() override {
        if (segs_.empty()) return false;
        pos_ += constant_ ? total_ / 600 : float(segs_.size()) / 600;
        float end = constant_ ? total_ : float(segs_.size());
        if (pos_ >= end) pos_ = closed_ ? pos_ - end : 0;
        return true;
    }
    void update() override {
        Vector2 m = GetMousePosition();
        Vec2 mv{m.x, m.y};
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) drag_ = -1;
        bool inside = !ui::wantsMouse() && m.y > kTop;
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && inside) {
            drag_ = nearest(mv);
            if (drag_ < 0) {
                insert(mv);
                drag_ = nearest(mv);
            }
        }
        if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT) && inside && pts_.size() > 3) {
            int k = nearest(mv);
            if (k >= 0) pts_.erase(pts_.begin() + k), rebuild();
        }
        if (drag_ >= 0 && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            pts_[size_t(drag_)] = {std::clamp(m.x, 8.0f, kWidth - 8.0f), std::clamp(m.y, kTop + 8, kHeight - 8.0f)};
            rebuild();
        }
    }

    void draw() const override {
        const Palette& pal = palette();
        const float hl = hairline();
        for (size_t i = 0; i + 1 < pts_.size(); ++i) drawDashedLine(rv(pts_[i]), rv(pts_[i + 1]), hl, Fade(pal.textMuted, 0.6f));
        if (closed_ && pts_.size() > 2) drawDashedLine(rv(pts_.back()), rv(pts_[0]), hl, Fade(pal.textMuted, 0.6f));
        if (compare_) {
            const float as[3] = {0, 0.5f, 1};
            const Color cs[3] = {Fade(pal.path, 0.55f), pal.text, Fade(pal.frontier, 0.9f)};
            for (int k = 0; k < 3; ++k)
                for (const Cubic& c : catmullRomChain(pts_, as[k], closed_)) drawCubic(c, k == 1 ? 2.0f : 1.5f, cs[k]);
        } else {
            for (size_t i = 0; i < segs_.size(); ++i) drawCubic(segs_[i], 2.0f, bad_[i] ? pal.path : pal.text);
        }
        for (Vec2 p : pts_) {
            DrawCircleV(rv(p), 5, pal.background);
            DrawCircleLinesV(rv(p), 5, pal.text);
        }
        // 小车：沿切线方向的三角形
        if (!segs_.empty()) {
            int k;
            float t;
            locate(&k, &t);
            Vec2 p = segs_[size_t(k)].at(t), d = normalized(segs_[size_t(k)].d1(t));
            if (lengthSq(d) == 0) d = {1, 0};
            Vec2 n = perp(d);
            // raylib 只画逆时针的三角形，朝向随切线变化，两种顺序都画一次
            DrawTriangle(rv(p + d * 11), rv(p - d * 7 - n * 6), rv(p - d * 7 + n * 6), pal.path);
            DrawTriangle(rv(p + d * 11), rv(p - d * 7 + n * 6), rv(p - d * 7 - n * 6), pal.path);
        }

        char buf[128];
        int nb = 0;
        for (bool b : bad_) nb += b;
        if (compare_) std::snprintf(buf, sizeof buf, "red: uniform   black: centripetal   amber: chordal");
        else std::snprintf(buf, sizeof buf, "points %zu   segments %zu   with loop/cusp %d   length %.0f px", pts_.size(), segs_.size(), nb, total_);
        drawText(buf, {12, kTop + 8}, 11, pal.textMuted);
        drawText("drag points · click to insert · right-click to delete", {12, kHeight - 20.0f}, 11, pal.textMuted);
        DrawLineEx({0, kTop}, {float(kWidth), kTop}, hl, pal.gridLine);
    }

    void drawUi() override {
        bool ch = false;
        const char* names[3] = {"uniform", "centripetal", "chordal"};
        const float as[3] = {0, 0.5f, 1};
        float x = 12;
        for (int i = 0; i < 3; ++i) {
            float w = i == 1 ? 104 : 80;
            if (ui::button({x, 10, w, 26}, names[i], !compare_ && std::fabs(alpha_ - as[i]) < 1e-3f)) alpha_ = as[i], compare_ = false, ch = true;
            x += w + 4;
        }
        ch |= ui::slider({x + 8, 10, 190, 26}, "alpha", &alpha_, 0, 1, "%.2f", 0.05f);
        ch |= ui::toggle({x + 210, 10, 90, 26}, "compare", &compare_);
        if (ui::toggle({12, 44, 140, 26}, "constant speed", &constant_)) pos_ = 0;
        ch |= ui::toggle({158, 44, 80, 26}, "closed", &closed_);
        if (ui::button({244, 44, 110, 26}, "reset points")) pts_ = defaultPoints(), ch = true;
        if (ch) rebuild();
    }

    std::string status() const override {
        char buf[96];
        std::snprintf(buf, sizeof buf, "alpha %.2f · %zu segments · %s", alpha_, segs_.size(), constant_ ? "constant speed" : "constant dt per segment");
        return buf;
    }

private:
    void rebuild() {
        segs_ = catmullRomChain(pts_, alpha_, closed_);
        tables_.clear(), bad_.clear(), starts_.clear();
        total_ = 0;
        for (const Cubic& c : segs_) {
            tables_.emplace_back(c, 16);
            starts_.push_back(total_);
            total_ += float(tables_.back().length());
            Defects d = findDefects(c, 100);
            bad_.push_back(d.cusp || d.loop);
        }
        float end = constant_ ? total_ : float(segs_.size());
        if (pos_ > end) pos_ = 0;
    }
    // 当前位置在哪一段、段内参数是多少
    void locate(int* k, float* t) const {
        if (!constant_) {
            *k = std::min(int(segs_.size()) - 1, int(pos_));
            *t = std::clamp(pos_ - *k, 0.0f, 1.0f);
            return;
        }
        int i = int(std::upper_bound(starts_.begin(), starts_.end(), pos_) - starts_.begin()) - 1;
        i = std::clamp(i, 0, int(segs_.size()) - 1);
        *k = i;
        *t = tables_[size_t(i)].paramAt(pos_ - starts_[size_t(i)], 3);
    }
    void drawCubic(const Cubic& c, float w, Color col) const {
        Vec2 prev = c.p[0];
        for (int i = 1; i <= 48; ++i) {
            Vec2 q = c.at(i / 48.0f);
            DrawLineEx(rv(prev), rv(q), w, col);
            prev = q;
        }
    }
    int nearest(Vec2 m) const {
        for (size_t i = 0; i < pts_.size(); ++i)
            if (length(pts_[i] - m) < 10) return int(i);
        return -1;
    }
    // 插到离鼠标最近的那一段中间
    void insert(Vec2 m) {
        size_t bi = 0;
        float bd = 1e30f;
        for (size_t i = 0; i < segs_.size(); ++i)
            for (int k = 0; k <= 32; ++k) {
                float d = length(segs_[i].at(k / 32.0f) - m);
                if (d < bd) bd = d, bi = i;
            }
        if (segs_.empty() || bd > 60) {
            // 离曲线太远：接在离得最近的端点上（开放曲线）或最近的段后面（闭合）
            if (!closed_ && length(pts_.front() - m) < length(pts_.back() - m)) pts_.insert(pts_.begin(), m);
            else if (!closed_) pts_.push_back(m);
            else pts_.insert(pts_.begin() + long(bi) + 1, m);
        } else {
            pts_.insert(pts_.begin() + long(bi) + 1, m);
        }
        rebuild();
    }

    std::vector<Vec2> pts_;
    std::vector<Cubic> segs_;
    std::vector<ArcLengthTable> tables_;
    std::vector<float> starts_;
    std::vector<bool> bad_;
    float alpha_ = 0, total_ = 0, pos_ = 0;
    bool closed_ = true, constant_ = true, compare_ = false;
    int drag_ = -1;
};

}  // namespace

int main() {
    static CatmullDemo demo;
    ga::DemoConfig config;
    config.title = "Catmull-Rom spline";
    config.width = kWidth;
    config.height = kHeight;
    config.stepsPerSecond = 60.0f;
    return ga::runDemo(demo, config);
}
