#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "navmesh.h"

namespace ga {

// #region portal-point
// 估计单位穿过门 a-b 的位置：从 from 指向终点的直线与门的交点，夹在门的范围内。
// 比直接取门的中点准确得多（门很长时，中点可能离真实路线很远）
inline Vec2 portalPoint(Vec2 a, Vec2 b, Vec2 from, Vec2 goal) {
    Vec2 ab = b - a, d = goal - from;
    float denom = cross(d, ab);
    float t = 0.5f;
    if (std::abs(denom) > 1e-6f) t = cross(d, from - a) / denom;  // 交点在 a-b 上的参数
    t = std::clamp(t, 0.02f, 0.98f);  // 稍微离开端点，端点通常紧贴障碍物
    return a + ab * t;
}
// #endregion

// #region channel
// 在多边形之间跑 A*，得到一串需要依次穿过的多边形（通道，channel）。
// 每个多边形的"位置"取单位进入它时穿过门的估计位置，代价是这些点之间的直线距离。
inline std::vector<int> findChannel(const NavMesh& mesh, Vec2 start, Vec2 goal, int* expanded = nullptr) {
    int startPoly = mesh.findPoly(start), goalPoly = mesh.findPoly(goal);
    if (startPoly < 0 || goalPoly < 0) return {};

    const int n = static_cast<int>(mesh.polys.size());
    std::vector<float> g(n, 1e30f);
    std::vector<Vec2> pos(n);
    std::vector<int> cameFrom(n, -1);
    std::vector<char> closed(n, 0);
    struct Entry {
        float f;
        int poly;
        bool operator<(const Entry& o) const { return f > o.f; }  // 小顶堆
    };
    std::vector<Entry> open{{distance(start, goal), startPoly}};
    g[startPoly] = 0;
    pos[startPoly] = start;
    cameFrom[startPoly] = startPoly;
    int count = 0;

    while (!open.empty()) {
        std::pop_heap(open.begin(), open.end());
        int cur = open.back().poly;
        open.pop_back();
        if (closed[cur]) continue;
        closed[cur] = 1;
        ++count;
        if (cur == goalPoly) break;
        for (const NavLink& link : mesh.polys[cur].links) {
            Vec2 entry = portalPoint(link.a, link.b, pos[cur], goal);
            float ng = g[cur] + distance(pos[cur], entry);
            if (!closed[link.to] && ng < g[link.to]) {
                g[link.to] = ng;
                pos[link.to] = entry;
                cameFrom[link.to] = cur;
                open.push_back({ng + distance(entry, goal), link.to});
                std::push_heap(open.begin(), open.end());
            }
        }
    }
    if (expanded) *expanded = count;
    if (cameFrom[goalPoly] < 0) return {};
    std::vector<int> channel;
    for (int p = goalPoly;; p = cameFrom[p]) {
        channel.push_back(p);
        if (p == startPoly) break;
    }
    std::reverse(channel.begin(), channel.end());
    return channel;
}
// #endregion

// #region portals
struct Portal {
    Vec2 left, right;  // 站在门前、面朝前进方向时的左端点和右端点
};

// 把通道转换成门的序列：起点和终点各视为一扇宽度为 0 的门
inline std::vector<Portal> channelPortals(const NavMesh& mesh, const std::vector<int>& channel, Vec2 start, Vec2 goal) {
    std::vector<Portal> portals{{start, start}};
    for (size_t i = 0; i + 1 < channel.size(); ++i) {
        const NavPoly& from = mesh.polys[channel[i]];
        for (const NavLink& link : from.links) {
            if (link.to != channel[i + 1]) continue;
            // 从多边形中心看向门：b 在 a 的右边（屏幕坐标 y 向下时叉积为正），则 a 是左端点
            bool aIsLeft = cross(link.a - from.center, link.b - from.center) > 0;
            portals.push_back(aIsLeft ? Portal{link.a, link.b} : Portal{link.b, link.a});
            break;
        }
    }
    portals.push_back({goal, goal});
    return portals;
}
// #endregion

// #region funnel
// 漏斗算法（Simple Stupid Funnel Algorithm，Mikko Mononen）：
// 从起点（漏斗尖）出发，用左右两条边夹住依次经过的门，不断收紧。
// 某一侧收紧时越过了另一侧，说明路径必须在另一侧的端点处拐弯：
// 把那个端点加入路径，作为新的漏斗尖，从它之后重新开始。
class Funnel {
public:
    explicit Funnel(std::vector<Portal> portals) : portals_(std::move(portals)) {
        if (portals_.empty()) return;
        apex_ = left_ = right_ = portals_[0].left;
        path_.push_back(apex_);
    }

    // 处理下一扇门。全部处理完时返回 false
    bool step() {
        if (i_ >= portals_.size()) return false;
        Vec2 l = portals_[i_].left, r = portals_[i_].right;

        // 收紧右边：新的右端点没有在当前右边的外侧（更右）
        if (cross(right_ - apex_, r - apex_) <= 0) {
            if (apex_ == right_ || cross(left_ - apex_, r - apex_) > 0) {
                right_ = r;  // 仍在左边的右侧：漏斗变窄
                rightIndex_ = i_;
            } else {
                restartAt(left_, leftIndex_);  // 越过了左边：在左端点拐弯
                return true;
            }
        }
        // 收紧左边：对称
        if (cross(left_ - apex_, l - apex_) >= 0) {
            if (apex_ == left_ || cross(right_ - apex_, l - apex_) < 0) {
                left_ = l;
                leftIndex_ = i_;
            } else {
                restartAt(right_, rightIndex_);
                return true;
            }
        }
        if (++i_ == portals_.size() && !(path_.back() == portals_.back().left)) {
            path_.push_back(portals_.back().left);  // 终点
        }
        return i_ < portals_.size();
    }

    const std::vector<Vec2>& path() const { return path_; }
    Vec2 apex() const { return apex_; }
    Vec2 left() const { return left_; }
    Vec2 right() const { return right_; }
    size_t index() const { return i_; }

private:
    void restartAt(Vec2 corner, size_t index) {
        path_.push_back(corner);
        apex_ = left_ = right_ = corner;
        leftIndex_ = rightIndex_ = index;
        i_ = index + 1;
    }

    std::vector<Portal> portals_;
    std::vector<Vec2> path_;
    Vec2 apex_{}, left_{}, right_{};
    size_t leftIndex_ = 0, rightIndex_ = 0;
    size_t i_ = 1;
};

inline std::vector<Vec2> stringPull(const std::vector<Portal>& portals) {
    Funnel f(portals);
    while (f.step()) {}
    return f.path();
}
// #endregion

// 对照：直接连接各扇门的中点
inline std::vector<Vec2> midpointPath(const std::vector<Portal>& portals) {
    std::vector<Vec2> path;
    for (const Portal& p : portals) path.push_back((p.left + p.right) * 0.5f);
    return path;
}

inline float pathLength(const std::vector<Vec2>& path) {
    float total = 0;
    for (size_t i = 1; i < path.size(); ++i) total += distance(path[i - 1], path[i]);
    return total;
}

}  // namespace ga
