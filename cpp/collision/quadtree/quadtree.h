#pragma once

// 宽相检测：四叉树与八叉树
//   - Tree<2> 是四叉树，Tree<3> 是八叉树：同一份代码，每个节点有 2^D 个孩子
//   - 严格树（looseness = 1）：物体放在能完全装下它的最深节点里；跨越分割线的物体只能留在上层
//   - 松散树（looseness = 2）：每个节点的边界向外扩大一倍，物体按中心选孩子，深度只由大小决定
//   - 每帧重建；物体用链表挂在节点上，重建时不分配内存

#include <algorithm>
#include <array>
#include <compare>
#include <limits>
#include <vector>

#include "../intersect/intersect.h"

namespace ga {

// #region box
// D 维的轴对齐包围盒。二维时就是 2.1 的 Aabb
template <int D>
struct BoxN {
    std::array<float, D> min, max;
};

template <int D>
bool overlaps(const BoxN<D>& a, const BoxN<D>& b) {
    for (int k = 0; k < D; ++k)
        if (!(a.min[k] < b.max[k] && b.min[k] < a.max[k])) return false;  // 任何一个轴上分开就不相交
    return true;
}
// #endregion

inline BoxN<2> toBox(const Aabb& b) { return {{b.min.x, b.min.y}, {b.max.x, b.max.y}}; }
inline Aabb toAabb(const BoxN<2>& b) { return {{b.min[0], b.min[1]}, {b.max[0], b.max[1]}}; }

struct TreePair {
    int a, b;  // a < b
    friend auto operator<=>(const TreePair&, const TreePair&) = default;
};

template <int D>
long bruteForcePairs(const std::vector<BoxN<D>>& boxes, std::vector<TreePair>& out) {
    out.clear();
    int n = static_cast<int>(boxes.size());
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            if (overlaps(boxes[i], boxes[j])) out.push_back({i, j});
    return static_cast<long>(n) * (n - 1) / 2;
}

template <int D>
class Tree {
public:
    static constexpr int kChildren = 1 << D;  // 四叉树 4 个，八叉树 8 个

    struct Node {
        BoxN<D> cell;         // 节点负责的区域（孩子把它等分成 2^D 块）
        BoxN<D> loose;        // 松散边界：物体一定在它里面（严格树里等于 cell）
        int firstChild = -1;  // 孩子在 nodes_ 里连续存放；-1 表示叶子
        int head = -1;        // 挂在这个节点上的物体链表
        int count = 0;
        int depth = 0;
    };

    struct Options {
        int maxDepth = 8;      // 最多分几层
        int splitCount = 8;    // 叶子里的物体超过这个数就分裂
        float looseness = 1;   // 1 = 严格树，2 = 松散树
    };

    Tree(BoxN<D> world, Options opt) : world_(world), opt_(opt) { opt_.maxDepth = std::clamp(opt_.maxDepth, 0, 32); }

    const Options& options() const { return opt_; }
    const std::vector<Node>& nodes() const { return nodes_; }
    int nodeOf(int i) const { return nodeOf_[i]; }
    int next(int i) const { return next_[i]; }

    // #region build
    void build(const std::vector<BoxN<D>>& boxes) {
        boxes_ = &boxes;
        nodes_.clear();
        BoxN<D> all;  // 根节点收留一切，包括世界之外的物体
        all.min.fill(-std::numeric_limits<float>::infinity());
        all.max.fill(std::numeric_limits<float>::infinity());
        nodes_.push_back({world_, all, -1, -1, 0, 0});
        next_.assign(boxes.size(), -1);
        nodeOf_.assign(boxes.size(), 0);
        for (int i = 0; i < static_cast<int>(boxes.size()); ++i) insert(i);
    }
    // #endregion

    // 节点的松散边界：严格树就是 cell 本身；根节点是整个空间
    const BoxN<D>& looseBounds(int node) const { return nodes_[node].loose; }

    // #region pairs-strict
    // 严格树里，一个节点的物体完全在它的 cell 内，只可能与同一节点、祖先、后代里的物体相交。
    // 深度优先遍历，维护一个"祖先物体栈"：每个物体只与同节点的物体和栈里的物体比较，
    // 每一对恰好比较一次，不需要去重（descend 见下文）
    long findPairsStrict(std::vector<TreePair>& out) {
        out.clear();
        stack_.clear();
        return descend(0, 0, 0, out);
    }
    // #endregion

    // #region pairs-loose
    // 松散树里，兄弟节点的松散边界互相重叠，不同分支的物体也可能相交，祖先栈的办法会漏掉它们。
    // 改为每个物体做一次区域查询，只保留编号更大的那一方
    long findPairsByQuery(std::vector<TreePair>& out) const {
        out.clear();
        long tests = 0;
        const auto& b = *boxes_;
        for (int i = 0; i < static_cast<int>(b.size()); ++i)
            tests += query(b[i], [&](int j) {
                if (j > i) out.push_back({i, j});
            }, i);
        return tests;
    }
    // #endregion

    long findPairs(std::vector<TreePair>& out) {
        return opt_.looseness == 1 ? findPairsStrict(out) : findPairsByQuery(out);
    }

    // #region query
    // 与 region 重叠的所有物体各回调一次（每个物体只挂在一个节点上，天然不重复）。
    // 跳过与 region 不重叠的节点及其整棵子树。返回做了多少次盒子测试
    template <class F>
    long query(const BoxN<D>& region, F&& visit, int skip = -1) const {
        long tests = 0;
        int stack[33 * kChildren];  // 深度优先：每层最多压入 2^D 个
        int top = 0;
        stack[top++] = 0;
        while (top > 0) {
            int n = stack[--top];
            const Node& node = nodes_[n];
            for (int j = node.head; j >= 0; j = next_[j]) {
                if (j == skip) continue;
                ++tests;
                if (overlaps((*boxes_)[j], region)) visit(j);
            }
            if (node.firstChild < 0) continue;
            for (int c = node.firstChild; c < node.firstChild + kChildren; ++c)
                if ((nodes_[c].count > 0 || nodes_[c].firstChild >= 0) && overlaps(looseBounds(c), region)) stack[top++] = c;
        }
        return tests;
    }
    // #endregion

    // 物体应该放进节点 n 的哪个孩子（n 必须已经分裂）；-1 表示哪个都装不下
    int fitChild(int n, const BoxN<D>& b) const { return childFor(n, b); }

    // 每个物体所在的深度，用于统计"卡在上层"的物体
    int depthOf(int i) const { return nodes_[nodeOf_[i]].depth; }

private:
    // #region insert
    void insert(int i) {
        int n = 0;
        for (;;) {
            Node& node = nodes_[n];
            if (node.firstChild < 0) {
                if (node.count < opt_.splitCount || node.depth == opt_.maxDepth) break;
                split(n);
            }
            int c = childFor(n, (*boxes_)[i]);
            if (c < 0) break;  // 装不进任何一个孩子：留在这一层
            n = c;
        }
        link(n, i);
    }

    // 叶子分裂：创建 2^D 个孩子，把能装进孩子的物体移下去，跨越分割线的留下
    void split(int n) {
        int first = static_cast<int>(nodes_.size());
        BoxN<D> cell = nodes_[n].cell;
        for (int c = 0; c < kChildren; ++c) {
            BoxN<D> sub;
            for (int k = 0; k < D; ++k) {
                float mid = (cell.min[k] + cell.max[k]) * 0.5f;
                bool high = (c >> k) & 1;  // 第 k 位决定第 k 个轴上取哪一半
                sub.min[k] = high ? mid : cell.min[k];
                sub.max[k] = high ? cell.max[k] : mid;
            }
            BoxN<D> loose = sub;
            for (int k = 0; k < D; ++k) {
                float pad = (sub.max[k] - sub.min[k]) * (opt_.looseness - 1) * 0.5f;
                loose.min[k] -= pad, loose.max[k] += pad;
            }
            nodes_.push_back({sub, loose, -1, -1, 0, nodes_[n].depth + 1});
        }
        nodes_[n].firstChild = first;
        int j = nodes_[n].head;
        nodes_[n].head = -1, nodes_[n].count = 0;
        while (j >= 0) {
            int nx = next_[j];
            int c = childFor(n, (*boxes_)[j]);
            link(c < 0 ? n : c, j);
            j = nx;
        }
    }
    // #endregion

    // #region child
    // 物体应该放进哪个孩子；-1 表示哪个都装不下
    int childFor(int n, const BoxN<D>& b) const {
        const BoxN<D>& cell = nodes_[n].cell;
        int c = 0;
        for (int k = 0; k < D; ++k) {
            float mid = (cell.min[k] + cell.max[k]) * 0.5f;
            if (opt_.looseness == 1) {
                // 严格：整个盒子必须落在分割线的同一侧（根节点还要检查它是否在世界之内）
                if (b.min[k] < cell.min[k] || b.max[k] > cell.max[k]) return -1;
                if (b.max[k] <= mid) continue;
                if (b.min[k] >= mid) { c |= 1 << k; continue; }
                return -1;
            }
            // 松散：按中心选孩子，再看盒子是否在孩子的松散边界内
            float center = (b.min[k] + b.max[k]) * 0.5f;
            float half = (cell.max[k] - cell.min[k]) * 0.25f;  // 孩子 cell 的半边长
            float childCenter = center >= mid ? mid + half : mid - half;
            if (center >= mid) c |= 1 << k;
            float reach = half * opt_.looseness;  // 松散边界的半边长
            if (b.min[k] < childCenter - reach || b.max[k] > childCenter + reach) return -1;
        }
        return c + nodes_[n].firstChild;
    }
    // #endregion

    void link(int n, int i) {
        next_[i] = nodes_[n].head;
        nodes_[n].head = i;
        ++nodes_[n].count;
        nodeOf_[i] = n;
    }

    // #region descend
    // stack_[begin, end) 是祖先节点里、与当前节点 cell 重叠的物体
    long descend(int n, size_t begin, size_t end, std::vector<TreePair>& out) {
        const Node& node = nodes_[n];
        const auto& b = *boxes_;
        long tests = 0;
        for (int i = node.head; i >= 0; i = next_[i]) {
            for (int j = next_[i]; j >= 0; j = next_[j], ++tests)  // 同一节点里的其他物体
                if (overlaps(b[i], b[j])) out.push_back({std::min(i, j), std::max(i, j)});
            for (size_t k = begin; k < end; ++k, ++tests) {  // 祖先节点里的物体
                int j = stack_[k];
                if (overlaps(b[i], b[j])) out.push_back({std::min(i, j), std::max(i, j)});
            }
        }
        if (node.firstChild < 0) return tests;
        for (int c = node.firstChild; c < node.firstChild + kChildren; ++c) {
            const Node& child = nodes_[c];
            if (child.count == 0 && child.firstChild < 0) continue;
            // 只把与孩子 cell 重叠的祖先物体传下去：与 cell 不重叠，就不可能与 cell 里的任何物体重叠
            size_t mark = stack_.size();
            for (size_t k = begin; k < end; ++k)
                if (overlaps(b[stack_[k]], child.cell)) stack_.push_back(stack_[k]);
            for (int i = node.head; i >= 0; i = next_[i])
                if (overlaps(b[i], child.cell)) stack_.push_back(i);
            tests += descend(c, mark, stack_.size(), out);
            stack_.resize(mark);
        }
        return tests;
    }
    // #endregion

    BoxN<D> world_;
    Options opt_;
    const std::vector<BoxN<D>>* boxes_ = nullptr;
    std::vector<Node> nodes_;
    std::vector<int> next_;    // 物体链表：同一节点里的下一个物体
    std::vector<int> nodeOf_;  // 每个物体挂在哪个节点上
    std::vector<int> stack_;
};

using Quadtree = Tree<2>;
using Octree = Tree<3>;

}  // namespace ga
