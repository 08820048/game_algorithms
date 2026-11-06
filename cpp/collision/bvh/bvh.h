#pragma once

// 宽相检测：Sweep and Prune 与动态 BVH
//   - SweepAndPrune：按 x 排序后扫描；保留上一帧的顺序，用插入排序更新（物体移动不多时接近线性）
//   - DynamicTree：动态包围盒层次（Box2D、Bullet 的宽相都是这一类）
//       * 叶子存放"胖"包围盒：物体在胖盒子里移动时树不需要改动
//       * 插入时沿着"周长增加最少"的方向往下走，找到最合适的兄弟
//       * AVL 式的旋转保持树的平衡
//       * 树与自身遍历，找出所有相交的物体对，每对恰好一次

#include <algorithm>
#include <cstdint>
#include <iterator>
#include <vector>

#include "../intersect/intersect.h"
#include "../spatial_hash/spatial_hash.h"  // Pair、bruteForcePairs

namespace ga {

inline Aabb merge(const Aabb& a, const Aabb& b) {
    return {{std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y)}, {std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y)}};
}
// 二维里用周长衡量盒子的"大小"（三维用表面积）
inline float perimeter(const Aabb& b) { return 2 * ((b.max.x - b.min.x) + (b.max.y - b.min.y)); }
inline bool containsBox(const Aabb& outer, const Aabb& inner) {
    return outer.min.x <= inner.min.x && outer.min.y <= inner.min.y && inner.max.x <= outer.max.x && inner.max.y <= outer.max.y;
}

// #region sap
class SweepAndPrune {
public:
    struct Entry {
        float lo, hi;  // 物体在 x 轴上的投影
        int id;
    };

    // 增量更新：保留上一帧的顺序，刷新每个物体的区间后做插入排序。
    // 物体每帧只移动一点点，顺序几乎不变，插入排序只需要很少的交换。返回交换次数
    long update(const std::vector<Aabb>& boxes) {
        boxes_ = &boxes;
        if (entries_.size() != boxes.size()) return sortFromScratch(boxes), 0;
        for (Entry& e : entries_) e.lo = boxes[e.id].min.x, e.hi = boxes[e.id].max.x;
        long swaps = 0;
        for (size_t i = 1; i < entries_.size(); ++i) {
            Entry e = entries_[i];
            size_t j = i;
            for (; j > 0 && entries_[j - 1].lo > e.lo; --j, ++swaps) entries_[j] = entries_[j - 1];
            entries_[j] = e;
        }
        return swaps;
    }

    // 对照：每帧从头排序
    void sortFromScratch(const std::vector<Aabb>& boxes) {
        boxes_ = &boxes;
        entries_.resize(boxes.size());
        for (int i = 0; i < static_cast<int>(boxes.size()); ++i) entries_[i] = {boxes[i].min.x, boxes[i].max.x, i};
        std::sort(entries_.begin(), entries_.end(), [](const Entry& a, const Entry& b) { return a.lo < b.lo; });
    }

    // 扫描：对每个物体，往后看那些 x 区间的起点落在它的区间内的物体，只有它们可能在 x 上重叠。
    // 一旦遇到起点超过它右端的物体就可以停下，后面的起点只会更大
    long findPairs(std::vector<Pair>& out) const {
        out.clear();
        const auto& b = *boxes_;
        long tests = 0;
        for (size_t p = 0; p < entries_.size(); ++p) {
            const Entry& e = entries_[p];
            for (size_t q = p + 1; q < entries_.size() && entries_[q].lo < e.hi; ++q) {
                ++tests;
                int i = e.id, j = entries_[q].id;
                // x 上几乎已经确定重叠，但仍做完整的测试：宽度为 0 的盒子恰好落在左端点上时，x 并不重叠
                if (overlaps(b[i], b[j])) out.push_back({std::min(i, j), std::max(i, j)});
            }
        }
        return tests;
    }

    const std::vector<Entry>& entries() const { return entries_; }

private:
    const std::vector<Aabb>* boxes_ = nullptr;
    std::vector<Entry> entries_;  // 按 lo 排好序
};
// #endregion

class DynamicTree {
public:
    struct Node {
        Aabb box;         // 叶子：胖包围盒；内部节点：两个孩子的包围盒的并
        int parent = -1;
        int left = -1, right = -1;  // 叶子两者都是 -1
        int height = 0;   // 叶子为 0；空闲节点为 -1
        int item = -1;    // 叶子对应的物体编号
        bool leaf() const { return left < 0; }
    };

    explicit DynamicTree(float margin = 2, bool balance = true) : margin_(margin), balance_(balance) {}

    int root() const { return root_; }
    const std::vector<Node>& nodes() const { return nodes_; }
    float margin() const { return margin_; }

    // #region proxy
    // 插入一个物体，返回叶子编号。叶子存放的是向外扩大 margin 的"胖"盒子
    int insert(const Aabb& tight, int item) {
        int leaf = allocate();
        nodes_[leaf].box = fatten(tight);
        nodes_[leaf].item = item;
        nodes_[leaf].height = 0;
        insertLeaf(leaf);
        return leaf;
    }

    void remove(int leaf) {
        removeLeaf(leaf);
        release(leaf);
    }

    // 物体移动后调用。仍在胖盒子里就什么都不做；跑出去了才删除、重新插入。返回是否重新插入
    bool move(int leaf, const Aabb& tight) {
        if (containsBox(nodes_[leaf].box, tight)) return false;
        removeLeaf(leaf);
        nodes_[leaf].box = fatten(tight);
        insertLeaf(leaf);
        return true;
    }
    // #endregion

    // #region query
    template <class F>
    long query(const Aabb& region, F&& visit) const {
        if (root_ < 0) return 0;
        long tests = 0;
        stack_.clear();
        stack_.push_back(root_);
        while (!stack_.empty()) {
            const Node& n = nodes_[stack_.back()];
            stack_.pop_back();
            ++tests;
            if (!overlaps(n.box, region)) continue;  // 整棵子树都可以跳过
            if (n.leaf()) visit(n.item);
            else stack_.push_back(n.left), stack_.push_back(n.right);
        }
        return tests;
    }
    // #endregion

    // #region pairs
    // 树与自身遍历：任意两个叶子都有唯一的"最近公共祖先"，只在那里比较它的左右两棵子树。
    // 两棵子树的包围盒不重叠，就整体跳过。tight 是物体的真实包围盒，叶子的胖盒子重叠后再用它确认
    long findPairs(const std::vector<Aabb>& tight, std::vector<Pair>& out) const {
        out.clear();
        if (root_ < 0) return 0;
        long tests = 0;
        selfPairs(root_, tight, out, tests);
        return tests;
    }
    // #endregion

    int height() const { return root_ < 0 ? 0 : nodes_[root_].height; }
    // 所有内部节点的周长之和：越小说明树越"紧凑"，查询时要检查的节点越少
    float internalPerimeter() const {
        float sum = 0;
        for (const Node& n : nodes_)
            if (n.height > 0) sum += perimeter(n.box);
        return sum;
    }

private:
    Aabb fatten(const Aabb& b) const { return {b.min - Vec2{margin_, margin_}, b.max + Vec2{margin_, margin_}}; }

    int allocate() {
        if (free_ >= 0) {
            int n = free_;
            free_ = nodes_[n].parent;  // 空闲链表借用 parent 字段
            nodes_[n] = Node{};
            return n;
        }
        nodes_.push_back({});
        return static_cast<int>(nodes_.size()) - 1;
    }
    void release(int n) {
        nodes_[n].height = -1;
        nodes_[n].parent = free_;
        free_ = n;
    }

    // #region insert
    void insertLeaf(int leaf) {
        if (root_ < 0) {
            root_ = leaf;
            nodes_[leaf].parent = -1;
            return;
        }
        // 1. 找兄弟：从根往下走。在每个节点上比较三种选择的代价：
        //    在这里和整个节点配对（新建一个父节点），或者继续往左、往右走
        Aabb box = nodes_[leaf].box;
        int idx = root_;
        while (!nodes_[idx].leaf()) {
            const Node& n = nodes_[idx];
            float area = perimeter(n.box);
            float combined = perimeter(merge(n.box, box));
            float here = 2 * combined;                // 新父节点的周长（乘 2 与 Box2D 的取法一致）
            float inherited = 2 * (combined - area);  // 往下走时，这个节点及其祖先都要变大
            auto descendCost = [&](int c) {
                float grown = perimeter(merge(nodes_[c].box, box));
                return (nodes_[c].leaf() ? grown : grown - perimeter(nodes_[c].box)) + inherited;
            };
            float costL = descendCost(n.left), costR = descendCost(n.right);
            if (here < costL && here < costR) break;
            idx = costL < costR ? n.left : n.right;
        }
        int sibling = idx;

        // 2. 新建一个父节点，把兄弟和新叶子挂在它下面
        int oldParent = nodes_[sibling].parent;
        int parent = allocate();
        nodes_[parent].parent = oldParent;
        nodes_[parent].box = merge(box, nodes_[sibling].box);
        nodes_[parent].height = nodes_[sibling].height + 1;
        nodes_[parent].left = sibling, nodes_[parent].right = leaf;
        nodes_[sibling].parent = parent, nodes_[leaf].parent = parent;
        if (oldParent < 0) root_ = parent;
        else (nodes_[oldParent].left == sibling ? nodes_[oldParent].left : nodes_[oldParent].right) = parent;

        // 3. 往上走，重新计算祖先的包围盒和高度，顺便做旋转
        refitUpward(parent);
    }

    void removeLeaf(int leaf) {
        if (leaf == root_) {
            root_ = -1;
            return;
        }
        int parent = nodes_[leaf].parent, grand = nodes_[parent].parent;
        int sibling = nodes_[parent].left == leaf ? nodes_[parent].right : nodes_[parent].left;
        // 父节点只剩一个孩子，没有存在的意义了：用兄弟顶替它的位置
        if (grand < 0) {
            root_ = sibling;
            nodes_[sibling].parent = -1;
        } else {
            (nodes_[grand].left == parent ? nodes_[grand].left : nodes_[grand].right) = sibling;
            nodes_[sibling].parent = grand;
        }
        release(parent);
        if (grand >= 0) refitUpward(grand);
    }

    void refitUpward(int idx) {
        while (idx >= 0) {
            if (balance_) idx = rotate(idx);
            Node& n = nodes_[idx];
            n.height = 1 + std::max(nodes_[n.left].height, nodes_[n.right].height);
            n.box = merge(nodes_[n.left].box, nodes_[n.right].box);
            idx = n.parent;
        }
    }
    // #endregion

    // #region rotate
    // 如果 a 的两棵子树高度相差超过 1，把较高的那个孩子提上来（AVL 树的单旋转）。返回旋转后占据 a 原位置的节点
    int rotate(int a) {
        Node& A = nodes_[a];
        if (A.leaf()) return a;
        int b = A.left, c = A.right;
        int diff = nodes_[c].height - nodes_[b].height;
        if (diff > 1) return liftChild(a, c, b, false);
        if (diff < -1) return liftChild(a, b, c, true);
        return a;
    }

    // 把 a 的高孩子 up 提到 a 的位置；a 变成 up 的孩子，并接手 up 较矮的那棵子树
    int liftChild(int a, int up, int other, bool upIsLeft) {
        int f = nodes_[up].left, g = nodes_[up].right;
        // up 接替 a 的位置
        nodes_[up].parent = nodes_[a].parent;
        nodes_[a].parent = up;
        int p = nodes_[up].parent;
        if (p < 0) root_ = up;
        else (nodes_[p].left == a ? nodes_[p].left : nodes_[p].right) = up;
        // up 保留较高的孙子，较矮的交给 a
        int keep = nodes_[f].height > nodes_[g].height ? f : g;
        int give = keep == f ? g : f;
        nodes_[up].left = a, nodes_[up].right = keep;
        if (upIsLeft) nodes_[a].left = give, nodes_[a].right = other;
        else nodes_[a].left = other, nodes_[a].right = give;
        nodes_[give].parent = a;
        nodes_[a].box = merge(nodes_[other].box, nodes_[give].box);
        nodes_[a].height = 1 + std::max(nodes_[other].height, nodes_[give].height);
        return up;
    }
    // #endregion

    void selfPairs(int n, const std::vector<Aabb>& tight, std::vector<Pair>& out, long& tests) const {
        const Node& node = nodes_[n];
        if (node.leaf()) return;
        selfPairs(node.left, tight, out, tests);
        selfPairs(node.right, tight, out, tests);
        crossPairs(node.left, node.right, tight, out, tests);
    }

    // #region cross
    void crossPairs(int a, int b, const std::vector<Aabb>& tight, std::vector<Pair>& out, long& tests) const {
        const Node &A = nodes_[a], &B = nodes_[b];
        ++tests;
        if (!overlaps(A.box, B.box)) return;
        if (A.leaf() && B.leaf()) {
            ++tests;
            if (overlaps(tight[A.item], tight[B.item])) out.push_back({std::min(A.item, B.item), std::max(A.item, B.item)});
            return;
        }
        // 拆开较大的那一个，让两边的大小保持接近
        if (B.leaf() || (!A.leaf() && perimeter(A.box) > perimeter(B.box))) {
            crossPairs(A.left, b, tight, out, tests);
            crossPairs(A.right, b, tight, out, tests);
        } else {
            crossPairs(a, B.left, tight, out, tests);
            crossPairs(a, B.right, tight, out, tests);
        }
    }
    // #endregion

    float margin_;
    bool balance_;
    int root_ = -1;
    int free_ = -1;
    std::vector<Node> nodes_;
    mutable std::vector<int> stack_;  // query 用的栈（不平衡的树可能很深，不用固定大小的数组）
};

// #region broadphase
// 物理引擎里的用法（Box2D 的思路）：物体对在帧与帧之间保留下来。
// 每帧只有跑出胖盒子、被重新插入的物体才去查询新的伙伴；其余物体的物体对沿用上一帧的结果。
// 保留的是"胖盒子重叠"的物体对，真正的重叠再用紧盒子筛一遍
class Broadphase {
public:
    explicit Broadphase(float margin = 4) : tree_(margin) {}

    const DynamicTree& tree() const { return tree_; }
    int leafOf(int id) const { return leaf_[id]; }

    void add(int id, const Aabb& tight) {
        if (id >= static_cast<int>(leaf_.size())) leaf_.resize(id + 1, -1), moved_.resize(id + 1, 0);
        leaf_[id] = tree_.insert(tight, id);
        moved_[id] = 1, moveList_.push_back(id);
    }
    void move(int id, const Aabb& tight) {
        if (tree_.move(leaf_[id], tight) && !moved_[id]) moved_[id] = 1, moveList_.push_back(id);
    }

    // 更新保留的物体对，返回本帧重新插入的物体数
    int update() {
        const auto& nodes = tree_.nodes();
        auto fat = [&](int id) -> const Aabb& { return nodes[leaf_[id]].box; };
        // 1. 胖盒子不再重叠的物体对，删掉
        size_t k = 0;
        for (Pair p : pairs_)
            if (overlaps(fat(p.a), fat(p.b))) pairs_[k++] = p;
        pairs_.resize(k);
        // 2. 移动过的物体查询新的伙伴。两个都移动过的物体对会被找到两次，只保留编号小的一方发起的那次
        fresh_.clear();
        for (int i : moveList_)
            tree_.query(fat(i), [&](int j) {
                if (j != i && (!moved_[j] || i < j)) fresh_.push_back({std::min(i, j), std::max(i, j)});
            });
        // 3. 并入（两个有序列表的合并，去掉已经存在的）
        std::sort(fresh_.begin(), fresh_.end());
        merged_.clear();
        std::set_union(pairs_.begin(), pairs_.end(), fresh_.begin(), fresh_.end(), std::back_inserter(merged_));
        pairs_.swap(merged_);
        int count = static_cast<int>(moveList_.size());
        for (int i : moveList_) moved_[i] = 0;
        moveList_.clear();
        return count;
    }

    // 胖盒子重叠的物体对里，紧盒子也重叠的那些。返回做了多少次测试
    long findPairs(const std::vector<Aabb>& tight, std::vector<Pair>& out) const {
        out.clear();
        for (Pair p : pairs_)
            if (overlaps(tight[p.a], tight[p.b])) out.push_back(p);
        return static_cast<long>(pairs_.size());
    }

    size_t candidatePairs() const { return pairs_.size(); }

private:
    DynamicTree tree_;
    std::vector<int> leaf_;
    std::vector<char> moved_;
    std::vector<int> moveList_;
    std::vector<Pair> pairs_;  // 保留下来的物体对（胖盒子重叠），有序
    std::vector<Pair> fresh_, merged_;
};
// #endregion

}  // namespace ga
