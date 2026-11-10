#pragma once

// 波函数坍缩的图块集与样图（测试、测量和演示共用）

#include <string>
#include <vector>

#include "wfc.h"

namespace ga::wfc {

// #region roads
// 道路：每条边要么是草（0）要么是路（1），16 种组合各是一个图块。掩码第 d 位 = d 方向有路
// noDeadEnds = true 时去掉只有一个出口的"断头路"，这时有些局面无解，会出现矛盾
inline std::vector<SocketTile> roadTiles(bool noDeadEnds) {
    std::vector<SocketTile> tiles;
    for (int m = 0; m < 16; ++m) {
        int k = __builtin_popcount(unsigned(m));
        if (noDeadEnds && k == 1) continue;
        double w = k == 0 ? 6 : k == 1 ? 0.3 : k == 2 ? ((m == 5 || m == 10) ? 2 : 1) : 0.5;
        tiles.push_back({{m & 1, (m >> 1) & 1, (m >> 2) & 1, (m >> 3) & 1}, w});
    }
    return tiles;
}
inline int roadMask(const SocketTile& t) { return t.edge[0] | t.edge[1] << 1 | t.edge[2] << 2 | t.edge[3] << 3; }
// #endregion

// #region terrain
// 地形：图块的四个角各是水（0）、沙（1）、草（2）之一，同一条边上的两个角不能是水和草。
// 边的接口 = 这条边上两个角的颜色（竖边从上到下、横边从左到右），相邻图块共享的角颜色必须一致
struct CornerTile {
    int tl, tr, br, bl;
};
inline std::vector<CornerTile> terrainCorners() {
    std::vector<CornerTile> out;
    for (int m = 0; m < 81; ++m) {
        CornerTile c{m % 3, m / 3 % 3, m / 9 % 3, m / 27};
        auto bad = [](int a, int b) { return (a == 0 && b == 2) || (a == 2 && b == 0); };
        if (bad(c.tl, c.tr) || bad(c.tr, c.br) || bad(c.br, c.bl) || bad(c.bl, c.tl)) continue;
        out.push_back(c);
    }
    return out;
}
// uniformWeight：四角同色的"纯"图块的权重（过渡图块为 1）。太大时整张图会被一种地形占满（见 wfc_bench terrain）
inline std::vector<SocketTile> terrainTiles(double uniformWeight = 8) {
    std::vector<SocketTile> tiles;
    for (const CornerTile& c : terrainCorners()) {
        bool uniform = c.tl == c.tr && c.tr == c.br && c.br == c.bl;
        tiles.push_back({{c.tr * 3 + c.br, c.bl * 3 + c.br, c.tl * 3 + c.bl, c.tl * 3 + c.tr}, uniform ? uniformWeight : 1.0});
    }
    return tiles;
}
// #endregion

// #region samples
// 样图：字符画，每种字符一种颜色
inline Sample sampleFromText(const std::vector<std::string>& rows, std::string* alphabet) {
    Sample s;
    s.h = int(rows.size()), s.w = int(rows[0].size());
    for (const std::string& r : rows)
        for (char ch : r) {
            size_t k = alphabet->find(ch);
            if (k == std::string::npos) alphabet->push_back(ch), k = alphabet->size() - 1;
            s.px.push_back(uint8_t(k));
        }
    return s;
}

// 房间：墙 #、地板 .、门 +。四周是墙，按非周期方式取图案
inline const std::vector<std::string>& roomsText() {
    static const std::vector<std::string> t = {
        "################",
        "#......#.......#",
        "#......#.......#",
        "#......+.......#",
        "#......#.......#",
        "###+####.......#",
        "#......#####+###",
        "#......#.......#",
        "#......+.......#",
        "#......#.......#",
        "#......#.......#",
        "####+######+####",
        "#.....#........#",
        "#.....+........#",
        "#.....#........#",
        "################",
    };
    return t;
}

// 湖泊：水 ~、沙 :、草 .。水和草之间总隔着一圈沙；首尾相接（周期）
inline const std::vector<std::string>& lakesText() {
    static const std::vector<std::string> t = {
        "................",
        "....::::........",
        "...:~~~~:.......",
        "..:~~~~~~:......",
        "..:~~~~~~:...::.",
        "...:~~~~:...:~~:",
        "....::::....:~~:",
        ".............::.",
        "......::........",
        ".....:~~:.......",
        "....:~~~~:......",
        "....:~~~~:......",
        ".....:~~:....::.",
        "......::....:~~:",
        ".............::.",
        "................",
    };
    return t;
}

// 迷宫：6×6 个格子的周期迷宫（深度优先生成），放大成 12×12，墙 #、通道 .
inline std::vector<std::string> mazeText(uint64_t seed = 3) {
    const int n = 6;
    std::vector<std::string> g(2 * n, std::string(2 * n, '#'));
    std::vector<int> seen(n * n, 0), stack{0};
    Pcg32 rng{seed};
    seen[0] = 1;
    g[1][1] = '.';
    while (!stack.empty()) {
        int c = stack.back(), x = c % n, y = c / n;
        int opts[4], k = 0;
        for (int d = 0; d < 4; ++d) {
            int nx = (x + kDx[d] + n) % n, ny = (y + kDy[d] + n) % n;
            if (!seen[ny * n + nx]) opts[k++] = d;
        }
        if (!k) {
            stack.pop_back();
            continue;
        }
        int d = opts[rng.below(uint32_t(k))];
        int nx = (x + kDx[d] + n) % n, ny = (y + kDy[d] + n) % n;
        seen[ny * n + nx] = 1;
        g[(2 * y + 1 + kDy[d] + 2 * n) % (2 * n)][(2 * x + 1 + kDx[d] + 2 * n) % (2 * n)] = '.';
        g[2 * ny + 1][2 * nx + 1] = '.';
        stack.push_back(ny * n + nx);
    }
    return g;
}
// #endregion

}  // namespace ga::wfc
