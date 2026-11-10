#pragma once

// 本章用到的 L-System（测试、测量和演示共用）。曲线类来自经典的分形定义，植物类前几个取自
// Prusinkiewicz & Lindenmayer《The Algorithmic Beauty of Plants》(1990) 图 1.24、1.27，最后一个是本章自己设计的

#include <vector>

#include "lsystem.h"

namespace ga::lsys {

// #region presets
struct Preset {
    const char* name;
    LSystem sys;
    TurtleOptions turtle;
    int gen;      // 演示的缺省代数
    int maxGen;   // 演示允许的最大代数（再大线段太多）
    int lattice;  // 转角是 360°/k 时的 k（4 或 6），否则 0
};

inline std::vector<Preset> presets() {
    std::vector<Preset> out;
    auto add = [&](const char* name, LSystem s, double angle, double heading, int gen, int maxGen, int lattice) {
        TurtleOptions t;
        t.angle = angle, t.heading = heading;
        out.push_back({name, std::move(s), t, gen, maxGen, lattice});
    };
    LSystem s;
    // Koch 雪花：三条边，每条边每代换成四段，中间凸起一个尖
    s = {}, s.axiom = "F--F--F", s.rule('F', "F+F--F+F");
    add("koch", s, 60, 0, 4, 7, 6);
    // 龙形曲线：X、Y 不画，只控制转向
    s = {}, s.axiom = "FX", s.rule('X', "X+YF+").rule('Y', "-FX-Y");
    add("dragon", s, 90, 0, 10, 16, 4);
    // Hilbert 曲线：n 代走遍 2^n × 2^n 个格子，每个恰好一次
    s = {}, s.axiom = "A", s.rule('A', "+BF-AFA-FB+").rule('B', "-AF+BFB+FA-");
    add("hilbert", s, 90, 0, 5, 8, 4);
    // Sierpinski 箭头曲线：F、G 都画线，交替弯折
    s = {}, s.axiom = "F", s.rule('F', "G-F-G").rule('G', "F+G+F");
    add("sierpinski", s, 60, 0, 6, 10, 6);
    // 植物（ABOP 图 1.24e）：X 是生长点，F 每代变长一倍
    s = {}, s.axiom = "X", s.rule('X', "F[+X][-X]FX").rule('F', "FF");
    add("plant", s, 25.7, 90, 6, 8, 0);
    // 灌木（ABOP 图 1.24f）
    s = {}, s.axiom = "X", s.rule('X', "F-[[X]+X]+F[+FX]-X").rule('F', "FF");
    add("weed", s, 22.5, 90, 5, 7, 0);
    // 随机植物（ABOP 图 1.27）：F 有三种改写，各 1/3
    s = {}, s.axiom = "F", s.rule('F', "F[+F]F[-F]F").rule('F', "F[+F]F").rule('F', "F[-F]F");
    add("stochastic", s, 25.7, 90, 5, 6, 0);
    return out;
}

// 演示 4.9b 的树（本章自己设计）：X 是芽。每个芽长出一段树干 F，分出一到两根更细（!）更短（"）的侧枝，
// 顶上留一个芽继续长。! 和 " 放在括号里只影响侧枝；放在括号外则影响之后的整段主干。
// 已有的 F 每代有六成概率变长一倍（F → FF），所以老枝条长短不一
inline LSystem treeSystem() {
    LSystem s;
    s.axiom = "X";
    s.rule('X', "F[!\"+X][!\"-X]!\"FX", 4)
        .rule('X', "F[!\"+X]!\"FX", 3)
        .rule('X', "F[!\"-X]!\"FX", 3)
        .rule('F', "FF", 6)
        .rule('F', "F", 4);
    return s;
}
// #endregion

}  // namespace ga::lsys
