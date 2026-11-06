#pragma once

// 演示与测试共用的例子：野外求生。目标是"火已点燃"并且"吃饱了"
// 同一个目标有多种达成方式：火可以用火柴点，也可以钻木取火；食物可以钓鱼、打猎或者采浆果。
// 规划器根据哪些工具还在、各个动作的代价，自动选出最省事的一条路

#include "goap.h"

namespace ga::camp {

// #region facts
enum Fact {
    AxeAvailable,      // 工具棚里有斧头
    HasAxe,
    HasWood,
    MatchesAvailable,  // 工具棚里有火柴
    HasMatches,
    FireLit,
    RodAvailable,      // 工具棚里有鱼竿
    HasRod,
    HasFish,
    HasSpear,
    HasMeat,
    Fed,
    kFactCount
};

inline const char* factName(int f) {
    const char* names[] = {"axe in shed", "has axe", "has wood", "matches in shed", "has matches", "fire lit",
                           "rod in shed", "has rod", "has fish", "has spear", "has meat", "fed"};
    return names[f];
}

enum Place { Shed, Forest, Camp, River, Bushes };
// #endregion

struct CampAction {
    GoapAction action;
    Place place;
};

// #region actions
inline std::vector<CampAction> actions() {
    auto act = [](const char* name, float cost, Place place, Facts pre, Facts eff) { return CampAction{{name, cost, pre, eff}, place}; };
    return {
        act("take axe", 1, Shed, Facts{}.set(AxeAvailable), Facts{}.set(HasAxe).set(AxeAvailable, false)),
        act("chop wood", 2, Forest, Facts{}.set(HasAxe), Facts{}.set(HasWood)),
        act("gather sticks", 6, Forest, Facts{}, Facts{}.set(HasWood)),
        act("take matches", 1, Shed, Facts{}.set(MatchesAvailable), Facts{}.set(HasMatches).set(MatchesAvailable, false)),
        act("light fire", 1, Camp, Facts{}.set(HasWood).set(HasMatches), Facts{}.set(FireLit).set(HasWood, false).set(HasMatches, false)),
        act("rub sticks", 8, Camp, Facts{}.set(HasWood), Facts{}.set(FireLit).set(HasWood, false)),
        act("take rod", 1, Shed, Facts{}.set(RodAvailable), Facts{}.set(HasRod).set(RodAvailable, false)),
        act("fish", 3, River, Facts{}.set(HasRod), Facts{}.set(HasFish)),
        act("make spear", 2, Camp, Facts{}.set(HasWood).set(HasAxe), Facts{}.set(HasSpear).set(HasWood, false)),
        act("hunt", 4, Forest, Facts{}.set(HasSpear), Facts{}.set(HasMeat)),
        act("cook fish", 1, Camp, Facts{}.set(HasFish).set(FireLit), Facts{}.set(Fed).set(HasFish, false)),
        act("cook meat", 1, Camp, Facts{}.set(HasMeat).set(FireLit), Facts{}.set(Fed).set(HasMeat, false)),
        act("pick berries", 9, Bushes, Facts{}, Facts{}.set(Fed)),
    };
}

inline Facts goal() { return Facts{}.set(FireLit).set(Fed); }
// #endregion

inline std::vector<GoapAction> plain(const std::vector<CampAction>& acts) {
    std::vector<GoapAction> v;
    for (const CampAction& a : acts) v.push_back(a.action);
    return v;
}

inline uint64_t start(bool axe, bool matches, bool rod) {
    uint64_t s = 0;
    if (axe) s |= uint64_t(1) << AxeAvailable;
    if (matches) s |= uint64_t(1) << MatchesAvailable;
    if (rod) s |= uint64_t(1) << RodAvailable;
    return s;
}

}  // namespace ga::camp
