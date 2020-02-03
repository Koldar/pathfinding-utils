#include "OtherCost.hpp"

namespace pathfinding {

    cost_t OtherCost::getCost(const OtherCost& c) {
        return c.cost;
    }

    bool operator ==(const OtherCost& a, const OtherCost& b) {
        return a.cost == b.cost && a.isDirty == b.isDirty;
    }

    std::ostream& operator <<(std::ostream& ss, const OtherCost& a) {
        ss << "{" << a.cost  <<", " << a.isDirty << "}";
        return ss;
    }

}