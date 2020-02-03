#ifndef _PATHFINDINGUTILS_OTHERCOST_HEADER__
#define _PATHFINDINGUTILS_OTHERCOST_HEADER__

#include "types.hpp"

namespace pathfinding {

    struct OtherCost {
        cost_t cost;
        bool isDirty;

        static cost_t getCost(const OtherCost& c);

        friend bool operator ==(const OtherCost& a, const OtherCost& b);
        friend std::ostream& operator <<(std::ostream& ss, const OtherCost& a);
    };

}

#endif