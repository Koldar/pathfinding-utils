#ifndef _ZEROHEURISTIC_HEADER__
#define _ZEROHEURISTIC_HEADER__

#include "IHeuristic.hpp"

namespace pathfinding::search {

/**
 * @brief and heuristic which always yield 0
 * 
 * @tparam STATE the type of the state involved. It actually doesn't matter
 */
template <typename STATE>
class ZeroHeuristic: public IHeuristic<STATE> {
public:
    virtual cost_t getHeuristic(const STATE& current, const STATE& goal) {
        return 0;
    }
    virtual bool isAdmissible() const {
        return true;
    }
    virtual bool isConsistent() const {
        return false;
    }
};

}

#endif