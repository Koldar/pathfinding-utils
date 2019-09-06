#ifndef _IGOALCHECKER_HEADER__
#define _IGOALCHECKER_HEADER__

#include <cpp-utils/ICleanable.hpp>
#include <cpp-utils/imemory.hpp>

namespace pathfinding::search {

template <typename STATE>
class IGoalChecker: public ICleanable, IMemorable {
public:
    /**
     * @brief check if a state is actually a goal of this search
     * 
     * @param state the state to consider
     * @param goal the goal of this search
     * @return true if @c state is a goal
     * @return false otherwise
     */
    virtual bool isGoal(const STATE& state, const STATE* goal) const = 0;
};

}

#endif