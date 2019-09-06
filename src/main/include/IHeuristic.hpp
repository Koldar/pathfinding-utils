#ifndef _IHEURISTIC_HEADER__
#define _IHEURISTIC_HEADER__

#include <cpp-utils/ICleanable.hpp>
#include <cpp-utils/imemory.hpp>

namespace pathfinding::search {

    using namespace cpp_utils;

/**
 * @brief All classes implementing this interface will be considered heuristics
 * 
 * @tparam STATE the type of state the heuristic can operate on. not all heuristic can work on all state representations
 */
template<typename STATE>
class IHeuristic: public ICleanable, public IMemorable {
public:
    using IMemorable::getByteMemoryOccupied;
public:
    /**
     * @brief Get the Heuristic object
     * 
     * @param current the state whose heuristic we need to evaluate
     * @param goal the state which represents the goal. nullptr if the goal cannot be simply represented with a single state
     * @return cost_t the estimate of the cost of the path from @c currentState till @c goalState
     */
    virtual cost_t getHeuristic(const STATE& current, const STATE* goal) = 0;

    virtual cost_t getHeuristic(const STATE& current, const STATE& goal) {
        return this->getHeuristic(current, &goal);
    }
    /**
     * @brief 
     * 
     * @return true if the heuristic is admissible
     * @return false otherwise  
     */
    virtual bool isAdmissible() const = 0;
    /**
     * @brief 
     * 
     * @return true if the heuristic is consistent
     * @return false otherwise
     */
    virtual bool isConsistent() const = 0;
};

}

#endif