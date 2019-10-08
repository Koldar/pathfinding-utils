#ifndef _MANHATTANHEURISTIC_HEADER__
#define _MANHATTANHEURISTIC_HEADER__

#include <cmath>
#include "IHeuristic.hpp"
#include "GraphlessState.hpp"
#include <cpp-utils/exceptions.hpp>

namespace pathfinding::search {

/**
 * @brief manhattan heuristic
 * 
 * The heuristic works lke this:
 * 
 * ```
 * | | | | | | | | | |
 * |N|*|*|*|*|*|*| | |
 * | | | | | | |*| | |
 * | | | | | | |G| | |
 * ```
 * 
 */
class ManhattanHeuristic : public IHeuristic<GraphlessState<xyLoc>> {
private:
    const pathfinding::maps::GridBranching branching;
    bool admissible;
    bool consistent;
public:
    using IHeuristic<GraphlessState<xyLoc>>::getHeuristic;
public:
    ManhattanHeuristic(const pathfinding::maps::GridBranching branching) : branching{branching}, admissible{false}, consistent{false} {
        switch (branching) {
            case pathfinding::maps::GridBranching::FOUR_CONNECTED: {
                admissible = true;
                consistent = true;
                break;
            }
            case pathfinding::maps::GridBranching::EIGHT_CONNECTED: {
                admissible = false;
                consistent = true;
                break;
            }
            default: {
                throw cpp_utils::exceptions::InvalidScenarioException<pathfinding::maps::GridBranching>(branching);
            }
        }
    }

    virtual cost_t getHeuristic(const GraphlessState<xyLoc>& current, const GraphlessState<xyLoc>* goal) {
        xyLoc distance = current.getFirstData().getDistance(goal->getFirstData());
        return distance.x + distance.y;
    }
    
    virtual bool isAdmissible() const {
        return admissible;
    }
    
    virtual bool isConsistent() const {
        return consistent;
    }
public:
    virtual MemoryConsumption getByteMemoryOccupied() const {
        return MemoryConsumption{sizeof(*this), MemoryConsumptionEnum::BYTE};
    }
public:
    virtual void cleanup() {
        
    }
};

}
#endif