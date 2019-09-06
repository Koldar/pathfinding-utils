#ifndef _MANHATTANHEURISTIC_HEADER__
#define _MANHATTANHEURISTIC_HEADER__

#include <cmath>
#include "IHeuristic.hpp"
#include "GridMapState.hpp"
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
class ManhattanHeuristic : public IHeuristic<GridMapState> {
private:
    const pathfinding::maps::GridBranching branching;
    bool admissible;
    bool consistent;
public:
    using IHeuristic<GridMapState>::getHeuristic;
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

    virtual cost_t getHeuristic(const GridMapState& current, const GridMapState* goal) {
        xyLoc distance = current.getPosition().getDistance(goal->getPosition());
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