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
class ManhattanHeuristic : IHeuristic<GridMapState> {
private:
    const GridBranching branching;
    bool admissible;
    bool consistent;
public:
    ManhattanHeuristic(const GridBranching branching) : branching{branching}, admissible{false}, consistent{false} {
        switch (branching) {
            case GridBranching::FOUR_CONNECTED: {
                admissible = true;
                consistent = true;
                break;
            }
            case GridBranching::EIGHT_CONNECTED: {
                admissible = false;
                consistent = true;
                break;
            }
            default: {
                throw cpp_utils::exceptions::InvalidScenarioException<GridBranching>(branching);
            }
        }
    }

    virtual cost_t getHeuristic(const GridMapState& current, const GridMapState& goal) {
        xyLoc distance = current.getPosition().getDistance(goal.getPosition());
        return distance.x + distance.y;
    }
    
    virtual bool isAdmissible() const {
        return admissible;
    }
    
    virtual bool isConsistent() const {
        return consistent;
    }

};

}
#endif