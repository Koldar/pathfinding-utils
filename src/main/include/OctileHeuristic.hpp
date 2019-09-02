#ifndef _OCTILEHEURISTIC_HEADER__
#define _OCTILEHEURISTIC_HEADER__

#include <cmath>
#include "IHeuristic.hpp"
#include "GridMapState.hpp"

namespace pathfinding::search {

/**
 * @brief octile heuristic
 * 
 * The heuristic works lke this:
 * 
 * ```
 * | | | | | | | | | |
 * |N|-|-|-|-| | | | |
 * | | | | | |\| | | |
 * | | | | | | |G| | |
 * ```
 * 
 */
class OctileHeuristic : IHeuristic<GridMapState> {

    virtual cost_t getHeuristic(const GridMapState& current, const GridMapState& goal) {
        xyLoc distance = current.getDistance(goal);
        auto minC = distance.getMinCoordinate();
        auto maxC = distance.getMaxCoordinate();
        //I use floor to ensure that the estimate is still admissible
        return (maxC - minC) + floor(min * M_SQRT2);
    }
    
    virtual bool isAdmissible() const {
        return true;
    }
    
    virtual bool isConsistent() const {
        return true;
    }

};

}
#endif