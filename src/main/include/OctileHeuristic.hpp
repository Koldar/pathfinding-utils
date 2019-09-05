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
class OctileHeuristic : public IHeuristic<GridMapState> {
    using IHeuristic<GridMapState>::getHeuristic;
private:
    const pathfinding::maps::GridBranching branching;
    bool admissible;
    bool consistent;
public:
    OctileHeuristic(pathfinding::maps::GridBranching branching): branching{branching} {
        switch (branching) {
            case pathfinding::maps::GridBranching::FOUR_CONNECTED: {
                admissible = true;
                consistent = true;
                break;
            }
            case pathfinding::maps::GridBranching::EIGHT_CONNECTED: {
                admissible = true;
                consistent = true;
                break;
            }
            default: {
                throw cpp_utils::exceptions::InvalidScenarioException<pathfinding::maps::GridBranching>(branching);
            }
        }
    }
public:
    virtual cost_t getHeuristic(const GridMapState& current, const GridMapState* goal) {
        xyLoc distance = current.getPosition().getDistance(goal->getPosition());
        auto minC = distance.getMinCoordinate();
        auto maxC = distance.getMaxCoordinate();
        //I use floor to ensure that the estimate is still admissible
        return (maxC - minC) + floor(minC * M_SQRT2);
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