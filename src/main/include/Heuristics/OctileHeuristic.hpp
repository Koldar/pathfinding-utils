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
template <typename STATE>
class OctileHeuristic : public IHeuristic<STATE> {
public:
    using This = OctileHeuristic<STATE>;
    using Super = IHeuristic<STATE>;
    using Super::getHeuristic;
    using IMemorable::getByteMemoryOccupied;
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
    virtual cost_t getHeuristic(const STATE& current, const STATE* goal) {
        xyLoc distance = current.getFirstData().getDistance(goal->getFirstData());
        auto minC = distance.getMinCoordinate();
        auto maxC = distance.getMaxCoordinate();
        //I use floor to ensure that the estimate is still admissible
        finest("current=", current, " goal", *goal, "maxC", maxC, "minC", minC);
        return (maxC - minC) + floor(minC * M_SQRT2);
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