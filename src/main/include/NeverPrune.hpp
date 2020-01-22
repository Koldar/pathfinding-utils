#ifndef _PATHFINDINGUTILS_NEVERPRUNE_HEADER__
#define _PATHFINDINGUTILS_NEVERPRUNE_HEADER__

#include "IStatePruner.hpp"

namespace pathfinding::search {

    /**
     * @brief never prune
     * 
     * @tparam STATE the type fo the state involved
     */
    template <typename STATE>
    class NeverPrune: public IStatePruner<STATE> {
    public:
        virtual bool shouldPrune(const STATE& state) const {
            return false;
        }
    public:
        virtual void cleanup() {
            
        }
    public:
        virtual MemoryConsumption getByteMemoryOccupied() const {
            return MemoryConsumption{sizeof(*this), MemoryConsumptionEnum::BYTE};
        }
    };

}

#endif