#ifndef _ISTATEPRUNER_HEADER__
#define _ISTATEPRUNER_HEADER__

namespace pathfinding::search {

 /**
 * @brief a class whose job is to check if a state under analysis should be pruned for search or not
 * 
 */
template <typename STATE>
class IStatePruner {
public:
    /**
     * @brief check if we should keep the search state of such search should be discarded since it cannot reach the solution
     * 
     * @param state the state involved
     * @return true if the state should be pruned away
     * @return false otherwise
     */
    virtual bool shouldPrune(const STATE& state) const = 0;
};

/**
 * @brief nbever prune
 * 
 * @tparam STATE the type fo the state involved
 */
template <typename STATE>
class NoPruning: public IStatePruner<STATE> {
    virtual bool shouldPrune(const STATE& state) const {
        return false;
    }
};

/**
 * @brief prune the state only if its expanded flag is set to true
 * 
 * @tparam STATE the type of the state involved
 */
template <typename STATE>
class PruneIfExpanded: public IStatePruner<STATE> {
    virtual bool shouldPrune(const STATE& state) const {
        return state.isExpanded();
    }
};

}

#endif