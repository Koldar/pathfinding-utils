#ifndef _ISTATE_HEADER__
#define _ISTATE_HEADER__

#include "IState.hpp"

namespace pathfinding::search {

/**
 * @brief a class whose job is to generate a new search state
 * 
 * @tparam STATE the IState implementation we will yield
 * @tparam OTHER additional parameters required to generate a state
 */
template <typename STATE, typename... OTHER>
class IStateSupplier {
	/**
	 * @brief get a search state with the given id
	 * 
	 * this function should generate a new IState instance if no state with `id` has never been generated or
	 * the previously existent state with the given `id` otherwise
	 * 
	 * @param id the id of the state which uniquely identifies the state
     * @param args additional information required to generate states
	 * @return IState& the state with such id
	 */
	virtual STATE& getState(stateid_t id, OTHER... args) = 0;
};

}

#endif