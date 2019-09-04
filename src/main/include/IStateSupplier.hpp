#ifndef _ISTATE_HEADER__
#define _ISTATE_HEADER__

#include "IState.hpp"

namespace pathfinding::search {

/**
 * @brief a class whose job is to generate a new search state
 * 
 */
template <typename STATE>
class IStateSupplier {
	/**
	 * @brief get a search state with the given id
	 * 
	 * this function should generate a new IState instance if no state with `id` has never been generated or
	 * the previously existent state with the given `id` otherwise
	 * 
	 * @param id the id of the state which uniquely identifies the state
	 * @return IState& the state with such id
	 */
	virtual STATE& getState(stateid_t id) = 0;
};

}

#endif