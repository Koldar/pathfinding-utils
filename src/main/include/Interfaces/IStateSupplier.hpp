#ifndef _ISTATESUPPLIER_HEADER__
#define _ISTATESUPPLIER_HEADER__

#include "ISearchState.hpp"
#include <cpp-utils/pool.hpp>
#include <cpp-utils/imemory.hpp>
#include <cpp-utils/ICleanable.hpp>
#include <memory>

namespace pathfinding::search {

template <typename STATE, typename... STATE_IMPORTANT_TYPES>
class IStateSupplier;

/**
 * @brief a class whose job is to generate a new search state
 * 
 * @tparam STATE the IState implementation we will yield
 * @tparam STATE_IMPORTANT_TYPES additional parameters required to generate a state
 */
template <typename STATE, typename... STATE_IMPORTANT_TYPES>
class IStateSupplier: public ICleanable, IMemorable {
public:
	/**
	 * @brief get a search state without providing an id
	 * 
	 * This will likely to generate a new state almost always
	 * 
	 * @param args additional information required to generate states
	 * @return STATE& the new state
	 */
	virtual STATE& getState(const STATE_IMPORTANT_TYPES&... args) = 0;
};

}

#endif