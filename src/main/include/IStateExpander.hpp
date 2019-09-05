#ifndef _ISTATEEXPANDER_HEADER__
#define _ISTATEEXPANDER_HEADER__

#include <cpp-utils/vectorplus.hpp>
#include <cpp-utils/ICleanable.hpp>
#include "IStateSupplier.hpp"

namespace pathfinding::search {

template <typename STATE, typename... OTHER>
class IStateExpander;

/**
 * @brief a class whose job is to compute all the successors of a given state
 * 
 */
template <typename STATE, typename... OTHER>
class IStateExpander: public ICleanable {
public:
    /**
     * @brief compute the list of successors from a given one
     * 
     * @param state the state whose children we need to compute
     * @param supplier a class whose job is to generate states. It's likely that we will need them
     * @return std::vector<std::pair<STATE&, cost_t>>  a vector containing the successors
     */
    virtual cpp_utils::vectorplus<std::pair<STATE&, cost_t>> getSuccessors(const STATE& state, IStateSupplier<STATE, OTHER...>& supplier) = 0;
};

}

#endif