#ifndef _ISTATEEXPANDER_HEADER__
#define _ISTATEEXPANDER_HEADER__

#include <cpp-utils/vectorplus.hpp>
#include <cpp-utils/ICleanable.hpp>
#include <cpp-utils/imemory.hpp>
#include "IStateSupplier.hpp"

namespace pathfinding::search {

template <typename STATE, typename... STATE_IMPORTANT_TYPES>
class IStateExpander;

/**
 * @brief a class whose job is to compute all the successors of a given state
 * 
 */
template <typename STATE, typename... STATE_IMPORTANT_TYPES>
class IStateExpander: public ICleanable, IMemorable {
public:
    /**
     * @brief compute the list of successors from a given one
     * 
     * @param state the state whose children we need to compute
     * @param supplier a class whose job is to generate states. It's likely that we will need them
     * @return std::vector<std::pair<STATE&, cost_t>>  a vector containing the successors
     */
    virtual cpp_utils::vectorplus<std::pair<STATE&, cost_t>> getSuccessors(const STATE& state, IStateSupplier<STATE, STATE_IMPORTANT_TYPES...>& supplier) = 0;

    /**
     * @brief generate a single successor
     * 
     * The order of the successor is not defined by this contract. The implementation is required to lay out an order itself
     * 
     * @param state the state whose successor we need to generate
     * @param successorNumber an index of the successor to generate
     * @param supplier state supplier which will concretely generate the state
     * @return std::pair<STATE&, cost_t> a pair where the first index is the successor generate and the second value is the cost of reaching the successor from @c state
     */
    virtual std::pair<STATE&, cost_t> getSuccessor(const STATE& state, int successorNumber, IStateSupplier<STATE, STATE_IMPORTANT_TYPES...>& supplier) = 0;
};

}

#endif