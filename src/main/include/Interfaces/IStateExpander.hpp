#ifndef _PATHFINDINGUTILS_ISTATEEXPANDER_HEADER__
#define _PATHFINDINGUTILS_ISTATEEXPANDER_HEADER__

#include <tuple>

#include <cpp-utils/vectorplus.hpp>
#include <cpp-utils/ICleanable.hpp>
#include <cpp-utils/imemory.hpp>
#include "IStateSupplier.hpp"

#include "ISearchState.hpp"
#include "StateExpanderOutput.hpp"

namespace pathfinding::search {

    template <typename STATE, typename SUCCESSORS_OUTPUT, typename... STATE_IMPORTANT_TYPES>
    class IStateExpander;

    /**
     * @brief a class whose job is to compute all the successors of a given state
     * 
     * @tparam STATE the state this expander generates (in some way or another)
     * @tparam SUCCESSORS_OUTPUT the output the function getSuccessors will have.
     * @tparam STATE_IMPORTANT_TYPES list of additional variables you need to supply to the associated state supplier in order to generate a state. These parameters will be essential to understand what are the compatible suppliers for this expander
     */
    template <typename STATE, typename SUCCESSORS_OUTPUT, typename... STATE_IMPORTANT_TYPES>
    class IStateExpander: public ICleanable, IMemorable {
    public:
        /**
         * @brief compute the list of successors from a given one
         * 
         * @param state the state whose children we need to compute
         * @param supplier a class whose job is to generate states. It's likely that we will need them
         * @return std::vector<SUCCESSORS_OUTPUT>  a vector containing the successors
         */
        virtual cpp_utils::vectorplus<SUCCESSORS_OUTPUT> getSuccessors(const STATE& state, IStateSupplier<STATE, STATE_IMPORTANT_TYPES...>& supplier) = 0;

        /**
         * @brief generate a single successor
         * 
         * The order of the successor is not defined by this contract. The implementation is required to lay out an order itself
         * 
         * @param state the state whose successor we need to generate
         * @param successorNumber an index of the successor to generate
         * @param supplier state supplier which will concretely generate the state
         * @return std::tuple<STATE&, cost_t, SUCCESSORS_OUTPUT> a tuple where the first index is the successor generate and the second value is the cost of reaching the successor from @c state
         */
        virtual SUCCESSORS_OUTPUT getSuccessor(const STATE& state, int successorNumber, IStateSupplier<STATE, STATE_IMPORTANT_TYPES...>& supplier) = 0;
    };

    /**
     * @brief a standard interface where the state expander generates pairs for getSuccessors functions.
     * 
     * This implementation is pretty popular since, normally, in a search algorithm, when we compute the successors of a state, what we are interested in are the actual successors and the cost we need to pay to go from the parent to the given successor: this is precisely the information contained in ::StateExpanderOutput
     * 
     */
    template <typename STATE, typename... STATE_IMPORTANT_TYPES>
    using IStateCostExpander = IStateExpander<STATE, StateExpanderOutput<STATE>, STATE_IMPORTANT_TYPES...>;

}

#endif