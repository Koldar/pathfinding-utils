#ifndef _PATHFINDING_UTILS_STANDARD_STATE_EXPANDER_HEADER__
#define _PATHFINDING_UTILS_STANDARD_STATE_EXPANDER_HEADER__

#include <cpp-utils/log.hpp>
#include <cpp-utils/functional.hpp>

#include "IStateExpander.hpp"

namespace pathfinding::search {

    //TODO rename maybe in something like GraphSucccessorStateExpander?
    /**
     * @brief allows to generate the successors of over a graph
     * 
     * @tparam G payload of the underlying weighted directed graph
     */
    template <typename STATE, typename G, typename V, typename E, typename REASON>
    class StandardStateExpander: public IStateCostExpander<STATE, nodeid_t, REASON> {
        using This = StandardStateExpander<STATE, G, V, E, REASON>;
        using Supplier = IStateSupplier<STATE, nodeid_t, REASON>;
    private:
        const cpp_utils::graphs::IImmutableGraph<G, V, E>& graph;
        /**
         * @brief the function we used to convert the edge into costs.
         * 
         * Owned by the class since lambdas tend to go out of scope quickly
         */
        const cpp_utils::function_t<E, cost_t> costFunction;
    public:
        /**
         * @brief Construct a new Graph State Expander object
         * 
         * @param graph the graph we're goingto use to generate the successors
         */ 
        StandardStateExpander(const cpp_utils::graphs::IImmutableGraph<G, V, E>& graph, const cpp_utils::function_t<E, cost_t>& costFunction): graph{graph}, costFunction{costFunction} {
            debug("StandardStateExpander called");
        }
        virtual ~StandardStateExpander() {

        }
        StandardStateExpander(const This& other) : graph{other.graph} {

        }
        This& operator =(const This& other) = delete;
        StandardStateExpander(This&& other) : graph{other.graph} {

        }
        This& operator =(This&& other) {
            this->graph = other.graph;
            return *this;
        }
    public:
        virtual cpp_utils::vectorplus<StateExpanderOutput<STATE>> getSuccessors(const STATE& state, Supplier& supplier) {
            cpp_utils::vectorplus<StateExpanderOutput<STATE>> result{};
            //****************** MOVING *********************
            for (auto outEdge : graph.getOutEdges(state.getPosition())) {
                fine("an outedge ", outEdge, " of ", state, "(", &state, ") goes to", outEdge.getSinkId(), "edge payload of", outEdge.getPayload());
                result.add(StateExpanderOutput{
                    supplier.getState(outEdge.getSinkId(), REASON::getFirst()),
                    this->costFunction(outEdge.getPayload())
                });
            }

            return result;
        }
        virtual StateExpanderOutput<STATE> getSuccessor(const STATE& state, int successorNumber, Supplier& supplier) {
            auto outEdge = this->graph.getOutEdge(state.getPosition(), successorNumber);
            return StateExpanderOutput{
                supplier.getState(outEdge.getSinkId(), REASON::getFirst()),
                this->costFunction(outEdge.getPayload()),
            };
        }
        
    public:
        virtual void cleanup() {

        }
    public:
        virtual MemoryConsumption getByteMemoryOccupied() const {
            return sizeof(*this);
        }
    };

}

#endif