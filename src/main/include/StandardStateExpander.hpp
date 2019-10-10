#ifndef _PATHFINDING_UTILS_STANDARD_STATE_EXPANDER_HEADER__
#define _PATHFINDING_UTILS_STANDARD_STATE_EXPANDER_HEADER__

namespace pathfinding::search {

    /**
     * @brief allows to generate the successors of over a graph
     * 
     * @tparam G payload of the underlying weighted directed graph
     */
    template <typename STATE, typename G, typename V, typename E=cost_t, cost_t(*GET_COST_FUNCTION)(const E&) =cpp_utils::identity<cost_t>::value>
    class StandardStateExpander: public IStateExpander<STATE, nodeid_t> {
        typedef StandardStateExpander<STATE, G, V, E, GET_COST_FUNCTION> StandardStateExpanderInstance;
    private:
        const cpp_utils::graphs::IImmutableGraph<G, V, E>& graph;
    public:
        /**
         * @brief Construct a new Graph State Expander object
         * 
         * @param graph the graph we're goingto use to generate the successors
         */ 
        StandardStateExpander(const cpp_utils::graphs::IImmutableGraph<G, V, E>& graph): graph{graph} {

        }
        virtual ~StandardStateExpander() {

        }
        StandardStateExpander(const StandardStateExpanderInstance& other) : graph{other.graph} {

        }
        StandardStateExpanderInstance& operator =(const StandardStateExpanderInstance& other) = delete;
        StandardStateExpander(StandardStateExpanderInstance&& other) : graph{other.graph} {

        }
        StandardStateExpanderInstance& operator =(StandardStateExpanderInstance&& other) {
            this->graph = other.graph;
            return *this;
        }
    public:
        virtual cpp_utils::vectorplus<std::pair<STATE&, cost_t>> getSuccessors(const STATE& state, IStateSupplier<STATE, nodeid_t>& supplier) {
            cpp_utils::vectorplus<std::pair<STATE&, cost_t>> result{};
            //****************** MOVING *********************
            for (auto outEdge : graph.getOutEdges(state.getPosition())) {
                fine("an outedge ", outEdge, " of ", state, "(", &state, ") goes to", outEdge.getSinkId(), "edge payload of", outEdge.getPayload());
                result.add(std::pair<STATE&, cost_t>{
                    supplier.getState(outEdge.getSinkId()),
                    GET_COST_FUNCTION(outEdge.getPayload())
                });
            }

            return result;
        }
        virtual std::pair<STATE&, cost_t> getSuccessor(const STATE& state, int successorNumber, IStateSupplier<STATE, nodeid_t>& supplier) {
            auto outEdge = this->graph.getOutEdge(state.getPosition(), successorNumber);
            return std::pair<STATE&, cost_t>{
                supplier.getState(outEdge.getSinkId()),
                GET_COST_FUNCTION(outEdge.getPayload())
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