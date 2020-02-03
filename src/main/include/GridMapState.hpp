#ifndef _GRIDMAPSTATE_HEADER__
#define _GRIDMAPSTATE_HEADER__

#include "ISearchAlgorithm.hpp"
#include "ISearchState.hpp"
#include "GridBranching.hpp"
#include "GraphState.hpp"
#include "IStateSupplier.hpp"
#include "IStateExpander.hpp"
#include "IGoalChecker.hpp"
#include "xyLoc.hpp"
#include "GraphlessState.hpp"
#include "AbstractSimpleWeightedDirectedGraphStateSupplier.hpp"
#include <cpp-utils/StaticPriorityQueue.hpp>
#include <cpp-utils/igraph.hpp>
#include <cpp-utils/vectorplus.hpp>
#include <utility>

namespace pathfinding::search {

    /**
     * @brief a search state over a map we know it's a grid map
     * 
     * In this state there is only x and y coordinates
     * 
     */
    template <typename REASON>
    using GridMapState = GraphlessState<xyLoc, REASON>;

    template <typename G, typename REASON>
    class GridMapStateSupplier: public AbstractSimpleWeightedDirectedGraphStateSupplier<GridMapState<REASON>, G, xyLoc, cost_t, REASON> {
    public:
        using State = GridMapState<REASON>;
        using This = GridMapStateSupplier<G, REASON>;
        using Super = AbstractSimpleWeightedDirectedGraphStateSupplier<State, G, xyLoc, cost_t, REASON>;
    protected:
        virtual stateid_t generateStateId(nodeid_t location, const REASON& reason) {
            return location;
        }

        virtual State generateNewInstance(stateid_t id, nodeid_t location, const REASON& reason) {
            return State{id, location, this->graph.getVertex(location), reason};
        }
    public:
        GridMapStateSupplier(const IImmutableGraph<G, xyLoc, cost_t>& graph): Super{graph} {

        }
        GridMapStateSupplier(const This& other): Super{other} {

        }
        GridMapStateSupplier(const This&& other): Super{::std::move(other)} {
            
        }
        virtual ~GridMapStateSupplier() {

        }
        This& operator =(const This& other) {
            Super::operator=(other);
            return *this;
        }
        This& operator =(This&& other) {
            Super::operator=(other);
            return *this;
        }
    };

    /**
     * @brief an expander that uses an underlying graph to compute successors
     * 
     * The successors of a state are the successors of the vertex representing the state
     * 
     * @note
     * Note that the state id of a search state is **the same** of the vertex id of a node. This
     * means that the state space is as large as the underlying graph!
     * 
     * @tparam G paylaod of the whole graph
     * @tparam V paylaod of each single vertex
     */
    template <typename G>
    class SimpleGridMapStateExpander: public IStateExpander<GridMapState<bool>, nodeid_t, bool> {
    public:
        using State = GridMapState<bool>;
        using This = SimpleGridMapStateExpander<G>;
        using Super = IStateExpander<State, nodeid_t>;
    private:
        const cpp_utils::graphs::IImmutableGraph<G, xyLoc, cost_t>& graph;
    public:
        SimpleGridMapStateExpander(const cpp_utils::graphs::IImmutableGraph<G, xyLoc, cost_t>& graph): graph{graph} {

        }
        virtual ~SimpleGridMapStateExpander() {

        }
        SimpleGridMapStateExpander(const This& other) : graph{other.graph} {

        }
        This& operator =(const This& other) {
            this->graph = other.graph;
            return *this;
        }
        SimpleGridMapStateExpander(This&& other): graph{other.graph} {

        }
        This& operator =(This&& other) {
            this->graph = other.graph;
            return *this;
        }
    public:
        virtual cpp_utils::vectorplus<std::pair<State&, cost_t>> getSuccessors(const State& state, IStateSupplier<State, nodeid_t, bool>& supplier) {
            cpp_utils::vectorplus<std::pair<State&, cost_t>> result{};
            for (auto outEdge : graph.getOutEdges(state.getId())) {
                fine("an outedge ", outEdge, " of ", state, "(", &state, ") goes to", outEdge.getSinkId(), "edge payload of", outEdge.getPayload());
                result.add(std::pair<State&, cost_t>{
                    supplier.getState(outEdge.getSinkId(), true),
                    outEdge.getPayload()
                });
            }
            return result;
        }
        virtual std::pair<State&, cost_t> getSuccessor(const State& state, int successorNumber, IStateSupplier<State, nodeid_t, bool>& supplier) {
            auto outEdge = this->graph.getOutEdge(state.getId(), static_cast<moveid_t>(successorNumber));
            return std::pair<State&, cost_t>{supplier.getState(outEdge.getSinkId(), true), outEdge.getPayload()};
        }
    public:
        virtual void cleanup() {

        }
    public:
        virtual MemoryConsumption getByteMemoryOccupied() const {
            return MemoryConsumption{sizeof(*this), MemoryConsumptionEnum::BYTE};
        }
    };

    class SAPFGridMapGoalChecker: public IGoalChecker<GridMapState<bool>> {
    public:
        using State = GridMapState<bool>;
    public:
        virtual bool isGoal(const State& state, const State* goal) const {
            return state.getPosition() == goal->getPosition();
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