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



}

#endif 