#ifndef _PATHFINDINGUTILS_STATETRACKER_HEADER__
#define _PATHFINDINGUTILS_STATETRACKER_HEADER__

#include <cpp-utils/igraph.hpp>
#include <cpp-utils/adjacentGraph.hpp>

#include "NodePath.hpp"
#include "StateCounter.hpp"

namespace pathfinding::search::listeners {

    using namespace cpp_utils;
    using namespace cpp_utils::graphs;

    /**
     * @brief a class that you can use to keep track of the pathfinding expanded/generated states
     * 
     * @tparam G 
     * @tparam E 
     */
    template <typename G, typename E>
    class StateTracker {
    public:
        using This = StateTracker;
    protected:
        AdjacentGraph<G,statevisited_e,E> perturbatedGraph;
        NodePath currentSolution;
    private:
        template <typename V>
        StateTracker(const IImmutableGraph<G, V, E>& perturbatedGraph): perturbatedGraph{}, currentSolution{} {
            function_t<V, statevisited_e> lambda = [&](const V& e) { return statevisited_e::UNVISITED;};
            auto p = perturbatedGraph.mapVertices(lambda);
            this->perturbatedGraph = *p;
            delete p;
        }
        virtual ~StateTracker() = default;
        StateTracker(const This& o) = default;
        StateTracker(This&& o) = default;
        This& operator =(const This& o) = default;
        This& operator = (This&& o) = default;
    public:
        const IImmutableGraph<G, statevisited_e, E>& getVisitedStates() const {
            return this->perturbatedGraph;
        }
        template <typename STATE>
        void updateNodeExpanded(const STATE& s) {
            this->perturbatedGraph.changeVertexPayload(s.getId(), statevisited_e::EXPANDED);
        }
        template <typename STATE>
        void updateNodeGenerated(const STATE& s) {
            this->perturbatedGraph.changeVertexPayload(s.getId(), statevisited_e::GENERATED);
        }
        template <typename STATE>
        void updateSolution(const STATE& goalReached, const function_t<const STATE&, nodeid_t>& mapper) {
            this->currentSolution = getNodePath(goalReached, mapper);
        }
        virtual void cleanup() {
            for (auto it=this->perturbatedGraph.beginVertices(); it!=this->states.endVertices(); ++it) {
                this->perturbatedGraph.changeVertexPayload(it->first, statevisited_e::UNVISITED);
            }
        }
    };

}

#endif