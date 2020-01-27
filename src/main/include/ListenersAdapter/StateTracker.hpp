#ifndef _PATHFINDINGUTILS_STATETRACKER_HEADER__
#define _PATHFINDINGUTILS_STATETRACKER_HEADER__

#include <cpp-utils/igraph.hpp>
#include <cpp-utils/adjacentGraph.hpp>
#include <cpp-utils/strings.hpp>

#include "utils.hpp"
#include "NodePath.hpp"
#include "StateCounter.hpp"

namespace pathfinding::search::listeners {

    using namespace cpp_utils;
    using namespace cpp_utils::graphs;
    using namespace pathfinding;

    template <typename V>
    class VertexInfo {
    public:
        using This = VertexInfo<V>;
    public:
        int expandedTimes;
        statevisited_e state;
        V payload;
    public:
        VertexInfo(int expandedTimes, statevisited_e state, V payload): expandedTimes{expandedTimes}, state{state}, payload{payload} {

        }
        virtual ~VertexInfo() {

        }
        VertexInfo(const This& o) = default;
        VertexInfo(This&& o) = default;
        This& operator = (const This& o) = default;
        This& operator = (This&& o) = default;
    public:
        friend bool operator ==(const This& a, const This& b) {
            return a.payload == b.payload;
        }
        friend std::ostream& operator <<(std::ostream& out, const This& a) {
            return cpp_utils::owcout(out, "{", a.payload, "}");
        }
    };

    /**
     * @brief a class that you can use to keep track of the pathfinding expanded/generated states
     * 
     * It can track a solution as well. such tracking si done by **copying** the solution, so some performance smay be lost there.
     * 
     * @tparam G 
     * @tparam E 
     */
    template <typename G, typename V, typename E>
    class StateTracker {
    public:
        using This = StateTracker;
        using VertexInfoReal = VertexInfo<V>;
    protected:
        AdjacentGraph<G, VertexInfoReal, E> perturbatedGraphState;
        NodePath currentSolution;
    public:
        StateTracker(const IImmutableGraph<G, V, E>& perturbatedGraph): perturbatedGraphState{}, currentSolution{} {
            static function_t<V, VertexInfoReal> lambda = [&](auto v) { return VertexInfoReal{0, statevisited_e::UNVISITED, v};};
            auto p = perturbatedGraph.mapVertices(lambda);
            this->perturbatedGraphState = *p;
            delete p;
        }
        virtual ~StateTracker() = default;
        StateTracker(const This& o) = default;
        StateTracker(This&& o) = default;
        This& operator =(const This& o) = default;
        This& operator = (This&& o) = default;
    public:
        const IImmutableGraph<G, VertexInfoReal, E>& getVisitedStates() const {
            return this->perturbatedGraphState;
        }
        const NodePath& getSolution() const {
            return this->currentSolution;
        }
        template <typename STATE>
        void updateNodeExpanded(const STATE& s) {
            static bifunction_t<nodeid_t, VertexInfoReal, VertexInfoReal> mapper = [&](auto n, auto v) { return VertexInfoReal{v.expandedTimes + 1, statevisited_e::EXPANDED, v.payload};};
            this->perturbatedGraphState.changeVertexPayload(s.getPosition(), mapper);
        }
        template <typename STATE>
        void updateNodeGenerated(const STATE& s) {
            static bifunction_t<nodeid_t, VertexInfoReal, VertexInfoReal> mapper = [&](auto n, auto v) { return VertexInfoReal{v.expandedTimes, statevisited_e::GENERATED, v.payload};};
            this->perturbatedGraphState.changeVertexPayload(s.getPosition(), mapper);
        }
        template <typename STATE>
        void updateSolution(const STATE& goalReached, const function_t<const STATE&, nodeid_t>& mapper) {
            this->currentSolution = utils::getNodePath(goalReached, mapper);
        }
        virtual void cleanup() {
            static bifunction_t<nodeid_t, VertexInfoReal, VertexInfoReal> lambda = [&](auto id, VertexInfoReal v) { return VertexInfoReal{0, statevisited_e::UNVISITED, v.payload};};
            for (auto it=this->perturbatedGraphState.beginVertices(); it!=this->perturbatedGraphState.endVertices(); ++it) {
                this->perturbatedGraphState.changeVertexPayload(it->first, lambda);
            }
        }
    };

}

#endif