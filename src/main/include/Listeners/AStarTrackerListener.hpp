#ifndef _PATHFINDINGUTILS_ASTARTRACKERLISTENER_HEADER__
#define _PATHFINDINGUTILS_ASTARTRACKERLISTENER_HEADER__

#include <cpp-utils/AbstractEnum.hpp>

#include "CountAStarListener.hpp"
#include "statevisited_e.hpp"
#include "StateCounter.hpp"
#include "StateTracker.hpp"

namespace pathfinding::search::listeners {

    using namespace cpp_utils;

    /**
     * @brief a listener that keeps track of the state we have expanded, and the optimal path generated
     * 
     * the listener allows you to keep track of heuristic average timings, set of node
     * expanded and the first solution an algorithm has found
     * 
     * @tparam STATE type of the A* state
     */
    template <typename G, typename V, typename E, typename STATE>
    class AstarTrackerListener: public AstarListener<STATE>, public StateCounter, public StateTracker<G, V, E> {
    public:
        using This = AstarTrackerListener<G, V, E, STATE>;
        using Super1 = StateCounter;
        using Super2 = StateTracker<G, V, E>;
    public:
        AstarTrackerListener(const IImmutableGraph<G,V,E>& originalGraph): Super1{}, Super2{originalGraph} {
        }
        virtual ~AstarTrackerListener() = default;
        AstarTrackerListener(const This& o) = default;
        AstarTrackerListener(This&& o) = default;
        This& operator =(const This& o) = default;
        This& operator = (This&& o) = default;
    public:
        virtual void onNodeExpanded(int iteration, const STATE& s) {
            Super1::updateNodeExpanded();
            Super2::updateNodeExpanded(s);
        }
        virtual void onNodeGenerated(int iteration, const STATE& s) {
            Super1::updateNodeGenerated();
            Super2::updateNodeGenerated(s);
        }
        virtual void onStartingComputingHeuristic(int iteration, const STATE& s) {
            Super1::startHeuristicTimer();
        }
        virtual void onEndingComputingHeuristic(int iteration, const STATE& s) {
            Super1::stopHeuristicTimer();
        }
        virtual void onSolutionFound(int iteration, const STATE& s) {
            static function_t<STATE, nodeid_t> mapper = [&](auto s) { return s.getPosition();};
            Super2::updateSolution(s, mapper);
        }
    public:
        virtual void cleanup() {
            Super1::cleanup();
            Super2::cleanup();
        }

    };

}

#endif