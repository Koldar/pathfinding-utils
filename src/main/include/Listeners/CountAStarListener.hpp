#ifndef _PATHFINDING_UTILS_COUNT_ASTAR_LISTENER_HEADER__
#define _PATHFINDING_UTILS_COUNT_ASTAR_LISTENER_HEADER__

#include <cpp-utils/NumTracker.hpp>
#include <cpp-utils/Timer.hpp>

#include "AstarListener.hpp"
#include "StateCounter.hpp"

namespace pathfinding::search::listeners {

    /**
     * @brief A listener which track the number of nodes expanded or generated, with the timings of the heuristic
     * 
     * @tparam STATE 
     */
    template <typename STATE>
    class CountAStarListener: public AstarListener<STATE>, public StateCounter {
        using This = CountAStarListener<STATE>;
        using Super = AstarListener<STATE>;
        using Super2 = StateCounter;
    public:
        CountAStarListener(): Super2{} {

        }
        virtual ~CountAStarListener() {

        }
        CountAStarListener(const This& o):Super2{o} {

        }
        CountAStarListener(This&& o): Super2{o} {

        }
        This& operator=(const This& o) = default;
        This& operator=(This&& o) = default;
    public:
        virtual void onNodeExpanded(const STATE& s) {
            this->updateNodeExpanded();
        }
        virtual void onNodeGenerated(const STATE& s) {
            this->updateNodeGenerated();
        }
        virtual void onStartingComputingHeuristic(const STATE& s) {
            this->startHeuristicTimer();
        }
        virtual void onEndingComputingHeuristic(const STATE& s) {
            this->stopHeuristicTimer();
        }
        virtual void onSolutionFound(const STATE& s) {

        }
    public:
        void cleanup() {
            Super2::cleanup();
        }
    };

}

#endif