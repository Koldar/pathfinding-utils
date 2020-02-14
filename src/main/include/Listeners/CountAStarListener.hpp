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
        virtual void onNewSearchStarted(const STATE& start, const STATE* goal) {

        }
        virtual void onNodePruned(int iteration, const STATE& state) {

        }
        virtual void onNodePoppedFromOpen(int iteration, const STATE& state) {

        }
        virtual void onNodeInOpenListHasBetterG(int iteration, const STATE& state, cost_t inOpenG, cost_t outOpenG) {

        }
        virtual void onNodeInOpenListHasWorseG(int iteration, const STATE& state, cost_t inOpenG, cost_t outOpenG) {

        }
        virtual void onNodeInClosedListHasBetterG(int iteration, const STATE& state, cost_t inCloseG, cost_t outCloseG) {

        }
        virtual void onNodeInClosedListHasWorseG(int iteration, const STATE& state, cost_t inCloseG, cost_t outCloseG) {

        }
        virtual void onNoSolutionFound(int iteration) {
            
        }
        virtual void onNodeExpanded(int iteration, const STATE& s) {
            this->updateNodeExpanded();
        }
        virtual void onNodeGenerated(int iteration, const STATE& s) {
            this->updateNodeGenerated();
        }
        virtual void onStartingComputingHeuristic(int iteration, const STATE& s) {
            this->startHeuristicTimer();
        }
        virtual void onEndingComputingHeuristic(int iteration, const STATE& s) {
            this->stopHeuristicTimer();
        }
        virtual void onSolutionFound(int iteration, const STATE& s) {

        }
        virtual void onStartingComputingSuccessors(int iteration, const STATE& s) {
            Super2::startSuccessorTimer();
        }

        virtual void onEndingComputingSuccessors(int iteration, const STATE& s) {
            Super2::stopSuccessorTimer();
        }
    public:
        void cleanup() {
            Super2::cleanup();
        }
    };

}

#endif