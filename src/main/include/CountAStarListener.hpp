#ifndef _PATHFINDING_UTILS_COUNT_ASTAR_LISTENER_HEADER__
#define _PATHFINDING_UTILS_COUNT_ASTAR_LISTENER_HEADER__

#include <cpp-utils/NumTracker.hpp>
#include <cpp-utils/Timer.hpp>

#include "AStar.hpp"

namespace pathfinding::search {

    /**
     * @brief A listener which track the number of nodes expanded or generated, with the timings of the heuristic
     * 
     * @tparam STATE 
     */
    template <typename STATE>
    class CountAStarListener: public AstarListener<STATE> {
        typedef CountAStarListener<STATE> This;
        typedef AstarListener<STATE> Super;
    private:
        int nodeExpanded;
        int nodeGenerated;
        /**
         * @brief times used by the algorithm to compute the heuristic
         * 
         */
        NumTracker<long> heuristicTime;
        Timer heuristicTimer;
    public:
        CountAStarListener(): nodeExpanded{0}, nodeGenerated{0}, heuristicTime{}, heuristicTimer{false} {

        }
        virtual ~CountAStarListener() {

        }
        CountAStarListener(const This& o): nodeExpanded{o.nodeExpanded}, nodeGenerated{o.nodeGenerated}, heuristicTime{o.heuristicTime}, heuristicTimer{o.heuristicTimer} {

        }
        CountAStarListener(This&& o): nodeExpanded{o.nodeExpanded}, nodeGenerated{o.nodeGenerated}, heuristicTime{o.heuristicTime}, heuristicTimer{o.heuristicTimer} {

        }
        This& operator=(const This& o) {
            this->nodeExpanded = o.nodeExpanded;
            this->nodeGenerated = o.nodeGenerated;
            this->heuristicTime = o.heuristicTime;
            this->heuristicTimer = o.heuristicTimer;

            return *this;
        }
        This& operator=(This&& o) {
            this->nodeExpanded = o.nodeExpanded;
            this->nodeGenerated = o.nodeGenerated;
            this->heuristicTime = o.heuristicTime;
            this->heuristicTimer = o.heuristicTimer;
            
            return *this;
        }
    public:
        int getNodeExpanded() const {
            return this->nodeExpanded;
        }
        int getNodeGenerated() const {
            return this->nodeGenerated;
        }
        NumTracker<long> getHeuristicTracking() const {
            return this->heuristicTime;
        }
    public:
        void onNodeExpanded(const STATE& s) {
            this->nodeExpanded += 1;
        }
        void onNodeGenerated(const STATE& s) {
            this->nodeGenerated += 1;
        }
        void onStartingComputingHeuristic(const STATE& s) {
            this->heuristicTimer.cleanup();
            this->heuristicTimer.start();
        }
        void onEndingComputingHeuristic(const STATE& s) {
            this->heuristicTimer.stop();
            this->heuristicTime.update(this->heuristicTimer.getElapsedMicroSeconds().toLong());
        }
    public:
        void cleanup() {
            this->nodeExpanded = 0;
            this->nodeGenerated = 0;
            this->heuristicTime.cleanup();
            this->heuristicTimer.cleanup();
        }
    };
}

#endif