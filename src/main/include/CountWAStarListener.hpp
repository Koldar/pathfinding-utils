#ifndef _PATHFINDING_UTILS_COUNT_WASTAR_LISTENER_HEADER__
#define _PATHFINDING_UTILS_COUNT_WASTAR_LISTENER_HEADER__

#include <cpp-utils/NumTracker.hpp>
#include <cpp-utils/Timer.hpp>

#include "CountAStarListener.hpp"
#include "WAStar.hpp"

namespace pathfinding::search {

    template <typename STATE>
    class CountWAStarListener: public WAstarListener<STATE>, public CountAStarListener<STATE> {
        using Super1 = WAstarListener<STATE>;
        using Super2 = CountAStarListener<STATE>;
    public:
        int getNodeExpanded() const {
            return Super2::getNodeExpanded();
        }
        int getNodeGenerated() const {
            return Super2::getNodeGenerated();
        }
        NumTracker<long> getHeuristicTracking() const {
            return Super2::getHeuristicTracking();
        }
    public:
        void onNodeExpanded(const STATE& s) {
            Super2::onNodeExpanded(s);
        }
        void onNodeGenerated(const STATE& s) {
            Super2::onNodeGenerated(s);
        }
        void onStartingComputingHeuristic(const STATE& s) {
            Super2::onStartingComputingHeuristic(s);
        }
        void onEndingComputingHeuristic(const STATE& s) {
            Super2::onEndingComputingHeuristic(s);
        }
    public:
        void cleanup() {
            Super2::cleanup();
        }
    };

}

#endif

