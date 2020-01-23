#ifndef _PATHFINDINGUTILS_STATECOUNTER_HEADER__
#define _PATHFINDINGUTILS_STATECOUNTER_HEADER__

#include <cpp-utils/NumTracker.hpp>
#include <cpp-utils/Timer.hpp>

namespace pathfinding::search::listeners {

    /**
     * @brief count the number of expanded and generated states
     * 
     */
    class StateCounter: public ICleanable {
    public:
        using This = StateCounter;
    private:
        int nodeExpanded;
        int nodeGenerated;
        NumTracker<long> heuristicTime;
        Timer heuristicTimer;
    public:
        StateCounter(): nodeExpanded{0}, nodeGenerated{0}, heuristicTime{}, heuristicTimer{false} {

        }
        virtual ~StateCounter() {

        }
        StateCounter(const This& o) = default;
        StateCounter(This&& o) = default;
        This& operator =(const This& o) = default;
        This& operator =(This&& o) = default;
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
        void updateNodeExpanded() {
            this->nodeExpanded += 1;
        }
        void updateNodeGenerated() {
            this->nodeGenerated += 1;
        }
        void startHeuristicTimer() {
            this->heuristicTimer.cleanup();
            this->heuristicTimer.start();
        }
        void stopHeuristicTimer() {
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