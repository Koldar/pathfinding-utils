#ifndef _PATHFINDING_UTILS_STANDARD_LOCATION_GOAL_CHECKER_HEADER__
#define _PATHFINDING_UTILS_STANDARD_LOCATION_GOAL_CHECKER_HEADER__

#include "IGoalChecker.hpp"
#include <cpp-utils/log.hpp>

namespace pathfinding::search {

    /**
     * 
     * @brief A generic goal checker which check if the node location is the same fo the target
     * 
     */
    template <typename STATE>
    class StandardLocationGoalChecker: public IGoalChecker<STATE> {
        typedef StandardLocationGoalChecker<STATE> StandardLocationGoalCheckerInstance;
        typedef IGoalChecker<STATE> Super;
    public:
        StandardLocationGoalChecker() {
            debug("goal checker constructed");
        }
        StandardLocationGoalChecker(const StandardLocationGoalCheckerInstance& o) {

        }
        StandardLocationGoalChecker(StandardLocationGoalCheckerInstance&& o) {

        }
        StandardLocationGoalCheckerInstance& operator=(const StandardLocationGoalCheckerInstance& o) {
            return *this;
        }
        StandardLocationGoalCheckerInstance& operator=(StandardLocationGoalCheckerInstance&& o) {
            return *this;
        }
        virtual ~StandardLocationGoalChecker() {

        }
    public:
        virtual bool isGoal(const STATE& state, const STATE* goal) const {
            return state.getPosition() == goal->getPosition();
        }
    public:
        virtual void cleanup() {

        }
    public:
        virtual MemoryConsumption getByteMemoryOccupied() const {
            return sizeof(*this);
        }
};

}

#endif