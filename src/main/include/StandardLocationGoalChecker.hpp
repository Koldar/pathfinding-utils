#ifndef _PATHFINDING_UTILS_STANDARD_LOCATION_GOAL_CHECKER_HEADER__
#define _PATHFINDING_UTILS_STANDARD_LOCATION_GOAL_CHECKER_HEADER__

namespace pathfinding::search {

    /**
     * 
     * @brief A generic goal checker which check if the node location is the same fo the target
     * 
     */
    template <typename STATE>
    class StandardLocationGoalChecker: public IGoalChecker<STATE> {
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