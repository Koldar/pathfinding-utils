#ifndef _PATHFINDINGUTILS_STATEEXPANDEROUTPUT_HEADER__
#define _PATHFINDINGUTILS_STATEEXPANDEROUTPUT_HEADER__

#include <iostream>

#include "types.hpp"

namespace pathfinding::search {

    /**
     * @brief represents the base class that every IStateExpander should output in their getSuccessors implementation
     * 
     * @tparam STATE 
     */
    template <typename STATE>
    class StateExpanderOutput {
    public:
        using This = StateExpanderOutput<STATE>;
    protected:
        STATE& state;
        cost_t costToReachState;
    public:
        StateExpanderOutput(STATE& state, cost_t costToReachState): state{state}, costToReachState{costToReachState} {

        }
        virtual ~StateExpanderOutput() {

        }
        StateExpanderOutput(const This& o): state{o.state}, costToReachState{o.costToReachState} {

        }
        StateExpanderOutput(This&& o): state{o.state}, costToReachState{o.costToReachState} {

        }
        This& operator = (const This& o) {
            this->state = o.state;
            this->costToReachState = o.costToReachState;
            return *this;
        }
        This& operator = (This&& o) {
            this->state = o.state;
            this->costToReachState = o.costToReachState;
            return *this;
        }
    public:
        /**
         * @brief retrieve the successor
         * 
         * @return STATE& 
         */
        STATE& getState() {
            return this->state;
        }
        /**
         * @brief retrieve the successor (as unmodifiable)
         * 
         * @return const STATE& 
         */
        const STATE& getState() const {
            return this->state;
        }
        /**
         * @brief retrieve the cost we need to pay in order to reach the successor from the parent
         * 
         * @return cost_t 
         */
        cost_t getCostToReachState() const {
            return this->costToReachState;
        }
    public:
        friend std::ostream& operator <<(std::ostream& out, const This& a) {
            return out << "{ state=" << a.state << " cost=" << a.costToReachState << "}";
        }
        friend bool operator ==(const This& a, const This& b) {
            return a.state == b.state && a.costToReachState == b.costToReachState;
        }
    };

}

#endif

