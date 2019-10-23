#ifndef _ISEARCHALGORITHM_HEADER__
#define _ISEARCHALGORITHM_HEADER__

#include <limits>
#include <cpp-utils/imemory.hpp>
#include <cpp-utils/wrapped_number.hpp>
#include <cpp-utils/exceptions.hpp>
#include <cpp-utils/macros.hpp>
#include <cpp-utils/commons.hpp>
#include <cpp-utils/vectorplus.hpp>
#include <cpp-utils/igraph.hpp>
#include "types.hpp"
#include "ISolutionPath.hpp"


namespace pathfinding::search {

using namespace cpp_utils;

/**
 * @brief thrown when the solution couldn't be found in the 
 * 
 */
class SolutionNotFoundException : public exceptions::AbstractException {
    
};

/**
 * @brief an algorithm that given a start state goes till a goal one
 * 
 * @tparam STATE the search state this search algorithm consider
 * @tparam STATE_IN_SOLUTION the base parameter we will put inside as a primary template of ISolutionPath
 * tparam CONST_REF the constant reference of a STATE
 */
template <typename STATE, typename STATE_IN_SOLUTION=const STATE*, typename CONST_REF=const STATE&>
class ISearchAlgorithm {
protected:
    /**
     * @brief after a search have been completed, this instruction tells how to effectively build a solution path starting from the goal fetched
     * 
     * @param start the start state of the search
     * @param actualGoal goal found during the search
     * @param goal goal input by the user
     * @return SolutionPath<STATE> list of state we need to traverse in order to reach the goal from the start state
     */
    virtual std::unique_ptr<ISolutionPath<STATE_IN_SOLUTION, CONST_REF>> buildSolutionFromGoalFetched(CONST_REF start, CONST_REF actualGoal, const STATE* goal) = 0;
    /**
     * @brief after a search have been completed, this instruction tells how to effectively build the solution cost starting from the goal fetched
     * 
     * @param start the start state of the search
     * @param actualGoal goal found during the search
     * @param goal goal input by the user
     * @return cost_t cost of the solution for reaching @c actualGoal from @c start
     */
    virtual cost_t getSolutionCostFromGoalFetched(CONST_REF start, CONST_REF actualGoal, const STATE* goal) const = 0;
    /**
     * @brief perform the actual search
     * 
     * @param start the initial state
     * @param goal the goal we want to reach. nullptr if we cannot represent in a concrete way the goal
     * @return const STATE& the goal we have retrieved.
     * @throw SolutionNotFoundException if we were unable to fetch the goal
     */
    virtual CONST_REF performSearch(STATE& start, const STATE* goal) = 0;
public:
    /**
     * @brief steps to execute before starting the search
     * 
     * @param start pointer of the start state the search start into. Can be null
     * @param goal the goal we want to reach. If the goal cannot be represented via a state, you can set it to null
     * 
     */
    virtual void setupSearch(const STATE* start, const STATE* goal) = 0;
    /**
     * @brief steps to execute after terminating the search, regardless of the outcome (solution found or not)
     * 
     * @param start the start state the search start into
     * @param goal the goal we want to reach. If the goal cannot be represented via a state, you can set it to null
     * 
     */
    virtual void tearDownSearch() = 0;
    /**
     * @brief find a solution when the goal cannot be represented whatsoever
     * 
     * @param start the start state
     * @return STATE& the goal found
     */
    virtual std::unique_ptr<ISolutionPath<STATE_IN_SOLUTION, CONST_REF>> search(STATE& start, bool performSetup=true, bool performTearDown=true) {
        return this->_search(start, nullptr, performSetup, performTearDown);
    }
    /**
     * @brief find a solution
     * 
     * @param start the start state
     * @param goal the goal state we want to achieve.
     * @return STATE& the goal found
     */
    virtual std::unique_ptr<ISolutionPath<STATE_IN_SOLUTION, CONST_REF>> search(STATE& start, const STATE& goal, bool performSetup=true, bool performTearDown=true) {
        return this->_search(start, &goal,  performSetup, performTearDown);
    }
    virtual cost_t getSolutionCost(STATE& start, bool performSetup=true, bool performTearDown=true) {
        return this->_getSolutionCost(start, nullptr,  performSetup, performTearDown);
    }
    virtual cost_t getSolutionCost(STATE& start, const STATE& goal, bool performSetup=true, bool performTearDown=true) {
        return this->_getSolutionCost(start, &goal, performSetup, performTearDown);
    }
protected:
    /**
     * @brief search for a solution
     * 
     * @param start the start state the search start into
     * @param goal the goal we want to reach. If the goal cannot be represented via a state, you can set it to null
     * @throws SolutionNotFoundException if a solution cannot be reached
     * @return SolutionPath<STATE> 
     */
    std::unique_ptr<ISolutionPath<STATE_IN_SOLUTION, CONST_REF>> _search(STATE& start, const STATE* goal, bool performSetup, bool performTearDown) {
        try {
            if (performSetup) {
                this->setupSearch(&start, goal);
            }
            CONST_REF goalFetched = this->performSearch(start, goal);
            if (performTearDown) {
                this->tearDownSearch();
            }
            //goal found!
            return this->buildSolutionFromGoalFetched(start, goalFetched, goal);
        } catch (const SolutionNotFoundException& e) {
            if (performTearDown) {
                this->tearDownSearch();
            }
            throw e;
        }

    }
    /**
     * @brief get the cost of the solution found. Does not generate the solution itself
     * 
     * @param start the state where the search starts
     * @param goal the state where ther search ends
     * @return cost_t the cost of the solution found by the algorithm
     * @throws SolutionNotFoundException if a solution cannot be reached
     */
    cost_t _getSolutionCost(STATE& start, const STATE* goal, bool performSetup=true, bool performTearDown=true) {
        cost_t result = 0;
        try {
            if (performSetup) {
                this->setupSearch(&start, goal);
            }
            CONST_REF goalFetched = this->performSearch(start, goal);
            if (performTearDown) {
                this->tearDownSearch();
            }
            //goal found!
            return this->getSolutionCostFromGoalFetched(start, goalFetched, goal);
        } catch (const SolutionNotFoundException& e) {
            if (performTearDown) {
                this->tearDownSearch();
            }
            throw e;
        }
    }
};

} // namespace cpp_utils::search


#endif 