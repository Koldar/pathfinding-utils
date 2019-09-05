#ifndef _ISEARCHALGORITHM_HEADER__
#define _ISEARCHALGORITHM_HEADER__

#include <limits>
#include <cpp-utils/imemory.hpp>
#include <cpp-utils/wrapped_number.hpp>
#include <vector>
#include <cpp-utils/exceptions.hpp>
#include <cpp-utils/macros.hpp>
#include "types.hpp"


namespace pathfinding::search
{

using namespace cpp_utils;

template <typename STATE>
class SolutionPath : public std::vector<STATE> {
};

/**
 * @brief thrown when the solution couldn't be found in the 
 * 
 */
class SolutionNotFoundException : public exceptions::GenericException {

};

/**
 * @brief an algorithm that given a start state goes till a goal one
 * 
 */
template <typename STATE>
class ISearchAlgorithm {
protected:
    /**
     * @brief perform the actual search
     * 
     * @param start the initial state
     * @param goal the goal we want to reach. nullptr if we cannot represent in a concrete way the goal
     * @return const STATE& the goal we have retrieved.
     * @throw SolutionNotFoundException if we were unable to fetch the goal
     */
    virtual const STATE& _search(STATE& start, const STATE* goal) = 0;
    /**
     * @brief steps to execute before starting the search
     * 
     */
    virtual void setupSearch() = 0;
    /**
     * @brief steps to execute after terminating the search, regardless of the outcome (solution found or not)
     * 
     */
    virtual void tearDownSearch() = 0;
public:
    /**
     * @brief find a solution when the goal cannot be represented whatsoever
     * 
     * @param start the start state
     * @return STATE& the goal found
     */
    STATE& _search(const STATE& start) {
        return this->_search(start.get_id(), nullptr);
    }
    /**
     * @brief find a solution
     * 
     * @param start the start state
     * @param goal the goal state we want to achieve.
     * @return STATE& the goal found
     */
    STATE& _search(const STATE& start, const STATE& goal) {
        return this->_search(start, &goal);
    }
    /**
     * @brief search for a solution
     * 
     * @param start the start state the search start into
     * @param goal the goal we want to reach. If the goal cannot be represented via a state, you can set it to null
     * @throws SolutionNotFoundException if a solution cannot be reached
     * @return SolutionPath<STATE> 
     */
    SolutionPath<const STATE&> search(STATE& start, const STATE* goal) {
        SolutionPath<const STATE&> result{};
        try {
            this->setupSearch();
            const STATE& goalFetched = this->_search(start, goal);
            this->tearDownSearch();
            //goal found!
            const STATE* tmp = &goalFetched;
            while (tmp != nullptr) {
                result.push_back(*tmp);
                tmp = tmp->getParent();
            }
            return result;
        } catch (const SolutionNotFoundException& e) {
            this->tearDownSearch();
            return result;
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
    cost_t getSolutionCost(stateid_t start, const stateid_t* goal) {
        cost_t result = 0;
        try {
            this->setupSearch();
            const STATE& goal = this->_search(start, goal);
            this->tearDownSearch();
            //goal found!
            return goal.getG();
        } catch (const SolutionNotFoundException& e) {
            this->tearDownSearch();
            throw e;
        }
    }
};

} // namespace cpp_utils::search


#endif 