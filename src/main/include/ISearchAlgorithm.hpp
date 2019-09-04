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
    virtual const STATE& _search(stateid_t start, const stateid_t* goal) = 0;
public:
    /**
     * @brief find a solution
     * 
     * @param start the start state
     * @param goal the goal state we want to achieve. It can be null if the representation of the goal is useless in the search
     * @return STATE& the goal found
     */
    STATE& _search(const STATE& start, const stateid_t* goal) {
        return this->_search(start.get_id(), goal);
    }
    /**
     * @brief find a solution
     * 
     * @param start the start state
     * @param goal the goal state we want to achieve. It can be null if the representation of the goal is useless in the search
     * @return STATE& the goal found
     */
    STATE& _search(const STATE& start, const STATE& goal) {
        return this->_search(start.get_id(), goal.get_id());
    }
    /**
     * @brief search for a solution
     * 
     * @param start 
     * @param goal 
     * @throws SolutionNotFoundException if a solution cannot be reached
     * @return SolutionPath<STATE> 
     */
    SolutionPath<const STATE&> search(stateid_t start, const stateid_t* goal) {
        SolutionPath<const STATE&> result{};
        try {
            this->setupSearch();
            const STATE& goal = this->_search(start, goal);
            this->tearDownSearch();
            //goal found!
            const STATE* tmp = &goal;
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

    /**
     * @brief steps to execute before starting the search
     * 
     */
    virtual void setupSearch() = 0;
    /**
     * @brief state to execute after starting the searhc, regardless of the outcome (solution found or not)
     * 
     */
    virtual void tearDownSearch() = 0;
};

} // namespace cpp_utils::search


#endif 