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

/**
 * @brief A search state
 * 
 * Implicitly a search state has f,g and h associated, its parent (only one), an id uniquely identifying the state for fast
 * comparisons and a flag indicating if the state has been fully explored (hance it maybe put in close list) or not
 * 
 */
template <typename PARENTSTATE>
class IState : public IMemorable {
public:
    friend std::ostream& operator <<(std::ostream& ss, const IState& s) {
        ss << "{g=" << s.getG() << " h=" << s.getH() << " f=" << s.getF();
        return ss;
    }
public:
    virtual void setF(cost_t f) = 0;
    virtual void setG(cost_t g) = 0;
    virtual void setH(cost_t h) = 0;
    virtual void setParent(PARENTSTATE* parent) = 0;
    virtual void setId(stateid_t id) = 0;
    virtual void setExpanded(bool expanded) = 0;
    virtual cost_t getF() const = 0;
    virtual cost_t getG() const = 0;
    virtual cost_t getH() const = 0;
    virtual PARENTSTATE* getParent() = 0;
    virtual const PARENTSTATE* getParent() const = 0;
    virtual stateid_t getId() const = 0;
    virtual bool isExpanded() const = 0;
public:
    void markAsExpanded() {
        this->setExpanded(true);
    }
    void markAsUnexpanded() {
        this->setExpanded(false);
    }
    /**
     * @brief check if the state is the initial state
     * 
     * @return true if it has no parent
     * @return false otherwise
     */
    bool isInitial() const {
        return this->getParent() == nullptr;
    }
};

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