#ifndef _ISTATE_HEADER__
#define _ISTATE_HEADER__

#include <cpp-utils/imemory.hpp>
#include <iostream>

namespace pathfinding::search {

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

}

#endif