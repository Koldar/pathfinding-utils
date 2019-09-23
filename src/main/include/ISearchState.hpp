#ifndef _ISEARCHSTATE_HEADER__
#define _ISEARCHSTATE_HEADER__

#include <cpp-utils/imemory.hpp>
#include <iostream>
#include "types.hpp"

namespace pathfinding::search {

    using namespace cpp_utils;
    using namespace pathfinding;

    /**
     * @brief an abstract search state. 
     * 
     * The only requirements is that the state has are:
     *  @li a uniquely identifying id
     *  @li a parent associated
     *  @li a cost of the state associated
     * 
     */
    template <typename PARENTSTATE>
    class ISearchState : public IMemorable {
    public:
        friend std::ostream& operator <<(std::ostream& ss, const ISearchState<PARENTSTATE>& s) {
            ss << "{g=" << s.getG() << " h=" << s.getH() << " f=" << s.getF();
            return ss;
        }
    public:
        virtual void setId(stateid_t id) = 0;
        virtual stateid_t getId() const = 0;
        virtual PARENTSTATE* getParent() = 0;
        virtual const PARENTSTATE* getParent() const = 0;
        virtual void setParent(PARENTSTATE* parent) = 0;
        /**
         * @brief cost of reaching this state
         * 
         * @return cost_t 
         */
        virtual cost_t getCost() const = 0;
    public:
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


    /**
     * @brief A search state
     * 
     * A more specific Search State which assume you have f,g,h and a parent in each state and that such states will be put inside
     * a priority open list.
     * Implicitly a search state has f,g and h associated, its parent (only one), an id uniquely identifying the state for fast
     * comparisons and a flag indicating if the state has been fully explored (hance it maybe put in close list) or not
     * 
     */
    template <typename PARENTSTATE>
    class IAstarState: public ISearchState<PARENTSTATE> {
    public:
        friend std::ostream& operator << (std::ostream& ss, const IAstarState<PARENTSTATE>& s) {
            ss << "{id=" << s.getId() << " g=" << s.getG() << " h=" << s.getH() << " f=" << s.getF();
            return ss;
        }
    public:
        /**
         * @brief set the F value for this state, regardless of g and h value
         * 
         * @param f the F value to set
         */
        virtual void setF(cost_t f) = 0;
        virtual void setG(cost_t g) = 0;
        virtual void setH(cost_t h) = 0;
        virtual void setExpanded(bool expanded) = 0;
        virtual cost_t getF() const = 0;
        virtual cost_t getG() const = 0;
        virtual cost_t getH() const = 0;
        virtual bool isExpanded() const = 0;
    public:
        virtual cost_t getCost() const {
            return this->getG();
        }
    public:
        void markAsExpanded() {
            this->setExpanded(true);
        }
        void markAsUnexpanded() {
            this->setExpanded(false);
        }
    };

}

#endif
