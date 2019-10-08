/**
 * @file
 * @author Massimo Bono
 * @brief a state which is based upon a graph but doesn't directly reference it. Use it if you don't care to access to the underlying graph
 * @version 0.1
 * @date 2019-10-7
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef _PATH_FINDING_UTILS_ABSTRACT_GRAPH_STATE_HEADER__
#define _PATH_FINDING_UTILS_ABSTRACT_GRAPH_STATE_HEADER__

#include "ISearchState.hpp"
#include "IStateSupplier.hpp"
#include "IStateExpander.hpp"
#include "IGoalChecker.hpp"
#include <cpp-utils/StaticPriorityQueue.hpp>
#include <cpp-utils/igraph.hpp>
#include <cpp-utils/mapplus.hpp>
#include <cpp-utils/vectorplus.hpp>
#include <cpp-utils/adjacentGraph.hpp>
#include <tuple>

namespace pathfinding::search {

    using namespace pathfinding;
    using namespace cpp_utils::graphs;

    /**
     * @brief A* state where a state is identified by a location nodeid_t 
     * 
     * The state represents a position in the graph. However, the graph is **not** referenced explicitly in the state.
     * This is a lightweight representation of the state.
     * 
     * This state contains the value of the vertex payload it intend to represents
     * 
     */
    template <typename PARENT_STATE, typename... IMPORTANT_STUFF>
    class AbstractGraphState: public IAstarState<PARENT_STATE>, cpp_utils::HasPriority {
    protected:
        /**
         * @brief f value in A*
         * 
         */
        cost_t f;
        /**
         * @brief g value in A*
         * 
         */
        cost_t g;
        /**
         * @brief h value in A*
         * 
         */
        cost_t h;
        /**
         * @brief parent of state. If null the state do not have any parent
         * 
         * @note
         * this pointer should point to a state which has the same type
         */
        void* parent;
        /**
         * @brief id uniquely identifying tyhe state
         * 
         */
        stateid_t id;
        /**
         * @brief if true it means this state should belong to the close list of A*
         * 
         */
        bool expanded;
        /**
         * @brief field necessary if you want to put this instance in a queue
         * 
         */
        priority_t priority;
        /**
         * @brief a structure containing where we are in the graph representing the map
         * 
         */
        nodeid_t position;
    public:
        AbstractGraphState(cost_t f, cost_t g, cost_t h, const PARENT_STATE* parent, stateid_t id, bool expanded, nodeid_t position): f{f}, g{g}, h{h}, parent{static_cast<void*>(parent)}, id{id}, expanded{expanded}, position{position}, priority{0} {

        }

        AbstractGraphState(stateid_t id, nodeid_t position): f{0}, g{0}, h{0}, parent{nullptr}, id{id}, expanded{false}, position{position}, priority{0} {

        }

        AbstractGraphState(const AbstractGraphState<PARENT_STATE, IMPORTANT_STUFF...>& other): f{other.f}, g{other.g}, h{other.h}, parent{other.parent}, id{other.id}, expanded{other.expanded}, position{other.position}, priority{other.priority} {
        }
        AbstractGraphState<PARENT_STATE, IMPORTANT_STUFF...>& operator =(const AbstractGraphState<PARENT_STATE, IMPORTANT_STUFF...>& other) {
            this->f = other.f;
            this->g = other.g;
            this->h = other.h;
            this->parent = other.parent;
            this->id = other.id;
            this->expanded = other.expanded;
            this->position = other.position;
            this->priority = other.priority;
            return *this;
        }
        AbstractGraphState<PARENT_STATE, IMPORTANT_STUFF...>(AbstractGraphState<PARENT_STATE, IMPORTANT_STUFF...>&& other): f{other.f}, g{other.g}, h{other.h}, parent{other.parent}, id{other.id}, expanded{other.expanded}, position{other.position}, priority{other.priority} {
        }
        AbstractGraphState<PARENT_STATE, IMPORTANT_STUFF...>& operator =(AbstractGraphState<PARENT_STATE, IMPORTANT_STUFF...>&& other) {
            this->f = other.f;
            this->g = other.g;
            this->h = other.h;
            this->parent = other.parent;
            this->id = other.id;
            this->expanded = other.expanded;
            this->position = other.position;
            this->priority = other.priority;
            return *this;
        }
        
        nodeid_t getPosition() const {
            return this->position;
        }

        virtual std::tuple<const IMPORTANT_STUFF&...> getPayload() const = 0;
    public:
        friend bool operator <(const AbstractGraphState<PARENT_STATE, IMPORTANT_STUFF...>& a, const AbstractGraphState<PARENT_STATE, IMPORTANT_STUFF...>& b) {
            if(a.f < b.f) {
                return true;
            } else if (b.f < a.f) {
                return false;
            }
            //ok, it appeans that a and b have the same f. We need some tie breaking mechanism
            // break ties in favour of larger g
            if(a.g > b.g) {
                return true;
            }
            return false;
        }
        friend bool operator ==(const AbstractGraphState<PARENT_STATE, IMPORTANT_STUFF...>& a, const AbstractGraphState<PARENT_STATE, IMPORTANT_STUFF...>& b) {
            if (&a == &b) {
                return true;
            }
            if (a.getId() != b.getId()) {
                return false;
            }
            return a.getPosition() == b.getPosition();
        }
        friend std::ostream& operator <<(std::ostream& out, const AbstractGraphState<PARENT_STATE, IMPORTANT_STUFF...>& state) {
            out 
                << "{id: " 
                << state.getId() 
                << " node: " 
                << state.getPayload()
                << "}";
            return out;
        }
    public:
        priority_t getPriority() const {
            return this->priority;
        }
        void setPriority(priority_t p) {
            this->priority = p;
        }
    public:
        virtual void setF(cost_t f) {
            this->f = f;
        }
        virtual void setG(cost_t g) {
            this->g = g;
        }
        virtual void setH(cost_t h) {
            this->h = h;
        }
        virtual void setParent(PARENT_STATE* parent) {
            this->parent = parent;
        }
        virtual void setId(stateid_t id) {
            this->id = id;
        }
        virtual void setExpanded(bool expanded) {
            this->expanded = expanded;
        }
        virtual cost_t getF() const {
            return this->f;
        }
        virtual cost_t getG() const {
            return this->g;
        }
        virtual cost_t getH() const {
            return this->h;
        }
        virtual PARENT_STATE* getParent() {
            return static_cast<PARENT_STATE*>(this->parent);
        }
        virtual const PARENT_STATE* getParent() const {
            return static_cast<const PARENT_STATE*>(this->parent);
        }
        virtual stateid_t getId() const {
        return this->id;
        }
        virtual bool isExpanded() const {
            return this->expanded;
        }
    public:
        MemoryConsumption getByteMemoryOccupied() const {
            return sizeof(*this);
        }
    };

}


#endif