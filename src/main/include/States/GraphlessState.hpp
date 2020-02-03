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
#ifndef _CPD_SEARCH_GRAPHLESSSTATE_HEADER__
#define _CPD_SEARCH_GRAPHLESSSTATE_HEADER__

#include "ISearchState.hpp"
#include "IStateSupplier.hpp"
#include "IStateExpander.hpp"
#include "IGoalChecker.hpp"
#include "AbstractGraphState.hpp"
#include <cpp-utils/StaticPriorityQueue.hpp>
#include <cpp-utils/igraph.hpp>
#include <cpp-utils/mapplus.hpp>
#include <cpp-utils/vectorplus.hpp>
#include <cpp-utils/adjacentGraph.hpp>
#include <tuple>

namespace pathfinding::search {

    using namespace pathfinding;
    using namespace cpp_utils::graphs;

    //TODO rename
    /**
     * @brief A* state where a state is identified by a location nodeid_t 
     * 
     * The state represents a position in the graph. However, the graph is **not** referenced explicitly in the state.
     * This is a lightweight representation of the state.
     * 
     * This state contains the value of the vertex payload it intend to represents
     * 
     */
    template <typename V, typename REASON>
    class GraphlessState: public AbstractGraphState<REASON, V> {
        using This = GraphlessState<V, REASON>;
        using Super = AbstractGraphState<REASON, V>;
    protected:
        /**
         * @brief reference of paylaod of the node it intends to mimic
         * 
         */
        V payload;
    public:
        GraphlessState(stateid_t id, nodeid_t position, const V& payload, const REASON& reason): Super{id, position, reason}, payload{payload} {

        }
        GraphlessState(stateid_t id, nodeid_t position, const V& payload): Super{id, position}, payload{payload} {

        }
        GraphlessState(const This& other): Super{other}, payload{other.payload} {
        }
        This& operator =(const This& other) {
            Super::operator=(other);
            this->payload = other.payload;
            return *this;
        }
        GraphlessState(This&& other): Super{::std::move(other)}, payload{::std::move(other.payload)} {
        }
        This& operator =(This&& other) {
            Super::operator=(other);
            this->payload = ::std::move(other.payload);
            return *this;
        }

        virtual void setParent(ISearchState<REASON>* parent) {
            this->parent = static_cast<This*>(parent);
        }
        virtual This* getParent() {
            return static_cast<This*>(this->parent);
        }
        virtual const This* getParent() const {
            return static_cast<const This*>(this->parent);
        }



        
        nodeid_t getPosition() const {
            return this->position;
        }

        std::tuple<const V&> getPayload() const {
            return std::make_tuple(this->payload);
        }

        const V& getFirstData() const {
            return this->payload;
        }
    public:
        MemoryConsumption getByteMemoryOccupied() const {
            return sizeof(*this);
        }
    };

}


#endif