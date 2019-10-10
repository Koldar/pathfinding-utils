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
    template <typename V>
    class GraphlessState: public AbstractGraphState<V> {
        typedef GraphlessState<V> GraphlessStateInstance;
    protected:
        /**
         * @brief reference of paylaod of the node it intends to mimic
         * 
         */
        V payload;
    public:
        GraphlessState(stateid_t id, nodeid_t position, const V& payload): AbstractGraphState<V>{id, position}, payload{payload} {

        }
        GraphlessState(const GraphlessStateInstance& other): AbstractGraphState<V>{other}, payload{other.payload} {
        }
        GraphlessStateInstance& operator =(const GraphlessStateInstance& other) {
            AbstractGraphState<V>::operator=(other);
            this->payload = other.payload;
            return *this;
        }
        GraphlessState(GraphlessStateInstance&& other): AbstractGraphState<V>{::std::move(other)}, payload{::std::move(other.payload)} {
        }
        GraphlessStateInstance& operator =(GraphlessStateInstance&& other) {
            AbstractGraphState<V>::operator=(other);
            this->payload = ::std::move(other.payload);
            return *this;
        }

        virtual void setParent(ISearchState* parent) {
            this->parent = static_cast<GraphlessState*>(parent);
        }
        virtual GraphlessState* getParent() {
            return static_cast<GraphlessState*>(this->parent);
        }
        virtual const GraphlessState* getParent() const {
            return static_cast<const GraphlessState*>(this->parent);
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