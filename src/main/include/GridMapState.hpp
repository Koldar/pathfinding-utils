#ifndef _GRIDMAPSTATE_HEADER__
#define _GRIDMAPSTATE_HEADER__

#include "ISearchAlgorithm.hpp"
#include "IState.hpp"
#include "GridBranching.hpp"
#include "IStateSupplier.hpp"
#include "IStateExpander.hpp"
#include "IGoalChecker.hpp"
#include "xyLoc.hpp"
#include <cpp-utils/StaticPriorityQueue.hpp>
#include <cpp-utils/igraph.hpp>
#include <cpp-utils/vectorplus.hpp>
#include <utility>

namespace pathfinding::search {

/**
 * @brief a search state over a map we know it's a grid map
 * 
 * In this state there is only x and y coordinates
 * 
 */
class GridMapState : public IState<GridMapState>, cpp_utils::HasPriority {
private:
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
     */
    GridMapState* parent;
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
     * @brief a structure containing where we are in the grid map
     * 
     */
    const xyLoc position;
    /**
     * @brief field necessary if you want to put this instance in a queue
     * 
     */
    priority_t priority;
public:
    GridMapState(cost_t f, cost_t g, cost_t h, GridMapState* parent, stateid_t id, bool expanded, xyLoc position): f{f}, g{g}, h{h}, parent{parent}, id{id}, expanded{expanded}, position{position}, priority{0} {

    }
    GridMapState(stateid_t id, xyLoc position): f{0}, g{0}, h{0}, parent{nullptr}, id{id}, expanded{false}, position{position}, priority{0} {

    }
    xyLoc getPosition() const {
        return position;
    }
public:
    priority_t getPriority() const {
        return this->priority;
    }
    void setPriority(priority_t p) {
        this->priority = p;
    }
public:
    /**
     * @brief create a dumb state containing only the location.
     * 
     * @note
     * Do **not** use this object for anything aside testing! This object is not good to be used in search!
     * 
     * @param loc the position the agent currently is
     * @return GridMapState 
     */
    static GridMapState make(const xyLoc& loc) {
        return GridMapState{0L, 0L, 0L, nullptr, 0L, false, loc};
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
    virtual void setParent(GridMapState* parent) {
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
    virtual GridMapState* getParent() {
        return this->parent;
    }
    virtual const GridMapState* getParent() const {
        return this->parent;
    }
    virtual stateid_t getId() const {
    return this->id;
    }
    virtual bool isExpanded() const {
        return this->expanded;
    }
public:
    MemoryConsumption getByteMemoryOccupied() const {
        return MemoryConsumption{sizeof(*this), MemoryConsumptionEnum::BYTE};
    }
};


class GridMapStateSupplier: public AbstractStateSupplier<GridMapState, xyLoc> {
protected:
    virtual GridMapState&& fetchNewInstance(stateid_t id, xyLoc position) {
        return std::move(GridMapState{id, position});
    }    
};

/**
 * @brief an expander that uses an underlying graph to compute successors
 * 
 * The successors of a state are the successors of the vertex representing the state
 * 
 * @note
 * Note that the state id of a search state is **the same** of the vertex id of a node. This
 * means that the state space is as large as the underlying graph!
 * 
 * @tparam G paylaod of the whole graph
 * @tparam V paylaod of each single vertex
 */
template <typename G>
class GridMapStateExpander: public IStateExpander<GridMapState, xyLoc> {
private:
    const IImmutableGraph<G, xyLoc, cost_t>& graph;
public:
    GridMapStateExpander(const IImmutableGraph<G, xyLoc, cost_t>& graph): graph{graph} {

    }
    virtual ~GridMapStateExpander() {

    }
    GridMapStateExpander(const GridMapStateExpander<G>& other) : graph{other.graph} {

    }
    GridMapStateExpander<G>& operator =(const GridMapStateExpander<G>& other) = delete;
public:
    virtual cpp_utils::vectorplus<std::pair<GridMapState&, cost_t>> getSuccessors(const GridMapState& state, IStateSupplier<GridMapState, xyLoc>& supplier) {
        cpp_utils::vectorplus<std::pair<GridMapState&, cost_t>> result{};
        for (auto outEdge : graph.getOutEdges(state.getId())) {
            result.add(std::pair<GridMapState&, cost_t>{
                supplier.getState(outEdge.getSinkId(), this->graph.getVertex(outEdge.getSinkId())),
                outEdge.getPayload()
            });
        }
        return result;
    }
    
    virtual void cleanup() {

    }
};

class SAPFGridMapGoalChecker: public IGoalChecker<GridMapState> {
public:
    virtual bool isGoal(const GridMapState& state, const GridMapState* goal) const {
        return state.getPosition() == goal->getPosition();
    }
};



}

#endif 