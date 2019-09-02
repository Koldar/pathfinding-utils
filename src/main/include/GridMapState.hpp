#ifndef _GRIDMAPSTATE_HEADER__
#define _GRIDMAPSTATE_HEADER__

#include "ISearchAlgorithm.hpp"

namespace pathfinding::search {

class GridMapState;

/**
 * @brief a search state over a map we know it's a grid map
 * 
 * In this state there is only x and y coordinates
 * 
 */
class GridMapState : public IState<GridMapState> {
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
public:
    GridMapState(cost_t f, cost_t g, cost_t h, GridMapState* parent, stateid_t id, bool expanded, xyLoc position): f{f}, g{g}, h{h}, parent{parent}, id{id}, expanded{expanded}, position{position} {

    }
    xyLoc getPosition() const {
        return position;
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
    // virtual void setId(stateid_t id) = 0;
    // virtual void setExpanded(bool expanded) = 0;
    // virtual cost_t getF() const = 0;
    // virtual cost_t getG() const = 0;
    // virtual cost_t getH() const = 0;
    // virtual IState* getParent() = 0;
    // virtual const IState* getParent() const = 0;
    // virtual stateid_t getId() const = 0;
    // virtual bool isExpanded() const = 0;
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
    MemoryConsumption getByteMemoryOccupied() const {
        return MemoryConsumption{sizeof(*this), MemoryConsumptionEnum::BYTE};
    }
};

}

#endif 