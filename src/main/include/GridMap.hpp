#ifndef _GRIDMAP_HEADER__
#define _GRIDMAP_HEADER__

#include "types.hpp"
#include "IPathFindingMap.hpp"
#include "xyLoc.hpp"
#include <cpp-utils/vectorplus.hpp>
#include <unordered_map>
#include <string>
#include <vector>
#include <cpp-utils/listGraph.hpp>

namespace pathfinding::maps {

/**
 * @brief 
 * 
 * As an example:
 * 
 * @code
 * | | | | | | 
 * | | | | |@|
 * | |@|@| |@|
 * | | | | | |
 * @endcode
 * 
 * width is 5 while height is 4. size is 16.
 * 
 * @tparam BRANCHING a GridMapBranching representing how the underlying graph is structured
 */
class GridMap: public IPathFindingMap {
private:
    /**
     * @brief name of the map
     * 
     */
    const std::string name;
    /**
     * @brief map where each key is a symbol in the gridmap filename representing a cell while the value is the cost an agent needs to pay to traverse such cell
     * 
     * @code
     *  terrainCost['.'] = 1000
     *  terrainCost['T'] = 1500
     *  terrainCost['@'] = cost_t::INFTY
     * @endcode
     */
    std::unordered_map<char, cost_t> terrainCost;
    /**
     * @brief number of columns this map has
     * 
     */
    const ucood_t width;
    /**
     * @brief number of rows this map has
     * 
     */
    const ucood_t height;
    /**
     * @brief content of the map.
     * 
     * Costs are associated vcia ::terrainCost
     * 
     */
    cpp_utils::vectorplus<char> cells;
    /**
     * @brief number of cells which has their cost different than +infinity
     * 
     */
    size_t size;
private:
    size_t computeSize() const {
        return this->cells
            .map<cost_t>([&](char x) {return this->terrainCost.at(x);})
            .filter([&](cost_t x) { return x.isNotInfinity(); })
            .size();
    }
    int toVectorCoord(xyLoc xy) const {
        return toVectorCoord(xy.y, xy.x);
    }
    int toVectorCoord(ucood_t y, ucood_t x) const {
        return y * this->width + x;
    }
    xyLoc toXyLoc(ucood_t x) const {
        return xyLoc{x / this->width, x % this->width};
    }
public:
    GridMap(const std::string& name, const std::vector<char>& cells, ucood_t width, ucood_t height, std::unordered_map<char, cost_t> terrainCost): name{name}, cells{cells}, width{width}, height{height}, terrainCost{terrainCost} {
        this->size = this->computeSize();
    }

    ~GridMap() {
    }

    GridMap(const GridMap& other): name{other.name}, cells{other.cells}, width{other.width}, height{other.height}, terrainCost{other.terrainCost} {
    }

    GridMap(GridMap&& other): name{other.name}, cells{other.cells}, width{other.width}, height{other.height}, terrainCost{other.terrainCost}, size{other.size} {
    }

    GridMap& operator= (const GridMap& map) = delete;
    GridMap operator= (GridMap&& map) = delete;

    ucood_t getWidth() const {
        return this->width;
    }

    ucood_t getHeight() const {
        return this->height;
    }

    char getCellTerrain(xyLoc loc) const {
        return this->cells[this->toVectorCoord(loc)];
    }

    cost_t getCellCost(xyLoc loc) const {
        debug("xyLoc is", loc, "cell is", this->cells[this->toVectorCoord(loc)]);
        return this->terrainCost.at(this->cells[this->toVectorCoord(loc)]);
    }

    bool isTraversable(xyLoc loc) const {
        return getCellCost(loc).isNotInfinity();
    }
public:
    virtual const std::string& getName() const {
        return this->name;
    }
    virtual size_t getSize() const {
        return this->size;
    }
};

}

#endif