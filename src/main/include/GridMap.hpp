#ifndef _GRIDMAP_HEADER__
#define _GRIDMAP_HEADER__

#include "types.hpp"
#include "IPathFindingMap.hpp"
#include "xyLoc.hpp"
#include <unordered_map>
#include <string>
#include <vector>
#include <cpp-utils/listGraph.hpp>

namespace pathfinding {

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
template <typename BRANCHING>
class GridMap: public IPathFindingMap<std::string, xyLoc, cost_t> {
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
    const cood_t width;
    const cood_t height;
    cpp_utils::vectorplus<char> cells;
    size_t size;
private:
    size_t computeSize() const {
        this->cells
            .map<cost_t>([&](char x) {return terrainCost[x];})
            .filter([&](cost_t x) { return x.isNotInfinity()})
            .size();
    }
    int toVectorCoord(xyLoc xy) {
        return toVectorCoord(xy.y, xy.x);
    }
    int toVectorCoord(cood_t y, cood_t x) {
        return y * this->width + x;
    }
    xyLoc toXyLoc(int x) {
        return xyLoc{x / this->width, x % this->width};
    }
public:
    GridMap(const std::string& name, const std::vector<char>& cells, cood_t width, cood_t height, std::unordered_map<char, cost_t> terrainCost): name{name}, cells{cells}, width{width}, height{height}, terrainCost{terrainCost} {
        this->size = this->computeSize();
    }

    ~GridMap() {
    }

    GridMap(const GridMap& other): name{other.name}, cells{other.cells}, width{other.width}, height{other.height}, terrainCost{other.terrainCost} {
    }

    GridMap(const GridMap&& other): name{other.name}, cells{other.cells}, width{other.width}, height{other.height}, terrainCost{other.terrainCost}, size{other.size} {
    }

    GridMap& operator= (const GridMap& map) {
        this->name = map.name;
        this->width = map.width;
        this->height = map.height;
        this->terrainCost = map.terrainCost;
        this->cells = map.cells;
        this->size = map.size;
        return *this;
    }

    GridMap operator= (const GridMap&& map) {
        this->name = map.name;
        this->width = map.width;
        this->height = map.height;
        this->terrainCost = map.terrainCost;
        this->cells = map.cells;
        this->size = map.size;
        return *this;
    }

    cood_t getWidth() const {
        return this->width;
    }

    cood_t getHeight() const {
        return this->height;
    }

    char getCellTerrain(xyLoc loc) const {
        return this->cells[this->toVectorCoord(loc)];
    }

    cost_t getCellCost(xyLoc loc) const {
        return this->terrainCost[this->cells[this->toVectorCoord(loc)]];
    }

    bool isTraversable(xyLoc loc) const {
        return getCellCost(loc).isNotInfinity();
    }

    virtual const std::string& getName() const {
        return this->name;
    }
    virtual size_t size() const {
        return this->size;
    }
    virtual IImmutableGraph<std::string,xyLoc,cost_t> toGraph() const {
        ListGraph<std::string, xyLoc, cost_t> result{this->name};

        //add vertices
        for (auto y=0; y<this->height; ++y) {
            for (auto x=0; x<this->width; ++x) {
                if (this)
                result.addVertex(xyLoc{x, y});
            }
        }

        //add edges
        for (auto y=0; y<this->height; ++y) {
            for (auto x=0; x<this->width; ++x) {
                if (!this->isTraversable(xyLoc{x, y})) {
                    //untraversable cell. Ignore
                    continue;
                }

                switch (BRANCHING) {
                    case GridBranching::FOUR_CONNECTED: {
                        
                        break;
                    }
                    case GridBranching::EIGHT_CONNECTED: {
                        break;
                    }
                    default: {
                        throw cpp_utils::exceptions::InvalidScenarioException{BRANCHING};
                    }
                }
            }
        }

    }
};

}

#endif