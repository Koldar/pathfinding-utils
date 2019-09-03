#ifndef _MOVINGAIMAPFACTORY_HEADER__
#define _MOVINGAIMAPFACTORY_HEADER__

#include "AbstractMapFactory.hpp"
#include "GridMap.hpp"
#include <unordered_map>

namespace pathfinding {

/**
 * @brief A factory which loads gridmaps
 * 
 * The supported format can be viewed at https://movingai.com/benchmarks/formats.html
 * 
 */

class GridMapFactory: public AbstractMapFactory<GridMap> {
private:
    GridBranching branching;
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
private:
    template <typename... OTHER>
    GridMapFactory& addTerrain(char symbol, cost_t cost, OTHER... others) {
        this->terrainCost[symbol] = cost;
        return this->addTerrain(others...);
    }
public:
    template <typename OTHER...>
    GridMapFactory(GridBranching branching, OTHER... args) : branching{branching}, terrainCost{} {
        this->addTerrain(args...);
    }
    GridMapFactory& addTerrain(char symbol, cost_t cost) {
        this->terrainCost[symbol] = cost;
        return *this;
    }

    virtual GridMap load(const boost::filesystem::path& mapPath) {
        //load the map
        //TODO continue from here
        //TODO make adjacent graph work!
    }
};

}

#endif