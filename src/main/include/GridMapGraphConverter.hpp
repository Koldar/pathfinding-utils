#ifndef _GRIDMAPGRAPHCONVERTER_HEADER__
#define _GRIDMAPGRAPHCONVERTER_HEADER__

#include <boost/smart_ptr/make_unique.hpp>
#include <boost/range/adaptor/map.hpp>
#include "xyLoc.hpp"
#include "IMapGraphConverter.hpp"
#include "GridBranching.hpp"
#include "GridMap.hpp"
#include <cpp-utils/listGraph.hpp>
#include <cpp-utils/exceptions.hpp>


namespace pathfinding::maps {

/**
 * @brief compute the graph underlying a grid map (4 connected)
 * 
 * The costs of each edge is obtained by averagin the endpoints of the involved edge
 * 
 */
class GridMapGraphConverter: public IMapGraphConverter<GridMap, cpp_utils::graphs::IImmutableGraph<std::string, xyLoc, cost_t>> {
private:
    GridBranching branching;
public:
    GridMapGraphConverter(GridBranching branching);
    virtual ~GridMapGraphConverter();
    GridMapGraphConverter(const GridMapGraphConverter& o);
    GridMapGraphConverter(GridMapGraphConverter&& o);
    GridMapGraphConverter& operator = (const GridMapGraphConverter& o);
    GridMapGraphConverter& operator = (GridMapGraphConverter&& o);
public:
    cost_t computeCost(Direction dir, cost_t sourceCost, cost_t sinkCost) const;
    virtual std::unique_ptr<cpp_utils::graphs::IImmutableGraph<std::string,xyLoc,cost_t>> toGraph(const GridMap& map) const;
public:
    virtual void cleanup();
};

}

#endif 