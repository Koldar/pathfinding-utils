#ifndef _GRIDMAPGRAPHCONVERTER_HEADER__
#define _GRIDMAPGRAPHCONVERTER_HEADER__

#include <boost/range/adaptor/map.hpp>
#include "xyLoc.hpp"
#include "IMapGraphConverter.hpp"
#include <cpp-utils/listGraph.hpp>
#include <cpp-utils/exceptions.hpp>

namespace pathfinding::maps {

/**
 * @brief compute the graph underlying a grid map (4 connected)
 * 
 * The costs of each edge is obtained by averagin the endpoints of the involved edge
 * 
 */
class GridMapGraphConverter: public IMapGraphConverter<GridMap, cpp_utils::IImmutableGraph<std::string, xyLoc, cost_t>> {
private:
    GridBranching branching;
public:
    GridMapGraphConverter(GridBranching branching): branching{branching} {

    }

    cost_t computeCost(Direction dir, cost_t sourceCost, cost_t sinkCost) const {
        return (DirectionMethods::isDiagonal(dir) ? M_SQRT2 : 1) * ((sourceCost + sinkCost)/2);
    }
    virtual void cleanup() {

    }
    virtual cpp_utils::IImmutableGraph<std::string,xyLoc,cost_t>&& toGraph(const GridMap& map) const {
        cpp_utils::graphs::ListGraph<std::string, xyLoc, cost_t> result{map.getName()};

        std::unordered_map<xyLoc, cpp_utils::nodeid_t> xyToId{};
        //add vertices
        for (ucood_t y=0; y<map.getHeight(); ++y) {
            for (ucood_t x=0; x<map.getWidth(); ++x) {
                if (map.isTraversable(xyLoc{x, y})) {
                    xyToId[xyLoc{x,y}] = result.addVertex(xyLoc{x, y});
                }
            }
        }

        //add edges
        xyLoc limits{map.getWidth(), map.getHeight()};
        cpp_utils::vectorplus<Direction> availableDirections{};
        switch (this->branching) {
            case GridBranching::FOUR_CONNECTED: {
                availableDirections.add(
                    Direction::NORTH, Direction::SOUTH, Direction::EAST, Direction::WEST
                );
                break;
            }
            case GridBranching::EIGHT_CONNECTED: {
                availableDirections.add(
                    Direction::NORTH, Direction::SOUTH, Direction::EAST, Direction::WEST,
                    Direction::NORTHEAST, Direction::NORTHWEST, Direction::SOUTHEAST, Direction::SOUTHWEST
                );
                break;
            }
            default: {
                throw cpp_utils::exceptions::InvalidScenarioException<GridBranching>{this->branching};
            }
        }

        for (auto&& nodeId : xyToId | boost::adaptors::map_values) {
            xyLoc current = result.getVertex(nodeId);
            if (!map.isTraversable(current)) {
                //untraversable cell. Ignore
                continue;
            }

            for (auto dir : availableDirections) {
                if (current.isThereLocationInDirectionOf(dir, limits)) {
                    xyLoc adjacent{current.getAdjacent(dir)};
                    if (!map.isTraversable(adjacent)) {
                        continue;
                    }
                    result.addEdge(
                        nodeId, 
                        xyToId[adjacent], 
                        this->computeCost(dir, map.getCellCost(current), map.getCellCost(adjacent))
                    );
                }
            }
        }
        return std::move(result);
    }

};

}

#endif 