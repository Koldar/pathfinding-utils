#ifndef _GRIDMAPGRAPHCONVERTER_HEADER__
#define _GRIDMAPGRAPHCONVERTER_HEADER__

#include <boost/range/adaptor/map.hpp>

namespace pathfinding {

/**
 * @brief compute the graph underlying a grid map (4 connected)
 * 
 * The costs of each edge is obtained by averagin the endpoints of the involved edge
 * 
 */
class GridMapGraphConverter: public IMapGraphConverter<GridMap, IImmutableGraph<std::string, xyLoc, cost_t>> {
private:
    GridBranching branching;
public:
    GridMapConverter(GridBranching branching): branching{branching} {

    }

    cost_t computeCost(cost_t sourceCost, cost_t sinkCost) const {
        return (sourceCost + sinkCost)/2;
    }
    virtual IImmutableGraph<std::string,xyLoc,cost_t> toGraph(const GridMap& map) const {
        ListGraph<std::string, xyLoc, cost_t> result{this->name};

        std::unordered_map<xyLoc, node_id> xyToId{};
        //add vertices
        for (auto y=0; y<map.getHeight(); ++y) {
            for (auto x=0; x<map.getWidth(); ++x) {
                if (map.isTraversable(xyLoc{x, y})) {
                    xyToId[xyLoc{x,y}] = result.addVertex(xyLoc{x, y});
                }
            }
        }

        //add edges
        xyLoc limits{map.getWidth(), map.getHeight()};
        vectorplus<Direction> availableDirections{};
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
            }
            default: {
                throw cpp_utils::exceptions::InvalidScenarioException<Direction>{this->branching};
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
                        this->computeCost(map.getCellCost(current), map.getCellCost(adjacent))
                    );
                }
            }
        }

        return result;

    }

};

}

#endif 