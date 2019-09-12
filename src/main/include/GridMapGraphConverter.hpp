#ifndef _GRIDMAPGRAPHCONVERTER_HEADER__
#define _GRIDMAPGRAPHCONVERTER_HEADER__

#include <boost/smart_ptr/make_unique.hpp>
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
class GridMapGraphConverter: public IMapGraphConverter<GridMap, cpp_utils::graphs::IImmutableGraph<std::string, xyLoc, cost_t>> {
private:
    GridBranching branching;
public:
    GridMapGraphConverter(GridBranching branching): branching{branching} {

    }

    cost_t computeCost(Direction dir, cost_t sourceCost, cost_t sinkCost) const {
        debug("dir is", DirectionMethods::getLabel(dir), "diagonal: ", DirectionMethods::isDiagonal(dir), "multiplier", (DirectionMethods::isDiagonal(dir) ? M_SQRT2 : 1.0));
        return (DirectionMethods::isDiagonal(dir) ? M_SQRT2 : 1.0) * static_cast<float>((sourceCost + sinkCost)/2);
    }
    virtual void cleanup() {

    }
    virtual std::unique_ptr<cpp_utils::graphs::IImmutableGraph<std::string,xyLoc,cost_t>> toGraph(const GridMap& map) const {
        debug("name is", map.getName(), &map.getName());
        cpp_utils::graphs::ListGraph<std::string, xyLoc, cost_t> result{map.getName()};
        debug("result name is", result.getPayload(), &result.getPayload());

        std::unordered_map<xyLoc, cpp_utils::graphs::nodeid_t> xyToId{};
        //add vertices
        for (ucood_t y=0; y<map.getHeight(); ++y) {
            for (ucood_t x=0; x<map.getWidth(); ++x) {
                if (map.isTraversable(xyLoc{x, y})) {
                    xyToId[xyLoc{x,y}] = result.addVertex(xyLoc{x, y});
                }
            }
        }

        //add edges
        xyLoc limits{map.getWidth() - 1, map.getHeight() - 1};
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
                debug("xyLoc is", current, "dir is", DirectionMethods::getLabel(dir),"and limits is", limits);
                if (current.isThereLocationInDirectionOf(dir, limits)) {
                    debug("the location should exist!");
                    xyLoc adjacent{current.getAdjacent(dir)};
                    if (!map.isTraversable(adjacent)) {
                        continue;
                    }
                    
                    if (DirectionMethods::isDiagonal(dir)) {
                        // if the direction is diagonal we need to check if the nearby cells are traversable as well
                        std::pair<xyLoc, xyLoc> adjacentsCells{xyLoc::getNearbyDiagonalCells(current, adjacent)};
                        if (!map.isTraversable(adjacentsCells.first)) {
                            continue;
                        }
                        if (!map.isTraversable(adjacentsCells.second)) {
                            continue;
                        }
                    }

                    finest("adding ", nodeId, "->", xyToId[adjacent], "cost: ", this->computeCost(dir, map.getCellCost(current), map.getCellCost(adjacent)));
                    result.addEdge(
                        nodeId, 
                        xyToId[adjacent], 
                        this->computeCost(dir, map.getCellCost(current), map.getCellCost(adjacent))
                    );
                }
            }
        }
        //return std::unique_ptr<>(new T(std::forward<Args>(args)...));
        //cpp_utils::graphs::ListGraph<std::string, xyLoc, cost_t>
        return std::unique_ptr<cpp_utils::graphs::IImmutableGraph<std::string, xyLoc, cost_t>>{new cpp_utils::graphs::ListGraph<std::string, xyLoc, cost_t>{result}};
        //return boost::make_unique<cpp_utils::graphs::ListGraph<std::string, xyLoc, cost_t>>{result};
    }

};

}

#endif 