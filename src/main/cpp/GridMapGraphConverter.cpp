#include "GridMapGraphConverter.hpp"
#include <cpp-utils/adjacentGraph.hpp>

namespace pathfinding::maps {

    GridMapGraphConverter::GridMapGraphConverter(GridBranching branching): branching{branching} {

    }

    GridMapGraphConverter::~GridMapGraphConverter() {

    }

    GridMapGraphConverter::GridMapGraphConverter(const GridMapGraphConverter& o): branching{o.branching} {

    }

    GridMapGraphConverter::GridMapGraphConverter(GridMapGraphConverter&& o): branching{o.branching} {

    }

    GridMapGraphConverter& GridMapGraphConverter::operator = (const GridMapGraphConverter& o) {
        this->branching = o.branching;

        return *this;
    }

    GridMapGraphConverter& GridMapGraphConverter::operator = (GridMapGraphConverter&& o) {
        this->branching = o.branching;

        return *this;
    }

    cost_t GridMapGraphConverter::computeCost(Direction dir, cost_t sourceCost, cost_t sinkCost) const {
        using namespace ::pathfinding;

        debug("dir is", DirectionMethods::getLabel(dir), "diagonal: ", DirectionMethods::isDiagonal(dir), "multiplier", (DirectionMethods::isDiagonal(dir) ? M_SQRT2 : 1.0));
        return (DirectionMethods::isDiagonal(dir) ? M_SQRT2 : 1.0) * static_cast<float>((sourceCost + sinkCost)/2);
    }

    void GridMapGraphConverter::cleanup() {

    }
    
    std::unique_ptr<cpp_utils::graphs::IImmutableGraph<std::string,xyLoc,cost_t>> GridMapGraphConverter::toGraph(const GridMap& map) const {
        debug("name is", map.getName(), &map.getName());
        cpp_utils::graphs::ListGraph<std::string, xyLoc, cost_t> result{map.getName()};
        debug("result name is", result.getPayload(), &result.getPayload());

        std::vector<xyLoc> vertexPayload{};

        std::unordered_map<xyLoc, cpp_utils::graphs::nodeid_t> xyToId{};
        //add vertices
        for (ucood_t y=0; y<map.getHeight(); ++y) {
            for (ucood_t x=0; x<map.getWidth(); ++x) {
                if (map.isTraversable(xyLoc{x, y})) {
                    xyToId[xyLoc{x,y}] = result.addVertex(xyLoc{x, y});
                    vertexPayload.push_back(xyLoc{x, y});
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
                throw cpp_utils::exceptions::InvalidScenarioException{"grid connection property", this->branching};
            }
        }

        cpp_utils::graphs::nodeid_t lastSourceId = 0;
        bool firstSourceId = true;
        std::vector<cpp_utils::graphs::OutEdge<cost_t>> edges{};
        std::vector<int> outEdgesOfvertexBegin{};

        for (cpp_utils::graphs::nodeid_t nodeId = 0; nodeId < result.numberOfVertices(); ++nodeId) {
            xyLoc current = vertexPayload[nodeId];
        
            if (!map.isTraversable(current)) {
                //untraversable cell. Ignore
                continue;
            }

            bool hasAtLeastOneEdge = false;
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

                    if (firstSourceId) {
                        //first edge we add
                        outEdgesOfvertexBegin.push_back(edges.size());
                        edges.push_back(cpp_utils::graphs::OutEdge<cost_t>{xyToId[adjacent], this->computeCost(dir, map.getCellCost(current), map.getCellCost(adjacent))});
                        firstSourceId = false;
                    } else if (lastSourceId < nodeId) {
                        //we have changed the source id of the edge ot add
                        outEdgesOfvertexBegin.push_back(edges.size());
                        edges.push_back(cpp_utils::graphs::OutEdge<cost_t>{xyToId[adjacent], this->computeCost(dir, map.getCellCost(current), map.getCellCost(adjacent))});
                    } else if (nodeId == lastSourceId) {
                        //the source id of the edge to add has not changed
                        edges.push_back(cpp_utils::graphs::OutEdge<cost_t>{xyToId[adjacent], this->computeCost(dir, map.getCellCost(current), map.getCellCost(adjacent))});
                    } else {
                        //AdjGraph invariant is not respected
                        throw cpp_utils::exceptions::ImpossibleException{};
                    }


                    lastSourceId = nodeId;
                    hasAtLeastOneEdge = true;
                }
            }

            if (!hasAtLeastOneEdge) {
                //there are no edges from that node. update outEdgesOfVertexBegin 
                outEdgesOfvertexBegin.push_back(edges.size());
            }
        }

        outEdgesOfvertexBegin.push_back(edges.size());

        //now we sort the edges to have the same sourceId
        //return std::unique_ptr<>(new T(std::forward<Args>(args)...));
        //cpp_utils::graphs::ListGraph<std::string, xyLoc, cost_t>
        cpp_utils::graphs::AdjacentGraph<std::string, xyLoc, cost_t>* realResult = new cpp_utils::graphs::AdjacentGraph<std::string, xyLoc, cost_t>{
            result.getPayload(), 
            vertexPayload,
            edges,
            outEdgesOfvertexBegin
        };
        return std::unique_ptr<cpp_utils::graphs::IImmutableGraph<std::string, xyLoc, cost_t>>{realResult};
        //return boost::make_unique<cpp_utils::graphs::ListGraph<std::string, xyLoc, cost_t>>{result};
    }

}