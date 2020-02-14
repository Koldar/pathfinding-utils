#ifndef _PATHFINDINGUTILS_PATHVALIDATORS_HEADER__
#define _PATHFINDINGUTILS_PATHVALIDATORS_HEADER__

#include <cpp-utils/igraph.hpp>
#include <cpp-utils/functional.hpp>

#include "ISolutionPath.hpp"
#include "DijkstraSearchAlgorithm.hpp"
#include "IPathFindingMap.hpp"


namespace pathfinding::validator {

    using namespace cpp_utils;
    using namespace cpp_utils::graphs;
    using namespace pathfinding;
    using namespace pathfinding::maps;
    using namespace pathfinding::search;

    /**
     * @brief check if a path is valid or not
     * 
     * 
     * @tparam G type of the graph payload
     * @tparam V 
     * @tparam E 
     * @param graph 
     * @param path 
     */
    template <typename G, typename V, typename E, typename STATE, typename IN_VECTOR, typename CONST_REF>
    void checkPathValid(const IImmutableGraph<G, V, E>& graph, const ISolutionPath<STATE, IN_VECTOR, CONST_REF>& path, cpp_utils::function_t<STATE, nodeid_t> mapper) {

        nodeid_t previous;
        for (int i=0; i<path.size(); ++i) {
            nodeid_t current = mapper(path[i]);
            if (!graph.containsVertex(current)) {
                    critical("node in ",i, " with id=", current, "is not in graph ", graph);
                    throw cpp_utils::exceptions::ImpossibleException{};
                }

            if (i == 0) {
                previous = current;
            } else {
                if (!graph.hasEdge(previous, current)) {
                    critical("there is not an edge from ", previous, " to ", current, "in graph ", graph);
                    throw cpp_utils::exceptions::ImpossibleException{};
                }
                previous = current;
            }
        }
    }

    template <typename G, typename V, typename E>
    void checkPathValid(const IImmutableGraph<G, V, E>& graph, const std::vector<nodeid_t>& path) {
        nodeid_t previous;
        for (int i=0; i<path.size(); ++i) {
            nodeid_t current = path[i];
            if (!graph.containsVertex(current)) {
                    critical("node in ",i, " with id=", current, "is not in graph ", graph);
                    throw cpp_utils::exceptions::ImpossibleException{};
                }

            if (i == 0) {
                previous = current;
            } else {
                if (!graph.hasEdge(previous, current)) {
                    critical("there is not an edge from ", previous, " to ", current, "in graph ", graph);
                    throw cpp_utils::exceptions::ImpossibleException{};
                }
                previous = current;
            }
        }
    }

    

    /**
     * @brief Check if a given path is optimal
     * 
     * @tparam G payload type of the whole graph
     * @tparam V payload type of each vertex
     * @tparam E payload type of each edge
     * @param graph the graph where we want to check cover
     * @param start start vertex
     * @param goal goal vertex
     * @param actualPath the path we want to check if it's optimal
     * @param edgeWeightConverter function that converts `E` of @c graph into a cost.
     * @param mapper function that converts the stats in @c actualPath into nodes
     * @return true if the actual path is optimal
     * @return false otherwise
     */
    template <typename G, typename V, typename E, typename STATE, typename IN_VECTOR, typename CONST_REF>
    void checkIfPathOptimal(const IImmutableGraph<G, V, E>& graph, nodeid_t start, nodeid_t goal, const ISolutionPath<STATE, IN_VECTOR, CONST_REF>& actualPath, const cpp_utils::function_t<E, cost_t>& costFunction, const cpp_utils::function_t<STATE, nodeid_t>& mapper) {
        auto realActualPath = actualPath.map(mapper);

        checkPathValid<G, V, E>(
            graph,
            realActualPath
        );

        DijkstraSearchAlgorithm<G, V, E> dijkstra{graph, costFunction}; 
        auto expectedPath = dijkstra.search(start, goal);
        
        if (expectedPath->getCost() != actualPath.getCost()) {
            log_error("real optimal path costs", expectedPath->getCost(), ". However the algorithm generated a path which costs", actualPath.getCost());
            log_error("expected path", *expectedPath);
            log_error("actual path", realActualPath);
            throw cpp_utils::exceptions::ImpossibleException{};
        }
    }

    template <typename G, typename V, typename E, typename STATE, typename IN_VECTOR, typename CONST_REF>
    void checkIfPathSuboptimalityBound(double bound, const IImmutableGraph<G, V, E>& graph, nodeid_t start, nodeid_t goal, const ISolutionPath<STATE, IN_VECTOR, CONST_REF>& actualPath, const cpp_utils::function_t<E, cost_t>& costFunction, const cpp_utils::function_t<STATE, nodeid_t>& mapper) {
        auto realActualPath = actualPath.map(mapper);

        checkPathValid<G, V, E>(
            graph,
            realActualPath
        );

        DijkstraSearchAlgorithm<G, V, E> dijkstra{graph, costFunction}; 
        auto expectedPath = dijkstra.search(start, goal);

        double expectedOptimalPathCost = static_cast<double>(expectedPath->getCost());
        double actualOptimalPathCost = static_cast<double>(actualPath.getCost());
        
        if (actualOptimalPathCost < expectedOptimalPathCost) {
            throw cpp_utils::exceptions::ImpossibleException{"suboptimal path is less than the optimal one!"};
        }

        if (actualOptimalPathCost > (bound * expectedOptimalPathCost)) {
            log_error("BOUND IS INVALID!");
            log_error("expected optimal path costs", expectedOptimalPathCost);
            log_error("actual bound cost to ", bound * expectedOptimalPathCost);
            log_error("actual optimal path costs", actualOptimalPathCost);
            log_error("expected path", *expectedPath);
            log_error("actual path", realActualPath);
            throw cpp_utils::exceptions::ImpossibleException{"suboptimal path was expected to be within a certain bound from the optimal solution, but it is not!"};
        }
    }

    template <typename G, typename V, typename E, typename STATE, typename IN_VECTOR, typename CONST_REF>
    void checkIfPathSuboptimalityBound(double bound, const IImmutableGraph<G, V, E>& graph, nodeid_t start, nodeid_t goal, const ISolutionPath<STATE, IN_VECTOR, CONST_REF>& actualPath, const cpp_utils::function_t<E, cost_t>& costFunction, const cpp_utils::function_t<STATE, nodeid_t>& mapper, const IPathFindingMap& map) {
        auto realActualPath = actualPath.map(mapper);

        checkPathValid<G, V, E>(
            graph,
            realActualPath
        );

        DijkstraSearchAlgorithm<G, V, E> dijkstra{graph, costFunction}; 
        auto expectedPath = dijkstra.search(start, goal);

        double expectedOptimalPathCost = static_cast<double>(expectedPath->getCost());
        double actualOptimalPathCost = static_cast<double>(actualPath.getCost());
        
        if (actualOptimalPathCost < expectedOptimalPathCost) {
            throw cpp_utils::exceptions::ImpossibleException{"suboptimal path is less than the optimal one!"};
        }

        if (actualOptimalPathCost > (bound * expectedOptimalPathCost)) {
            log_error("BOUND IS INVALID!");
            log_error("expected optimal path costs", expectedOptimalPathCost);
            log_error("actual bound cost to ", bound * expectedOptimalPathCost);
            log_error("actual optimal path costs", actualOptimalPathCost);
            log_error("expected path", *expectedPath);
            log_error("actual path", realActualPath);
            //print graph. We assume it's gridmap for now
            const GridMap& gridMap = static_cast<const GridMap&>(map);

            auto image = std::unique_ptr<GridMapImage>{gridMap.getPPM()};
            auto startVertex = static_cast<xyLoc>(graph.getVertex(start));
            auto goalVertex = static_cast<xyLoc>(graph.getVertex(goal));
            //mark expected optimal path
            DijkstraSearchAlgorithm<G, V, E> dijkstra{graph, costFunction}; 
            auto expectedPath = dijkstra.search(start, goal);
            for (auto it=expectedPath->cbegin(); it<expectedPath->cend(); ++it) {
                auto tmpVertex = static_cast<xyLoc>(graph.getVertex(*it));
                image->lerpGridCellColor(startVertex, color_t::BLUE.darker(0.3));
            }
            //mark actual optimal path
            auto* actualTmp = &actualPath.getGoal();
            while (actualTmp != nullptr) {
                auto tmpVertex = static_cast<xyLoc>(graph.getVertex(actualTmp->getPosition()));
                image->lerpGridCellColor(startVertex, color_t::RED.darker(0.3));
                actualTmp = actualTmp->getParent();    
            }
            //mark start and goal
            image->setGridCellColor(startVertex, color_t::YELLOW.darker(0.2));
            image->setGridCellColor(goalVertex, color_t::YELLOW.darker(0.4));

            boost::filesystem::path name{"expected=blue_actual=red.jpeg"};
            image->saveJPEG(name);

            throw cpp_utils::exceptions::ImpossibleException{"suboptimal path was expected to be within a certain bound from the optimal solution, but it is not! Map available as", name};
        }
    }

}

#endif 