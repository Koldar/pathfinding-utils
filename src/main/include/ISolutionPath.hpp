#ifndef _PATHFINDING_UTILS_ISOLUTION_PATH_HEADER__
#define _PATHFINDING_UTILS_ISOLUTION_PATH_HEADER__

#include <cpp-utils/vectorplus.hpp>
#include <cpp-utils/igraph.hpp>
#include <cpp-utils/commons.hpp>
#include <cpp-utils/math.hpp>

#include "types.hpp"
#include "operators.hpp"
#include "IPath.hpp"

namespace pathfinding::search {

    using namespace pathfinding;
    using namespace cpp_utils::graphs;

    template <typename G, typename V, typename E>
    class DijkstraSearchAlgorithm;

    /**
     * @brief a vector which represents the solution path generated by A*
     * 
     * @tparam STATE type of state this solution will actually contain
     * @tparam CONST_REF constant reference of a STATE
     */
    template <typename STATE, typename CONST_REF>
    class ISolutionPath : public cpp_utils::vectorplus<STATE> {
        typedef cpp_utils::vectorplus<STATE> Super;
    public:
        using Super::map;
        using Super::filter;
        using Super::addHead;
        using Super::addTail;
    public:
        /**
         * @brief Get the solution cost
         * 
         * @return cost_t 
         */
        virtual cost_t getCost() const = 0;
    public:
        virtual CONST_REF getStart() const {
            return cpp_utils::Types<STATE>::cref(this->at(0));
        }
        virtual CONST_REF getGoal() const {
            return cpp_utils::Types<STATE>::cref(this->at(-1));
        }
    };

    /**
     * @brief A solution which contain search algorithm A* like states
     * 
     * This solution contains only pointers of the actual states
     * 
     * @tparam STATE the type of state inside this solution
     * @tparam CONST_REF constant reference of STATE
     */
    template <typename STATE>
    class StateSolutionPath: public ISolutionPath<const STATE*, const STATE&> {
    public:
        using ISolutionPath<const STATE*, const STATE&>::vectorplus;
        using ISolutionPath<const STATE*, const STATE&>::make;
        using ISolutionPath<const STATE*, const STATE&>::map;
        using ISolutionPath<const STATE*, const STATE&>::addHead;
        using ISolutionPath<const STATE*, const STATE&>::addTail;
        using ISolutionPath<const STATE*, const STATE&>::filter;
    public:
        virtual cost_t getCost() const {
            return this->getGoal().getCost();
        }
    };

    /**
     * @brief A solution which is a sequence of vertices inside a graph
     * 
     * @tparam G payload of the whole graph
     * @tparam V payload of an vertex
     * @tparam E paylaod type of each edge
     */
    template <typename G, typename V, typename E>
    class GraphSolutionPath: public ISolutionPath<cpp_utils::graphs::nodeid_t, cpp_utils::graphs::nodeid_t> {
    public:
        using ISolutionPath<cpp_utils::graphs::nodeid_t, cpp_utils::graphs::nodeid_t>::vectorplus;
        using ISolutionPath<cpp_utils::graphs::nodeid_t, cpp_utils::graphs::nodeid_t>::make;
        using ISolutionPath<cpp_utils::graphs::nodeid_t, cpp_utils::graphs::nodeid_t>::map;
        using ISolutionPath<cpp_utils::graphs::nodeid_t, cpp_utils::graphs::nodeid_t>::addHead;
        using ISolutionPath<cpp_utils::graphs::nodeid_t, cpp_utils::graphs::nodeid_t>::addTail;
        using ISolutionPath<cpp_utils::graphs::nodeid_t, cpp_utils::graphs::nodeid_t>::filter;
    private:
        const cpp_utils::graphs::IImmutableGraph<G, V, E>& g;
        std::function<cost_t(const E&)> costFunction;
    public:
        GraphSolutionPath(const cpp_utils::graphs::IImmutableGraph<G, V, E>& g, std::function<cost_t(const E&)> costFunction): g{g}, costFunction{costFunction} {

        }
    public:
        virtual cost_t getCost() const {
            if (this->size() <= 1) {
                return 0;
            }
            cost_t result = 0;
            for (int i=1; i<this->size(); ++i) {
                result += this->costFunction(g.getEdge((*this)[i-1], (*this)[i]));
            }
            return result;
        }
    };

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
    template <typename G, typename V, typename E, typename STATE, typename CONST_REF>
    void checkPathValid(const IImmutableGraph<G, V, E>& graph, const ISolutionPath<STATE, CONST_REF>& path, std::function<nodeid_t(CONST_REF)> mapper) {

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
    template <typename G, typename V, typename E, typename STATE, typename CONST_REF>
    void checkIfPathOptimal(const IImmutableGraph<G, V, E>& graph, nodeid_t start, nodeid_t goal, const ISolutionPath<STATE, CONST_REF>& actualPath, const std::function<cost_t(const E&)>& costFunction, const std::function<nodeid_t(STATE)>& mapper) {
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

    template <typename G, typename V, typename E, typename STATE, typename CONST_REF>
    void checkIfPathSuboptimalityBound(double bound, const IImmutableGraph<G, V, E>& graph, nodeid_t start, nodeid_t goal, const ISolutionPath<STATE, CONST_REF>& actualPath, const std::function<cost_t(const E&)>& costFunction, const std::function<nodeid_t(STATE)>& mapper) {
        
        auto realActualPath = actualPath.map(mapper);

        checkPathValid<G, V, E>(
            graph,
            realActualPath
        );

        DijkstraSearchAlgorithm<G, V, E> dijkstra{graph, costFunction}; 
        auto expectedPath = dijkstra.search(start, goal);

        double optimalPathCost = static_cast<double>(expectedPath->getCost());
        double actualPathCost = static_cast<double>(actualPath.getCost());
        
        if (cpp_utils::isDefinitelyLessThan(actualPathCost, optimalPathCost, 1e-6)) {
            throw cpp_utils::exceptions::ImpossibleException{"suboptimal path is less than the optimal one!"};
        }

        if (cpp_utils::isDefinitelyGreaterThan(actualPathCost, (optimalPathCost * bound), 1e-6)) {
            log_error("real optimal path costs", expectedPath->getCost(), ". However the algorithm generated a suboptimal path (with bound ", bound, ") which however costs much more! costs", optimalPathCost);
            log_error("expected path", *expectedPath);
            log_error("actual path", realActualPath);
            throw cpp_utils::exceptions::ImpossibleException{"suboptimal path was expected to be within a certain bound from the optimal solution, but it is not!"};
        }
    }

    /**
     * @brief get the optimal path over a graph in a simplistic way as **a sequence of vertices**
     * 
     * If start == goal the sequence generated has size 1
     * 
     * @tparam G type of a paylaod associated to the graph
     * @tparam V type of a payload associated to every vertex
     * @tparam E type of a payload associated to every edge
     * @param graph the graph where to compute the optimal path
     * @param start start of the optimal path
     * @param goal goal of the optimal path
     * @param mapper the function to use to convert E to cost_t
     * @return NodePath an optimal path from @c start to @c goal
     */
    template <typename G, typename V, typename E>
	NodePath getOptimalPathAsVertices(const IImmutableGraph<G, V, E>& graph, nodeid_t start, nodeid_t goal, std::function<cost_t(const E&)> mapper) {
		DijkstraSearchAlgorithm<G, V, E> dijkstra{graph, mapper};
		std::unique_ptr<ISolutionPath<nodeid_t, nodeid_t>> path = ::std::move(dijkstra.search(start, goal));
		NodePath result{*path};
        finer("path is ", *path, path.get());
        finest("done building path with NodePath, which ", result, &result);
        return result;
	}

    /**
     * @brief get the optimal path over a graph in a simplistic way as **a sequence of edges**
     * 
     * If start == goal the sequence generated has size 0
     * 
     * @tparam E 
     * @param graph 
     * @param start 
     * @param goal 
     * @return vectorplus<Edge<E>> 
     */
	template <typename G, typename V, typename E>
	cpp_utils::vectorplus<Edge<E>> getOptimalPathAsEdges(const IImmutableGraph<G, V, E>& graph, nodeid_t start, nodeid_t goal, const std::function<cost_t(const E&)> costFunction) {
		DijkstraSearchAlgorithm<G, V, E> dijkstra{graph, costFunction};
		auto path = dijkstra.search(start, goal);

		cpp_utils::vectorplus<Edge<E>> result{};
		if (path.isEmpty()) {
			return result;
		} else if (path.size() == 1) {
			return result;
		} else {
			for (int i=0; i<(path.size() - 1); ++i) {
				result.add(Edge<E>{path[i], path[i+1], graph.getEdge(path[i], path[i+1])});
			}
			return result;
		}
	}

}

#endif