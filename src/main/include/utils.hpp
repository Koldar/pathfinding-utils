#ifndef _PATHFINDINGUTILS_UTILS_HEADER__
#define _PATHFINDINGUTILS_UTILS_HEADER__

#include <boost/filesystem.hpp>
#include <cstdlib>

#include <cpp-utils/adjacentGraph.hpp>
#include <cpp-utils/functional.hpp>
#include <cpp-utils/igraph.hpp>
#include <cpp-utils/SetPlus.hpp>
#include <cpp-utils/serializers.hpp>

#include "GridMap.hpp"
#include "GridMapImage.hpp"
#include "DijkstraSearchAlgorithm.hpp"


namespace pathfinding::utils {

    using namespace cpp_utils;
    using namespace cpp_utils::graphs;
    using namespace pathfinding::maps;
    using namespace pathfinding::search;

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

    /**
     * @brief Loads the perturbated graph starting from the perturbations saved into a file
     * 
     * @tparam G the type of the paylaod of the whole graph
     * @tparam V the type of the payload of each vertex
     * @tparam E the type of the edges to load from the file
     * @param path path where the perturbated edges are stored
     * @param originalGraph the original graph before the perturbations
     * @param mapper a function that map edge cost_t in the @c originalGraph into 
     * @return AdjacentGraph<G, V, E>* a graph containing the perturbatations. This needs to be freed manually
     */
    template <typename G, typename V, typename E>
    IImmutableGraph<G, V, E>* loadPerturbatedMap(const boost::filesystem::path& path, const IImmutableGraph<G, V, cost_t>& originalGraph, const cpp_utils::function_t<cost_t, E>& mapper) {
        info("perturbated graph file should be located in", path);

        if (!boost::filesystem::exists(path)) {
            throw cpp_utils::exceptions::FileOpeningException{path};
        }
         
        AdjacentGraph<G,V,E>* result = new AdjacentGraph<G, V, E>{
            *originalGraph.mapEdges(mapper)
        };

        FILE* f = fopen(path.native().c_str(), "rb");
        SetPlus<Edge<E>> perturbatedEdges{};
        cpp_utils::serializers::loadFromFile(f, perturbatedEdges);
        fclose(f);

        //apply the new perturbated edges
        for (auto perturbatedEdge : perturbatedEdges) {
            result->changeWeightUndirectedEdge(
                perturbatedEdge.getSourceId(), 
                perturbatedEdge.getSinkId(), 
                perturbatedEdge.getPayload()
            );
        }

        return result;
    }

    /**
	 * @brief Create a Grid Map Perturbated Map object
	 * 
	 * @tparam G 
	 * @tparam V 
	 * @tparam E 
	 * @param gridmap 
	 * @param baseMapGraph 
	 * @param perturbatedGraph 
	 * @param worseEdgeColor 
	 * @param betterEdgeColor 
	 * @return GridMapImage& 
	 */
	template <typename G, typename V, typename E>
	GridMapImage* createGridMapPerturbatedMap(const GridMap& gridMap, const IImmutableGraph<G, V, cost_t>& baseMapGraph, const IImmutableGraph<G, V, E>& perturbatedGraph, color_t worseEdgeColor, color_t betterEdgeColor, const std::function<cost_t(const E&)>& costFunction) {
		auto image = const_cast<GridMapImage*>(gridMap.getPPM());
		for (auto it=perturbatedGraph.beginEdges(); it!=perturbatedGraph.endEdges(); ++it) {
			if (!it->getPayload().isPerturbated()) {
				continue;
			}
			xyLoc source = perturbatedGraph.getVertex(it->getSourceId());
			xyLoc sink = perturbatedGraph.getVertex(it->getSinkId());
			int pixel1x, pixel1y, pixel2x, pixel2y;
			switch (xyLoc::getDirection(source, sink)) {
				case Direction::SOUTHEAST:  pixel1x=2; pixel1y=2; pixel2x=0; pixel2y=0; break;
				case Direction::EAST:       pixel1x=2; pixel1y=1; pixel2x=0; pixel2y=1; break;
				case Direction::NORTHEAST:  pixel1x=2; pixel1y=0; pixel2x=0; pixel2y=2; break;

				case Direction::SOUTHWEST:  pixel1x=0; pixel1y=2; pixel2x=2; pixel2y=0; break;
				case Direction::WEST:       pixel1x=0; pixel1y=1; pixel2x=2; pixel2y=1; break;
				case Direction::NORTHWEST:  pixel1x=0; pixel1y=0; pixel2x=2; pixel2y=2; break;
				
				case Direction::SOUTH:      pixel1x=1; pixel1y=2; pixel2x=1; pixel2y=0; break;
				case Direction::NORTH:      pixel1x=1; pixel1y=0; pixel2x=1; pixel2y=2; break;
				default:
					throw cpp_utils::exceptions::makeImpossibleException("invalid direction!");
			}
			cost_t oldWeight = baseMapGraph.getEdge(it->getSourceId(), it->getSinkId());
			cost_t newWeight = costFunction(perturbatedGraph.getEdge(it->getSourceId(), it->getSinkId()));
			color_t perturbatedColor;
			if (newWeight > oldWeight) {
				//cost has increased
				perturbatedColor = worseEdgeColor;
			} else {
				//cost has decreased
				perturbatedColor = betterEdgeColor;
			}
			image->setPixelInGrid(source.x, source.y, pixel1x, pixel1y, perturbatedColor); 
			image->setPixelInGrid(sink.x, sink.y, pixel2x, pixel2y, perturbatedColor);
		}

		return image;
	}

}

#endif