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
#include "NodePath.hpp"
#include "statevisited_e.hpp"


namespace pathfinding::utils {

    using namespace cpp_utils;
    using namespace cpp_utils::graphs;
    using namespace pathfinding::maps;
    using namespace pathfinding::search;

	/**
	 * @brief generate a path from a search state
	 * 
	 * @tparam implementation of a ISearchState
	 * @param mapper function converting state into a node over a graph
	 * @return NodePath 
	 */
	template <typename STATE>
	NodePath getNodePath(const STATE& state, const function_t<STATE, nodeid_t>& mapper) {
		NodePath result{};
		auto tmp = &state;
		debug("getNodePath is", tmp);
		while (tmp != nullptr) {
			debug("getNodePath is", tmp);
			result.add(mapper(*tmp));
			debug("mapper is", mapper(*tmp));
			tmp = tmp->getParent();
		}
		return result;
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
		std::unique_ptr<ISolutionPath<nodeid_t, nodeid_t, nodeid_t>> path = ::std::move(dijkstra.search(start, goal));
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
         
		auto resultTmp = std::unique_ptr<IImmutableGraph<G,V,E>>{originalGraph.mapEdges(mapper)};
        AdjacentGraph<G,V,E>* result = new AdjacentGraph<G, V, E>{*resultTmp};

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
	 * @brief Alters an image (assumed to be a GridMapImage) to add all the perturbations present in `perturbatedGraph`
	 * 
	 * @tparam G 
	 * @tparam V 
	 * @tparam E 
	 * @param gridmap 
	 * @param baseMapGraph 
	 * @param perturbatedGraph 
	 * @param worseEdgeColor 
	 * @param betterEdgeColor 
	 * @return GridMapImage& the @c image altered
	 */
	template <typename G, typename V, typename E>
	PPMImage& addPerturbationsOnMap(PPMImage& image, const IPathFindingMap& map, const IImmutableGraph<G, V, cost_t>& baseMapGraph, const IImmutableGraph<G, V, E>& perturbatedGraph, color_t worseEdgeColor, color_t betterEdgeColor, const std::function<cost_t(const E&)>& costFunction) {
		GridMapImage& gridMapImage = static_cast<GridMapImage&>(image);
		const GridMap& gridMap = static_cast<const GridMap&>(map);
		
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
			gridMapImage.setPixelInGrid(source.x, source.y, pixel1x, pixel1y, perturbatedColor); 
			gridMapImage.setPixelInGrid(sink.x, sink.y, pixel2x, pixel2y, perturbatedColor);
		}

		return gridMapImage;
	}

	/**
	 * @brief update the @c image by coloring the states which have been expanded/generated
	 * 
	 * @tparam G1 type of the payload of the graph representig the map where we're searching
	 * @tparam V1 type of the payload of eachv vertex representig the map where we're searching
	 * @tparam E1 type of the payload of each edge representig the map where we're searching
	 * @tparam G2 type of the payload of the graph representig the map storing the state of each node
	 * @tparam V2 type of the payload of each vertex representig the map storing the state of each node
	 * @tparam E2 type of the payload of each edge representig the map storing the state of each node
	 * @param image the image to update
	 * @param map the actual map where we are searching
	 * @param graphMap the repersentation of the graph where we are searching
	 * @param stateGraph the graph we use to fetch the state of each vertex
	 * @param mapper function converting V2 into states
	 * @param unvisitedColor color to use to color the vertex we haven't even visited
	 * @param expandedColor color to use to colro the vertices we have expanded at least once
	 * @param generatedColor color to use to colro the vertices we have generated but not expanded
	 * @return PPMImage& the updated image
	 */
	template <typename G1, typename V1, typename E1, typename G2, typename V2, typename E2>
	PPMImage& addExpandedNodesInImage(PPMImage& image, const IPathFindingMap& map, const IImmutableGraph<G1,V1,E1>& graphMap, const IImmutableGraph<G2,V2,E2>& stateGraph, const function_t<V2, statevisited_e>& mapper, const color_t& unvisitedColor, const color_t& expandedColor, const color_t& generatedColor) {
		GridMapImage& gridMapImage = static_cast<GridMapImage&>(image);
		const GridMap& gridMap = static_cast<const GridMap&>(map);

		//now add to the map the expanded nodes
		for (auto it=stateGraph.beginVertices(); it!=stateGraph.endVertices(); ++it) {
			auto data = mapper(it->second);
			xyLoc loc = graphMap.getVertex(it->first);
			if (data == statevisited_e::UNVISITED) {
				//do nothing;
			} else if (data == statevisited_e::GENERATED) {
				gridMapImage.lerpGridCellColor(loc, generatedColor); 
			} else if (data == statevisited_e::EXPANDED) {
				gridMapImage.lerpGridCellColor(loc, expandedColor);
			} else {
				throw cpp_utils::exceptions::InvalidScenarioException{"state visited", data};
			}
		}

		return gridMapImage;
	}

	/**
	 * @brief add 
	 * 
	 * @tparam G 
	 * @tparam V 
	 * @tparam E 
	 * @param image 
	 * @param map 
	 * @param graph 
	 * @param path 
	 * @param path 
	 * @param start 
	 * @param goal 
	 * @return PPMImage& 
	 */
	template <typename G, typename V, typename E>
	PPMImage& addPathInImage(PPMImage& image, const IPathFindingMap& map, const IImmutableGraph<G,V,E>& graph, const NodePath& path, const color_t& pathColor, const color_t& startColor, const color_t& goalColor) {
		GridMapImage& gridMapImage = static_cast<GridMapImage&>(image);
		const GridMap& gridMap = static_cast<const GridMap&>(map);

		color_t c{};
		critical("putting path of size", path.size());
		for (int i=0; i<path.size(); ++i) {
			if (i == 0) {
				c = startColor;
			} else if (i == path.size()) {
				c = goalColor;
			} else {
				c = pathColor;
			}
			xyLoc loc = graph.getVertex(path.at(i));
			gridMapImage.setGridCellColor(loc, c);
		}
		return gridMapImage;
	}
}

#endif