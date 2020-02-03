#ifndef _GRIDMAPGRAPHCONVERTER_HEADER__
#define _GRIDMAPGRAPHCONVERTER_HEADER__

#include <boost/smart_ptr/make_unique.hpp>
#include <boost/range/adaptor/map.hpp>

#include <cpp-utils/igraph.hpp>
#include <cpp-utils/listGraph.hpp>
#include <cpp-utils/exceptions.hpp>

#include "xyLoc.hpp"
#include "IMapGraphConverter.hpp"
#include "GridBranching.hpp"
#include "GridMap.hpp"


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
    /**
     * @brief convert the grid map into a JSON which can be read by some other tools
     * 
     * the json generated is divided into 2 parts: vertices and edges.
     * vertices is a dictionary containing for each pair of x-y, the associated node id;
     * edges is a array containing all the edges, expressed as a 3 value array containing sourceid, sinkid and label.
     * 
     * @param map the grid map to convert to json
     * @return std::string 
     */
    template <typename G, typename V, typename E>
    std::string toJson(const cpp_utils::graphs::IImmutableGraph<G, V, E>& map) const {

        std::stringstream ss;
        ss << "{" << std::endl;

        ss << "\t\"vertices\": {" << std::endl;
        int vertexNumber = map.numberOfVertices();
        int current = 0;
        for (auto it=map.beginVertices(); it!=map.endVertices(); ++it) {
            ss << "\t\t{ \"" << it->first << "\": " << "\"" << it->second.x << ", " << it->second.y << "\"}";
            if ((current+1) < vertexNumber) {
                //we need to add a "," except for the last entry
                ss << ",";
            }
            ss << std::endl;
            current += 1;
        }
        ss << "\t}," << std::endl;
        ss << "\t\"edges\": [" << std::endl;

        int edgeNumber = map.numberOfEdges();
        current = 0;
        for (auto it=map.beginEdges(); it!=map.endEdges(); ++it) {
            ss << "\t\t{ \"source\": " << it->getSourceId() << ", \"sink\": " << it->getSinkId() << ", \"label\": " << it->getPayload() << "}";
            if ((current+1) < edgeNumber) {
                ss << ",";
            }
            ss << std::endl;
            current += 1;
        }
        ss << "\t]" << std::endl;

        ss << "}";

        return ss.str();
    }
public:
    virtual void cleanup();
};

}

#endif 