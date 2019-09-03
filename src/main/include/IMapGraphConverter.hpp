#ifndef _IMAPGRAPHCONVERTER_HEADER__
#define _IMAPGRAPHCONVERTER_HEADER__

namespace pathfinding {

/**
 * @brief a class which allows you to convert a IPathFindingMap into a graph
 * 
 */
template <typename MAP, typename GRAPH>
class IMapGraphConverter {
    /**
     * @brief returns a graph representing the map
     * 
     * @return GRAPH the graph generated
     */
    virtual GRAPH toGraph(const MAP& map) const = 0;
};

}

#endif