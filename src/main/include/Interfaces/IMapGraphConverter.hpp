#ifndef _IMAPGRAPHCONVERTER_HEADER__
#define _IMAPGRAPHCONVERTER_HEADER__

#include <cpp-utils/ICleanable.hpp>

namespace pathfinding::maps {

    using namespace cpp_utils;

/**
 * @brief a class which allows you to convert a IPathFindingMap into a graph
 * 
 */
template <typename MAP, typename GRAPH>
class IMapGraphConverter: public ICleanable {
    /**
     * @brief returns a graph representing the map
     * 
     * @return GRAPH the graph generated
     */
    virtual std::unique_ptr<GRAPH> toGraph(const MAP& map) const = 0;
};

}

#endif