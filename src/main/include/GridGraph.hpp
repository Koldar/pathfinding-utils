#ifndef _PATHFINDINGUTILS_GRIDGRAPH_HEADER__
#define _PATHFINDINGUTILS_GRIDGRAPH_HEADER__

#include <cpp-utils/igraph.hpp>

namespace pathfinding::datastructures {

    template<typename G, typename V, typename E>
    class GridGraph: public AdjacentGraph<G,V,E> {

    };

}

#endif