#ifndef _IPATHFINDINGMAP_HEADER__
#define _IPATHFINDINGMAP_HEADER__

#include <string>

namespace pathfinding {

/**
 * @brief represents a map where the agents traverse in order to reach an objective
 * 
 * @tparam G 
 * @tparam V 
 * @tparam E 
 */
class IPathFindingMap {
public:
    /**
     * @brief the name of the map
     * 
     * @return const std::string& map name
     */
    virtual const std::string& getName() const = 0;
    /**
     * @brief number of available locations the map has
     * 
     * @return size_t the number of locations an agent can traverse
     */
    virtual size_t getSize() const = 0;
};

}

#endif