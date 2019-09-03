#ifndef _ABSTRACTMAPFACTORY_HEADER__
#define _ABSTRACTMAPFACTORY_HEADER__

#include <string>
#include <boost/filesystem.hpp>

namespace pathfinding {

/**
 * @brief allows you to load a map from the file system
 * 
 */
template <typename PATHFINDINGMAP>
class AbstractMapFactory {
public:
    /**
     * @brief given a map path pointing to a map, it generates a instance of ::IPathFindingMap
     * 
     * The same file may produce different maps (for instance, the same grid map may produce 4 connecte dmaps or 8 connected ones).
     * This choicen is left to the factory implementation
     * 
     * @param mapPath name of the map to load
     * @return PATHFINDINGMAP 
     */
    virtual PATHFINDINGMAP load(const boost::filesystem::path& mapPath) = 0;
};

}

#endif