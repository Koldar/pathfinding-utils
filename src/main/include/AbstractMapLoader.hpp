#ifndef _ABSTRACTMAPLOADER_HEADER__
#define _ABSTRACTMAPLOADER_HEADER__

#include <string>

namespace pathfinding {

class IPathFindingMap {
public:
    virtual const std::string& getName() const = 0;
    virtual size_t size() const = 0;
    //toGraph()
};

class AbstractMapLoader {
public:

};

}

#endif