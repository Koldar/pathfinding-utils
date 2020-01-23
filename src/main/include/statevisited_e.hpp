#ifndef _PATHFINDINGUTILS_STATEVISITEDE_HEADER__
#define _PATHFINDINGUTILS_STATEVISITEDE_HEADER__

#include <cpp-utils/AbstractEnum.hpp>

namespace pathfinding::search {

    using namespace cpp_utils;

    struct statevisited_e: public AbstractEnum {
    public:
        static const statevisited_e UNVISITED;
        static const statevisited_e GENERATED;
        static const statevisited_e EXPANDED;
    private:
        statevisited_e(const std::string& name): AbstractEnum{name} {

        }
    };
}

#endif