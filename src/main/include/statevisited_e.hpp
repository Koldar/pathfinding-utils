#ifndef _PATHFINDINGUTILS_STATEVISITEDE_HEADER__
#define _PATHFINDINGUTILS_STATEVISITEDE_HEADER__

#include <cpp-utils/AbstractEnum.hpp>

namespace pathfinding::search {

    using namespace cpp_utils;

    struct statevisited_e: public AbstractEnum<statevisited_e> {
    private:
        static std::vector<const statevisited_e*> VALUES;
    public:
        static const statevisited_e UNVISITED;
        static const statevisited_e GENERATED;
        static const statevisited_e EXPANDED;
    public:
        static const statevisited_e& getFirst();
        static const std::vector<const statevisited_e*>& getValues();
    private:
        statevisited_e(const std::string& name): AbstractEnum{name, VALUES} {

        }
    };
}

#endif