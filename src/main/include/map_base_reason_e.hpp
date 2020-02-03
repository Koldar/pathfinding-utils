#ifndef _PATHFINDINGUTILS_MAP_BASE_REASON_HEADER__
#define _PATHFINDINGUTILS_MAP_BASE_REASON_HEADER__

#include <cpp-utils/AbstractEnum.hpp>

namespace pathfinding {

    using namespace cpp_utils;

    /**
     * @brief the standard enumeration to use if you don't know the reason for a generation of a state
     * 
     */
    struct map_base_reason_e: public AbstractEnum<map_base_reason_e> {
    private:
        static std::vector<const map_base_reason_e*> VALUES;
    public:
        static const map_base_reason_e INPUT;
        static const map_base_reason_e GENERATED;
    public:
        static const map_base_reason_e& getFirst();
        static const std::vector<const map_base_reason_e*>& getValues();
    private:
        map_base_reason_e(const std::string& name): AbstractEnum{name, VALUES} {

        }
    };
}

#endif