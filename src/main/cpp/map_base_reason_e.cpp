#include "map_base_reason_e.hpp"

namespace pathfinding {

    std::vector<const map_base_reason_e*> map_base_reason_e::VALUES;

    const map_base_reason_e map_base_reason_e::INPUT = map_base_reason_e{"INPUT"};
    const map_base_reason_e map_base_reason_e::GENERATED = map_base_reason_e{"GENERATED"};

    const map_base_reason_e& map_base_reason_e::getFirst() {
        return *VALUES[0];
    }

    const std::vector<const map_base_reason_e*>& map_base_reason_e::getValues() {
        return map_base_reason_e::VALUES;
    }
}