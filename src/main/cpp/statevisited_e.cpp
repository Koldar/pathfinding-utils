#include "statevisited_e.hpp"

namespace pathfinding::search {

    std::vector<const statevisited_e*> statevisited_e::VALUES;

    const statevisited_e statevisited_e::UNVISITED = statevisited_e{"UNVISITED"};
    const statevisited_e statevisited_e::GENERATED = statevisited_e{"GENERATED"};
    const statevisited_e statevisited_e::EXPANDED = statevisited_e{"EXPANDED"};

    const statevisited_e& statevisited_e::getFirst() {
        return *VALUES[0];
    }

    const std::vector<const statevisited_e*>& statevisited_e::getValues() {
        return statevisited_e::VALUES;
    }
}