#include "statevisited_e.hpp"

namespace pathfinding::search {
    const statevisited_e statevisited_e::UNVISITED = statevisited_e{"UNVISITED"};
    const statevisited_e statevisited_e::GENERATED = statevisited_e{"GENERATED"};
    const statevisited_e statevisited_e::EXPANDED = statevisited_e{"EXPANDED"};
}