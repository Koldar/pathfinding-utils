#include "GridBranching.hpp"

namespace pathfinding::maps {

    std::ostream& operator <<(std::ostream& ss, const GridBranching& b) {
        switch (b) {
            case GridBranching::FOUR_CONNECTED: { ss << 4; return ss; }
            case GridBranching::SIX_CONNECTED: { ss << 6; return ss; }
            case GridBranching::EIGHT_CONNECTED: { ss << 8; return ss; }
            default:
                throw cpp_utils::exceptions::makeInvalidScenarioException(b);
        }
    }

}