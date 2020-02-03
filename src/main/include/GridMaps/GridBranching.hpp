#ifndef _GRIDBRANCHING_HEADER__
#define _GRIDBRANCHING_HEADER__

#include <iostream>

#include <cpp-utils/exceptions.hpp>

namespace pathfinding::maps {

    /**
     * @brief represents how many connections a grid map cell has
     * 
     */
    enum class GridBranching {
        /**
         * @brief 
         * 
         * The cells are squares
         * Here the connections available are
         * 
         * @code
         * | |*| |
         * |*|N|*|
         * | |*| |
         * @endcode
         * 
         */
        FOUR_CONNECTED,
        /**
         * @brief 
         * 
         * The cells are squares
         * Here the connections available are
         * 
         * @code
         * |*|*|*|
         * |*|N|*|
         * |*|*|*|
         * @endcode
         * 
         */
        EIGHT_CONNECTED,
        /**
         * @brief 
         * 
         * The cells are hexagons
         * Here the connections available are:
         * 
         * @code
         * | |*|*| |
         *  |*|N|*|
         * | |*|*| |
         * @endcode
         * 
         */
        SIX_CONNECTED
    };

    namespace GridBranchingMethods {

        template<GridBranching N>
        constexpr int getBranching() {
            throw cpp_utils::exceptions::ImpossibleException{};
        }

        template <>
        constexpr int getBranching<GridBranching::FOUR_CONNECTED>() {
            return 4;
        }
        template <>
        constexpr int getBranching<GridBranching::SIX_CONNECTED>() {
            return 6;
        }
        template <>
        constexpr int getBranching<GridBranching::EIGHT_CONNECTED>() {
            return 8;
        }

    }

    std::ostream& operator <<(std::ostream& ss, const GridBranching& b);

}

#endif