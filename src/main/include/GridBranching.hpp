#ifndef _GRIDBRANCHING_HEADER__
#define _GRIDBRANCHING_HEADER__

namespace pathfinding::search {

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

}

#endif