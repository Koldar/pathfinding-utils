#ifndef _PATHFINDING_UTILS_GRIDMAP_HEADER__
#define _PATHFINDING_UTILS_GRIDMAP_HEADER__

#include <unordered_map>
#include <string>
#include <vector>

#include <cpp-utils/listGraph.hpp>
#include <cpp-utils/vectorplus.hpp>
#include <cpp-utils/IImageable.hpp>

#include "types.hpp"
#include "IPathFindingMap.hpp"
#include "xyLoc.hpp"
#include "GridMapImage.hpp"

namespace pathfinding::maps {

    using namespace cpp_utils;

    /**
     * @brief 
     * 
     * As an example:
     * 
     * @code
     * | | | | | | 
     * | | | | |@|
     * | |@|@| |@|
     * | | | | | |
     * @endcode
     * 
     * width is 5 while height is 4. size is 16.
     * 
     * @tparam BRANCHING a GridMapBranching representing how the underlying graph is structured
     */
    class GridMap: public IPathFindingMap, public IImageable {
    private:
        /**
         * @brief name of the map
         * 
         */
        const std::string name;
        /**
         * @brief map where each key is a symbol in the gridmap filename representing a cell while the value is the cost an agent needs to pay to traverse such cell
         * 
         * @code
         *  terrainCost['.'] = 1000
         *  terrainCost['T'] = 1500
         *  terrainCost['@'] = cost_t::INFTY
         * @endcode
         */
        std::unordered_map<char, cost_t> terrainCost;
        /**
         * @brief map where each key is a symbol in the gridmap filename representing a cell while the value is the color we're going to use when printing the gridmap on a file
         * 
         * @code
         *  terrainCost['.'] = color_t::WHITE
         *  terrainCost['T'] = color_t::GREEN
         *  terrainCost['@'] = color_t::BLACK
         * @endcode
         */
        std::unordered_map<char, color_t> terrainColor;
        /**
         * @brief number of columns this map has
         * 
         */
        const ucood_t width;
        /**
         * @brief number of rows this map has
         * 
         */
        const ucood_t height;
        /**
         * @brief content of the map.
         * 
         * Costs are associated vcia ::terrainCost
         * 
         */
        cpp_utils::vectorplus<char> cells;
        /**
         * @brief number of cells which has their cost different than +infinity
         * 
         */
        size_t size;
    public:
        GridMap(const std::string& name, const std::vector<char>& cells, ucood_t width, ucood_t height, std::unordered_map<char, cost_t> terrainCost, std::unordered_map<char, color_t> terrainColor);

        virtual ~GridMap();

        GridMap(const GridMap& other);
        GridMap(GridMap&& other);
        GridMap& operator= (const GridMap& map) = delete;
        GridMap& operator= (GridMap&& map) = delete;
    private:
        size_t computeSize() const;
        int toVectorCoord(xyLoc xy) const;
        int toVectorCoord(ucood_t y, ucood_t x) const;
        xyLoc toXyLoc(ucood_t x) const;
    public:
        ucood_t getWidth() const;
        ucood_t getHeight() const;
        char getCellTerrain(xyLoc loc) const;
        cost_t getCellCost(xyLoc loc) const;
        bool isTraversable(xyLoc loc) const;
    public:
        //TODO the method should rfeturn a modificable image since some times we want to change it!
        const GridMapImage* getPPM() const;
    public:
        virtual const std::string& getName() const;
        virtual size_t getSize() const;
    };

}

#endif