#ifndef _MOVINGAIGRIDMAPREADER_HEADER__
#define _MOVINGAIGRIDMAPREADER_HEADER__

#include <unordered_map>
#include <fstream>
#include <iostream>

#include <cpp-utils/mapplus.hpp>
#include <cpp-utils/ppmImage.hpp>

#include "IPathFindingMapReader.hpp"
#include "GridMap.hpp"

namespace pathfinding::maps {

/**
 * @brief A factory which loads gridmaps
 * 
 * The supported format can be viewed at https://movingai.com/benchmarks/formats.html
 * 
 */

class MovingAIGridMapReader: public IPathFindingMapReader<GridMap> {
private:
    /**
     * @brief map where each key is a symbol in the gridmap filename representing a cell while the value is the cost an agent needs to pay to traverse such cell
     * 
     * @code
     *  terrainCost['.'] = 1000
     *  terrainCost['T'] = 1500
     *  terrainCost['@'] = cost_t::INFTY
     * @endcode
     */
    cpp_utils::MapPlus<char, cost_t> terrainCost;
    cpp_utils::MapPlus<char, color_t> terrainColor;
private:
    template <typename... OTHER>
    MovingAIGridMapReader& addTerrain(char symbol, cost_t cost, color_t color, OTHER... others) {
        this->terrainCost[symbol] = cost;
        return this->addTerrain(others...);
    }
public:
    template <typename... OTHER>
    MovingAIGridMapReader(OTHER... args): terrainCost{}, terrainColor{} {
        this->addTerrain(args...);
    }
    MovingAIGridMapReader& addTerrain(char symbol, cost_t cost, color_t color) {
        this->terrainCost[symbol] = cost;
        this->terrainColor[symbol] = color;
        return *this;
    }

    virtual GridMap load(const boost::filesystem::path& mapPath) {
        //load the map
        std::ifstream ifs{mapPath.string()};
        std::string str;
        ucood_t width;
        ucood_t height;

        // HEADER

        ifs >> str;
        if (str != "type") {
            throw cpp_utils::exceptions::InvalidFormatException<boost::filesystem::path, std::string>{mapPath, str};
        }
        ifs >> str;
        if (str != "octile") {
            throw cpp_utils::exceptions::InvalidFormatException<boost::filesystem::path, std::string>{mapPath, str};
        }
        ifs >> str >> height;
        if (str != "height") {
            throw cpp_utils::exceptions::InvalidFormatException<boost::filesystem::path, std::string>{mapPath, str};
        }
        ifs >> str >> width;
        if (str != "width") {
            throw cpp_utils::exceptions::InvalidFormatException<boost::filesystem::path, std::string>{mapPath, str};
        }

        ifs >> str;
        if (str != "map") {
            throw cpp_utils::exceptions::InvalidFormatException<boost::filesystem::path, std::string>{mapPath, str};
        }

        // MAP BODY
        char cellSymbol;
        cpp_utils::vectorplus<char> cells{};
        finest("width is", width, "height is", height);
        for (auto y=0; y<height; ++y) {
            for (auto x=0; x<width; ++x) {
                debug("reading cell x=", x, "y=", y);
                ifs >> cellSymbol;
                if (!this->terrainCost.containsKey(cellSymbol)) {
                    throw cpp_utils::exceptions::ElementNotFoundException<char, cpp_utils::MapPlus<char, cost_t>>{cellSymbol, this->terrainCost};
                }
                cells.add(cellSymbol);
            }
        }

        ifs.close();

        return GridMap{
            mapPath.stem().string(),
            cells,
            width, height, 
            this->terrainCost,
            this->terrainColor
        };
    }
};

}

#endif