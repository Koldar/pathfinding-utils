#ifndef _MOVINGAIGRIDMAPREADER_HEADER__
#define _MOVINGAIGRIDMAPREADER_HEADER__

#include "IPathFindingMapReader.hpp"
#include "GridMap.hpp"
#include <unordered_map>
#include <fstream>
#include <iostream>
#include <cpp-utils/mapplus.hpp>

namespace pathfinding {

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
private:
    template <typename... OTHER>
    MovingAIGridMapReader& addTerrain(char symbol, cost_t cost, OTHER... others) {
        this->terrainCost[symbol] = cost;
        return this->addTerrain(others...);
    }
public:
    template <typename... OTHER>
    MovingAIGridMapReader(OTHER... args): terrainCost{} {
        this->addTerrain(args...);
    }
    MovingAIGridMapReader& addTerrain(char symbol, cost_t cost) {
        this->terrainCost[symbol] = cost;
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
        for (auto y=0; y<height; ++y) {
            for (auto x=0; x<width; ++x) {
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
            width, height, this->terrainCost
        };
    }
};

}

#endif