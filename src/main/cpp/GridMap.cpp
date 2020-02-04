#include "GridMap.hpp"
#include <cpp-utils/operators.hpp>

namespace pathfinding::maps {
    GridMap::GridMap(const std::string& name, const std::vector<char>& cells, ucood_t width, ucood_t height, std::unordered_map<char, cost_t> terrainCost, std::unordered_map<char, color_t> terrainColor): name{name}, cells{cells}, width{width}, height{height}, terrainCost{terrainCost}, terrainColor{terrainColor} {
        this->size = this->computeSize();
    }

    GridMap::~GridMap() {
    }

    GridMap::GridMap(const GridMap& other): name{other.name}, cells{other.cells}, width{other.width}, height{other.height}, terrainCost{other.terrainCost}, terrainColor{other.terrainColor} {
    }

    GridMap::GridMap(GridMap&& other): name{other.name}, cells{other.cells}, width{other.width}, height{other.height}, terrainCost{other.terrainCost}, size{other.size}, terrainColor{other.terrainColor} {
    }

    size_t GridMap::computeSize() const {
        return this->cells
            .map<cost_t>([&](char x) {return this->terrainCost.at(x);})
            .select([&](cost_t x) { return x.isNotInfinity(); })
            .size();
    }
    int GridMap::toVectorCoord(xyLoc xy) const {
        return toVectorCoord(xy.y, xy.x);
    }
    int GridMap::toVectorCoord(ucood_t y, ucood_t x) const {
        return y * this->width + x;
    }
    xyLoc GridMap::toXyLoc(ucood_t x) const {
        return xyLoc{x / this->width, x % this->width};
    }

    ucood_t GridMap::getWidth() const {
        return this->width;
    }

    ucood_t GridMap::getHeight() const {
        return this->height;
    }

    char GridMap::getCellTerrain(xyLoc loc) const {
        return this->cells[this->toVectorCoord(loc)];
    }

    cost_t GridMap::getCellCost(xyLoc loc) const {
        debug("xyLoc is", loc, "cell is", this->cells[this->toVectorCoord(loc)]);
        return this->terrainCost.at(this->cells[this->toVectorCoord(loc)]);
    }

    bool GridMap::isTraversable(xyLoc loc) const {
        return getCellCost(loc).isNotInfinity();
    }

    GridMapImage* GridMap::getPPM() const {
        auto result = new GridMapImage{3, 3, 1, 1, this->getWidth(), this->getHeight(), color_t::BLACK, color_t::WHITE};

        for (auto y=0; y<this->getHeight(); ++y) {
            for (auto x=0; x<this->getWidth(); ++x) {
                xyLoc loc{static_cast<ucood_t>(x), static_cast<ucood_t>(y)};
                debug("terrain is ", this->getCellTerrain(loc), "terrain colors are", this->terrainColor);
                result->setGridCellColor(loc, this->terrainColor.at(this->getCellTerrain(loc)));
            }
        }

        return result;
    }

    const std::string& GridMap::getName() const {
        return this->name;
    }
    size_t GridMap::getSize() const {
        return this->size;
    }
}