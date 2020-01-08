#include "GridMap.hpp"

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
            .filter([&](cost_t x) { return x.isNotInfinity(); })
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

    const PPMImage* GridMap::getPPM() const {
        int cellWidth = 3;
        int cellHeight = 3;
        int borderWidth = 1;
        int borderHeight = 1;

        PPMImage* result = new PPMImage{
            (borderWidth + cellWidth) * this->width + borderWidth, 
            (borderHeight + cellHeight) * this->height + borderHeight
        };

        this->createGrid(*result, cellWidth, cellHeight, borderWidth, borderHeight);

        for (ucood_t y=0; y<this->height; ++y) {
            for (ucood_t x=0; x<this->width; ++x) {
                this->setCellColor(
                    *result, 
                    xyLoc{x, y}, 
                    cellWidth, cellHeight,
                    borderWidth, borderHeight, 
                    this->terrainColor.at(this->getCellTerrain(xyLoc{x, y}))
                );
            }
        }

        return result;
    }

    void GridMap::setCellColor(PPMImage& image, xyLoc cell, int cellWidth, int cellHeight, int borderWidth, int borderHeight, const color_t& color) const {
        ucood_t topy = cell.y * (borderHeight + cellHeight) + borderHeight;
        ucood_t topx = cell.x * (borderWidth + cellWidth) + borderWidth;
        image.setRectanglePixel(topy, topx, topy + cellHeight, topx + cellWidth, color, true, true);
    }

    void GridMap::setPixelInCell(PPMImage& image, xyLoc cell, xyLoc pixel, int cellWidth, int cellHeight, int borderWidth, int borderHeight, const color_t& color) const {
        ucood_t topy = cell.y * (borderHeight + cellHeight) + borderHeight;
        ucood_t topx = cell.x * (borderWidth + cellWidth) + borderWidth;
        image.setPixel(topy + pixel.y, topx + pixel.x, color);
    }

    void GridMap::createGrid(PPMImage& image, int cellWidth, int cellHeight, int borderWidth, int borderHeight) const {
        //horizontal lines
        for (ucood_t y=0; y<this->height; y+=(cellHeight + borderHeight)) {
            image.setRectanglePixel(y, 0, y + borderHeight, image.getWidth(), color_t::BLACK, true, false);
        }
        //vertical lines
        for (ucood_t x=0; x<this->width; x+=(cellWidth + borderWidth)) {
            image.setRectanglePixel(0, x, image.getHeight(), x + borderHeight, color_t::BLACK, false, true);
        }
    }

    const std::string& GridMap::getName() const {
        return this->name;
    }
    size_t GridMap::getSize() const {
        return this->size;
    }
}