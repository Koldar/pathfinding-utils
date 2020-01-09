#include "GridMapImage.hpp"
#include <cpp-utils/math.hpp>

namespace pathfinding::maps {

    GridMapImage::GridMapImage(size_t cellWidth, size_t cellHeight, size_t horizontalSize, size_t verticalSize, const size_t gridMapWidth, size_t gridMapHeight, color_t gridColor, color_t color) : Super{(cellWidth + verticalSize)* gridMapWidth, (cellHeight + horizontalSize)* gridMapHeight, color}, cellWidth{cellWidth}, cellHeight{cellHeight}, gridWidth{verticalSize}, gridHeight{horizontalSize}, gridMapWidth{gridMapWidth}, gridMapHeight{gridMapHeight} {
        this->createGrid(gridColor);
    }
    GridMapImage::GridMapImage(const GridMapImage::This& o): Super{o}, cellWidth{o.cellWidth}, cellHeight{o.cellHeight}, gridWidth{o.gridWidth}, gridHeight{o.gridHeight}, gridMapWidth{o.gridMapWidth}, gridMapHeight{o.gridMapHeight} {

    }

    GridMapImage::~GridMapImage() {

    }

    GridMapImage::This& GridMapImage::operator=(const GridMapImage::This& o) {
        Super::operator=(o);
        this->cellWidth = o.cellWidth;
        this->cellHeight = o.cellHeight;
        this->gridWidth = o.gridWidth;
        this->gridHeight = o.gridHeight;
        this->gridMapWidth = o.gridMapWidth;
        this->gridMapHeight = o.gridMapHeight;
        return *this;
    }

    GridMapImage::This& GridMapImage::operator=(const GridMapImage::This&& o) {
        *this = ::std::move(o);
        this->cellWidth = o.cellWidth;
        this->cellHeight = o.cellHeight;
        this->gridWidth = o.gridWidth;
        this->gridHeight = o.gridHeight;
        this->gridMapWidth = o.gridMapWidth;
        this->gridMapHeight = o.gridMapHeight;
        return *this;
    }

    void GridMapImage::setGridCellColor(xyLoc loc, const color_t& c) {
        this->setRectanglePixel(
            loc.y*(this->cellHeight + this->gridHeight) + this->gridHeight,
            loc.x*(this->cellWidth + this->gridWidth) + this->gridWidth,
            loc.y*(this->cellHeight + this->gridHeight) + this->gridHeight + this->cellHeight,
            loc.x*(this->cellWidth + this->gridWidth) + this->gridWidth + this->cellWidth,
            c
        );
    }

    void GridMapImage::setPixelInGrid(xyLoc loc, xyLoc subPixel, const color_t& c) {
        this->setPixelInGrid(loc.x, loc.y, subPixel.x, subPixel.y, c);
    }

    void GridMapImage::setPixelInGrid(int x, int y, int subPixelx, int subPixely, const color_t& c) {
        int px = x*(this->cellWidth + this->gridWidth) + this->gridWidth + cpp_utils::ringBound(subPixelx, static_cast<int>(this->cellWidth));
        int py = y*(this->cellHeight + this->gridHeight) + this->gridHeight + cpp_utils::ringBound(subPixely, static_cast<int>(this->cellHeight));
        critical("setting psubpixel", px, py, "total is", this->getWidth(), this->getHeight());
        this->setPixel(px, py, c);
    }

    void GridMapImage::createGrid(color_t gridColor) {
        for (int y=0; y<this->gridMapHeight; ++y) {
            this->setRectanglePixel(
                y*(this->cellHeight + this->gridHeight), 0, 
                y*(this->cellHeight + this->gridHeight) + this->gridHeight, this->getWidth(),
                gridColor
            );
        }
        for (int x=0; x<this->gridMapWidth; ++x) {
            this->setRectanglePixel(
                0, x*(this->cellWidth + this->gridWidth),
                this->getHeight(), x*(this->cellWidth + this->gridWidth) + this->gridWidth,
                gridColor
            );
        }
    }
}