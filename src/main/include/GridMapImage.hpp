#ifndef _PATHFINDINGUTILS_GRIDMAPIMAGE_HEADER__
#define _PATHFINDINGUTILS_GRIDMAPIMAGE_HEADER__

#include <cpp-utils/ppmImage.hpp>
#include "xyLoc.hpp"

namespace pathfinding::maps {

    using namespace cpp_utils;

    /**
     * @brief A ppm image which represents a grid map
     * 
     */
    class GridMapImage: public cpp_utils::PPMImage {
    public:
        typedef GridMapImage This;
        typedef cpp_utils::PPMImage Super;
    private:
        size_t cellWidth;
        size_t cellHeight;
        size_t gridWidth;
        size_t gridHeight;
        size_t gridMapWidth;
        size_t gridMapHeight;
    public:
        /**
         * @brief Construct a new Grid Map Image object
         * 
         * @param cellWidth width of a cell in pixel
         * @param cellHeight height of a cell in pixel
         * @param horizontalSize height of a horizontal line of the grid in pixel
         * @param verticalSize width or a vertical line of a the grid in pixel
         * @param gridMap gridmap involved
         * @param gridColor color of the grid
         * @param color initial color of the background
         */
        GridMapImage(size_t cellWidth, size_t cellHeight, size_t horizontalSize, size_t verticalSize, const size_t gridMapWidth, size_t gridMapHeight, color_t gridColor, color_t color);
        GridMapImage(const This& o);
        virtual ~GridMapImage();
        This& operator=(const This& o);
        This& operator=(const This&& o);
    public:
        void setGridCellColor(xyLoc loc, const color_t& c);
        void lerpGridCellColor(xyLoc loc, const color_t& c);
        void invertGridCellColor(xyLoc loc);
        void scaleGridCellColor(xyLoc loc, double scale);
        void maxGridCellColor(xyLoc loc, const color_t& c);
        void minGridCellColor(xyLoc loc, const color_t& c);
        void setPixelInGrid(xyLoc loc, xyLoc subPixel, const color_t& c);
        void setPixelInGrid(int x, int y, int subPixelx, int subPixely, const color_t& c);
        color_t getPixelInGrid(xyLoc loc, xyLoc subPixel);
        color_t getPixelInGrid(int x, int y, int subPixelx, int subPixely);
    private:
        void createGrid(color_t gridColor);
    };

}

#endif