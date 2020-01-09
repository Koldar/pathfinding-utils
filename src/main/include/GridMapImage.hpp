#ifndef _PATHFINDINGUTILS_GRIDMAPIMAGE_HEADER__
#define _PATHFINDINGUTILS_GRIDMAPIMAGE_HEADER__

namespace pathfinding::maps {

    /**
     * @brief A ppm image which represents a grid map
     * 
     */
    class GridMapImage: public PPMImage {
    public:
    
    public:
        GridMapImage(const size_t& width, const size_t& height, color_t color) : PPMImage{width, height} {
		this->setAllPixels(color);
	}
    };

}

#endif