#ifndef _GRIDMAP_HEADER__
#define _GRIDMAP_HEADER__

#include <unordered_map>
#include <string>
#include <vector>

#include <cpp-utils/listGraph.hpp>
#include <cpp-utils/vectorplus.hpp>
#include <cpp-utils/IImageable.hpp>

#include "types.hpp"
#include "IPathFindingMap.hpp"
#include "xyLoc.hpp"


namespace pathfinding::maps {

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
    class GridMap: public IPathFindingMap, IImageable {
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
    private:
        size_t computeSize() const {
            return this->cells
                .map<cost_t>([&](char x) {return this->terrainCost.at(x);})
                .filter([&](cost_t x) { return x.isNotInfinity(); })
                .size();
        }
        int toVectorCoord(xyLoc xy) const {
            return toVectorCoord(xy.y, xy.x);
        }
        int toVectorCoord(ucood_t y, ucood_t x) const {
            return y * this->width + x;
        }
        xyLoc toXyLoc(ucood_t x) const {
            return xyLoc{x / this->width, x % this->width};
        }
    public:
        GridMap(const std::string& name, const std::vector<char>& cells, ucood_t width, ucood_t height, std::unordered_map<char, cost_t> terrainCost): name{name}, cells{cells}, width{width}, height{height}, terrainCost{terrainCost} {
            this->size = this->computeSize();
        }

        virtual ~GridMap() {
        }

        GridMap(const GridMap& other): name{other.name}, cells{other.cells}, width{other.width}, height{other.height}, terrainCost{other.terrainCost} {
        }

        GridMap(GridMap&& other): name{other.name}, cells{other.cells}, width{other.width}, height{other.height}, terrainCost{other.terrainCost}, size{other.size} {
        }

        GridMap& operator= (const GridMap& map) = delete;
        GridMap operator= (GridMap&& map) = delete;

        ucood_t getWidth() const {
            return this->width;
        }

        ucood_t getHeight() const {
            return this->height;
        }

        char getCellTerrain(xyLoc loc) const {
            return this->cells[this->toVectorCoord(loc)];
        }

        cost_t getCellCost(xyLoc loc) const {
            debug("xyLoc is", loc, "cell is", this->cells[this->toVectorCoord(loc)]);
            return this->terrainCost.at(this->cells[this->toVectorCoord(loc)]);
        }

        bool isTraversable(xyLoc loc) const {
            return getCellCost(loc).isNotInfinity();
        }
    protected:
        const PPMImage* getPPM() const {
            callExternalProgram("rm -f /tmp/getPPM.png /tmp/getPPM.ppm /tmp/getPPM.dot");

            int cellWidth = 3;
            int cellHeight = 3;
            int borderWidth = 1;
            int borderHeight = 1;

            PPMImage* result = new PPMImage{
                (borderWidth + cellWidth) * this->width + borderWidth, 
                (borderHeight + cellHeight) * this->height + borderHeight
            };

            result->setPixel();

            std::ofstream f;
            f.open("/tmp/getPPM.ppm", std::ofstream::out | std::ofstream::trunc);

            f << "digraph {\n";
            info("iterate over vertices...");
            for (auto it=this->beginVertices(); it!=this->endVertices(); ++it) {
                info("drawing vertex", it->first);
                f << "N" << it->first << " [label=\"id:" << it->first << "\\n" << it->second << "\"];\n";
            }

            info("iterate over edges...");
            for (auto it=this->beginEdges(); it!=this->endEdges(); ++it) {
                info("drawing edge", it->getSourceId(), "->", it->getSinkId());
                f << "N" << it->getSourceId() << " -> N" << it->getSinkId() << " [label=\"" << it->getPayload() << "\"];\n";
            }
            f << "}\n";

            f.close();

            callExternalProgram("dot -Tpng -o /tmp/getPPM.png /tmp/getPPM.dot");
            //https://askubuntu.com/a/84415/703658
            callExternalProgram("convert -depth 8 -compress none /tmp/getPPM.png /tmp/getPPM.ppm");
            PPMImage* result = new PPMImage{"/tmp/getPPM.ppm"};
            return result;
        }
    private:
        void createGrid(PPMImage& image, int cellWidth, int cellHeight, int borderWidth, int borderHeight) {
            //horizontal lines
            for (ucood_t y=0; y<this->height; y+=(cellHeight + borderHeight)) {
                for (ucood_t)
                for (ucood_t x=0; x<this->width; ++x) {

                }
            }
        }
    public:
        virtual const std::string& getName() const {
            return this->name;
        }
        virtual size_t getSize() const {
            return this->size;
        }
    };

}

#endif