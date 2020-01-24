#ifndef _PATHFINDINGUTILS_STATEIMAGEPRODUCER_HEADER__
#define _PATHFINDINGUTILS_STATEIMAGEPRODUCER_HEADER__

#include "utils.hpp"
#include "IPathFindingMap.hpp"

namespace pathfinding::search::listeners {

    using namespace pathfinding;
    using namespace pathfinding::maps;

    /**
     * @brief a class which generate an image every time a node is expanded
     * 
     */
    template <typename G, typename V, typename EORIGINAL, typename EPERTURBATED, typename MAPTYPE, typename IMAGETYPE>
    class StateImageProducer: public StateTracker<G, V, EPERTURBATED> {
    public:
        using This = StateImageProducer<G, V, EORIGINAL, EPERTURBATED, MAPTYPE, IMAGETYPE>;
        using Super = StateTracker<G, V, EPERTURBATED>;
    private:
        const MAPTYPE& map;
        const IImmutableGraph<G,V,EORIGINAL>& originalGraph;
        const IImmutableGraph<G,V,EPERTURBATED>& perturbatedGraph;
        const function_t<EPERTURBATED,cost_t>& costFunction;
    public:
        StateImageProducer(const IImmutableGraph<G,V,EORIGINAL>& originalGraph, const IImmutableGraph<G, V, EPERTURBATED>& perturbatedGraph, const MAPTYPE& map, const function_t<EPERTURBATED,cost_t>& costFunction): Super{perturbatedGraph}, originalGraph{originalGraph}, perturbatedGraph{perturbatedGraph}, map{map}, costFunction{costFunction} {

        }
        virtual ~StateImageProducer() {

        }
        StateImageProducer(const This& o) = default;
        StateImageProducer(This&& o) = default;
        This& operator=(const This& o) = default;
        This& operator=(This&& o) = default;
    public:
        IMAGETYPE* drawMap() const {
            critical("drawing image!");
            IMAGETYPE* image = map.getPPM();

            if (!image->isValid()) {
                throw cpp_utils::exceptions::makeImpossibleException("image is not valid!");
            }

            static function_t<VertexInfo<V>, statevisited_e> mapper = [&](const VertexInfo<V>& v) { return v.state; };
            utils::addExpandedNodesInImage(
                *image, map, 
                this->perturbatedGraph, this->perturbatedGraphState,
                mapper,
                color_t::BLACK,
                color_t::YELLOW.scale(0.4),
                color_t::YELLOW.scale(0.6)
            );

            //draw the grid and the perturbations on it
            utils::addPerturbationsOnMap(
                *image,
                map,
                this->originalGraph, 
                this->perturbatedGraph, 
                color_t::RED, 
                color_t::BLUE, 
                this->costFunction
            );
            
            //add path
            utils::addPathInImage(
                *image,
                map,
                this->perturbatedGraph,
                Super::currentSolution,
                color_t::YELLOW.scale(0.8),
                color_t::YELLOW.scale(0.8),
                color_t::YELLOW.scale(0.8)
            );
            
            critical("end drawing image!");
            return image;
        }
    };

}

#endif