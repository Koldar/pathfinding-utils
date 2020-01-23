#ifndef _PATHFINDINGUTILS_STATEIMAGEPRODUCER_HEADER__
#define _PATHFINDINGUTILS_STATEIMAGEPRODUCER_HEADER__

#include "utils.hpp"

namespace pathfinding::search::listeners {

    using namespace pathfinding;

    /**
     * @brief a class which generate an image every time a node is expanded
     * 
     */
    template <typename G, typename V, typename IMAGETYPE>
    class StateImageProducer: public StateTracker<G, E> {
    public:
        using This = StateImageProducer<G,V,E>;
        using Super = StateTracker<G,E>;
    private:
        const IPathFindingMap& map;
        const IImmutableGraph<G,V,E>& perturbatedGraph;
        const function_t<E,cost_t>& costFunction;
    public:
        template <typename EORIGINAL, typename EPERTURBATED>
        StateImageProducer(const ImmutableGraph<G,V,E>& originalGraph, const IImmutableGraph<G, V, E>& perturbatedGraph, const IPathFindingMap& map, const function_t<E,cost_t>& costFunction): Super{perturbatedGraph}, map{map}, costFunction{costFunction} {

        }
        virtual ~StateImageProducer() {

        }
        StateImageProducer(const This& o) = default;
        StateImageProducer(This&& o) = default;
        This& operator=(const This& o) = default;
        This& operator=(This&& o) = default;
    public:
        IMAGETYPE* drawMap(const sta) const {
            IMAGETYPE* image = map->getPPM();

            autofunction_t<statevisited_e> identity = [&](const statevisited_e& a) { return a; };
            *image = utils::addExpandedNodesInImage(
                *image,
                map,
                Super::states,
                identity,
                color_t::BLACK,
                color_t::YELLOW.scale(0.4),
                color_t::YELLOW.scale(0.6)
            );

            //draw the grid and the perturbations on it
            costFunction_t<PerturbatedCost> costFunction = [&] (const PerturbatedCost& p) { return p.getCost();};
            *image = addPerturbationsOnMap(
                *image,
                map,
                originalMap, 
                perturbatedGraph, 
                color_t::RED, 
                color_t::BLUE, 
                costFunction
            );
            
            //add path
            *image = addPathInImage(
                *image,
                map,
                graph,
                Super2::solution,
                color_t::YELLOW.scale(0.8),
                color_t::YELLOW.scale(0.8),
                color_t::YELLOW.scale(0.8)
            )
            
            image->saveBMP("./query1");
            delete image;
        }
    }

}

#endif