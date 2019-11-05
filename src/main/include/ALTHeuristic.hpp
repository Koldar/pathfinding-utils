#ifndef _PATHFINDING_UTILS_ALT_HEURISTIC_HEADER__
#define _PATHFINDING_UTILS_ALT_HEURISTIC_HEADER__

#include <cpp-utils/igraph.hpp>

#include "LandmarkDatabase.hpp"
#include "IHeuristic.hpp"

namespace pathfinding::search {

    /**
     * Represents a class which can be injected into ::abstract_pathfinding_astar in order to
     * 
     * @note
     * this implementation assume the underlying graph is, albeit represented via a directed graph, an undirected graph
     */
    template <typename STATE, typename G, typename V>
    class ALTHeuristic : public IHeuristic<STATE> {
        using This = ALTHeuristic<STATE, G, V>;
    public:
        ALTHeuristic(const IImmutableGraph<G,V, cost_t>& graph, const LandmarkDatabase<G,V>& landmarkDatabase): graph{graph}, landmarkDatabase{landmarkDatabase} {

        }
        ALTHeuristic(const This& o): graph{o.graph}, landmarkDatabase{o.landmarkDatabase} {

        }
        ALTHeuristic(This&& o): graph{::std::move(o.graph)}, landmarkDatabase{o.landmarkDatabase} {

        }
        This& operator =(const This& o) = delete;
        This& operator =(This&& o) = delete;
        virtual ~ALTHeuristic() {

        }
    public:
        virtual cost_t getHeuristic(const STATE& current, const STATE* goal) {
            cost_t result = 0;

            debug("evaluating state", current);
            for (nodeid_t landmark : this->landmarkDatabase.getLandmarks()) {
                debug("considering landmark", mapper(landmark));

                cost_t a = this->landmarkDatabase.getDistanceFromLandmarkToNode(landmark, current.getPosition());
                cost_t b = this->landmarkDatabase.getDistanceFromLandmarkToNode(landmark, goal->getPosition());

                debug(current, "->", graph.getVertex(landmark), ":", a);
                debug(*goal, "->", graph.getVertex(landmark), ":", b);
                long diff = static_cast<long>(a) - static_cast<long>(b);

                cost_t value{diff > 0 ? diff : -diff};

                debug("value=", value, "max=",result);

                if (value > result) {
                    result = value;
                }
            }

            return result;
        }

        virtual bool isAdmissible() const {
            //TODO the value has been set quickly, recheck when you have time
            return true;
        }
        
        virtual bool isConsistent() const {
            //TODO the value has been set quickly, recheck when you have time
            return true;
        }
    public:
        void cleanup() {

        }
    public:
        virtual MemoryConsumption getByteMemoryOccupied() const {
            return sizeof(*this);
        }
    private:
        const LandmarkDatabase<G, V>& landmarkDatabase;
        const IImmutableGraph<G, V, cost_t>& graph;
    };


    

// void DifferentialHeuristic::clear() {

// }

}

#endif