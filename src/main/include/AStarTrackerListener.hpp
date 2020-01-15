#ifndef _PATHFINDINGUTILS_ASTARTRACKERLISTENER_HEADER__
#define _PATHFINDINGUTILS_ASTARTRACKERLISTENER_HEADER__

#include <cpp-utils/AbstractEnum.hpp>

#include "CountAStarListener.hpp"


namespace pathfinding::search {

    using namespace cpp_utils;

    struct statevisited_e: public AbstractEnum {
    public:
        static const statevisited_e UNVISITED;
        static const statevisited_e GENERATED;
        static const statevisited_e EXPANDED;
    private:
        statevisited_e(const std::string& name): AbstractEnum{name} {

        }
    };

    /**
     * @brief a listener that keeps track of the state we have expanded, and the optimal path generated
     * 
     * the listener allows you to keep track of heuristic average timings, set of node
     * expanded and the first solution an algorithm has found
     * 
     * @tparam STATE type of the A* state
     */
    template <typename G, typename V, typename E, typename STATE>
    class AstarTrackerListener: public CountAStarListener<STATE> {
    public:
        typedef AstarTrackerListener<G, V, E, STATE> This;
        typedef CountAStarListener<STATE> Super;
    private:
        AdjacentGraph<G,statevisited_e,E> states;
    public:
        AstarTrackerListener(const IImmutableGraph<G,V,E>& originalGraph): Super{}, states{} {
            function_t<V, statevisited_e> lambda = [&](const V& e) { return statevisited_e::UNVISITED;};
            auto p = originalGraph->mapVertices(lambda);
            this->states = *p;
            delete p;
        }
        virtual ~AstarTrackerListener() = default;
        AstarTrackerListener(const This& o) = default;
        AstarTrackerListener(This&& o) = default;
        This& operator =(const This& o) = default;
        This& operator = (This&& o) = default;
    public:
        const IImmutableGraph<G, statevisited_e, E>& getVisitedStates() const {
            return this->states;
        }
    public:
        virtual void onNodeExpanded(const STATE& s) {
            Super::onNodeExpanded(s);
            this->states.getVertex(s.getId()) = statevisited_e::EXPANDED;
        }
        virtual void onNodeGenerated(const STATE& s) {
            Super::onNodeGenerated(s);
            this->states.changeVertexPayload(s.getId(), statevisited_e::GENERATED);
        }
        void cleanup() {
            Super::cleanup();
            for (auto it=this->states.beginVertices(); it!=this->states.endVertices(); ++it) {
                this->states.changeVertexPayload(it->first, statevisited_e::UNVISITED);
            }
        }

    };

}

#endif