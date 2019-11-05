#ifndef _DIJKSTRA_SEARCH_ALGORITHM_HEADER__
#define _DIJKSTRA_SEARCH_ALGORITHM_HEADER__

#include "ISearchAlgorithm.hpp"
#include <cpp-utils/KHeaps.hpp>
#include <cpp-utils/igraph.hpp>

namespace pathfinding::search {

    using namespace cpp_utils::graphs;

    /**
     * @brief A unidrectional Dijkstra algorithm which stops as soon as it finds the goal
     * 
     * If you need the actual Dijkstra algorithm and not this variant, please use DijkstraAlgorithm class
     * 
     * @tparam G 
     * @tparam V 
     */
    template <typename G, typename V>
    class DijkstraSearchAlgorithm: public ISearchAlgorithm<nodeid_t, nodeid_t, nodeid_t> {
    public:
        using ISearchAlgorithm<nodeid_t, nodeid_t, nodeid_t>::search;
    private:
        /**
         * @brief underlying graph Dijkstra works on
         * 
         */
        const IImmutableGraph<G,V,cost_t>& g;
        /**
         * @brief vector indexed by id of grpah vertices.
         * 
         * In the cell there is the cost we need to pay to optimally reach the given vertex from the source
         * 
         */
        std::vector<cost_t> distancesFromSource;
        /**
         * id represents the id we want to reach while the value is the nodewe need to arrive from
         */
        std::vector<nodeid_t> bestPreviousNode;
        /**
         * @brief queue used by disjkstra to order the nodes
         * 
         */
        cpp_utils::min_id_heap<nodeid_t, cost_t> queue;
    public:
        DijkstraSearchAlgorithm(const IImmutableGraph<G,V,cost_t>& g): g{g}, distancesFromSource{}, bestPreviousNode{}, queue{g.numberOfVertices()} {
        }
        virtual ~DijkstraSearchAlgorithm() {

        }
        DijkstraSearchAlgorithm& operator =(const DijkstraSearchAlgorithm& other) = delete;
    public:
        virtual void cleanup() {
            this->distancesFromSource.resize(g.numberOfVertices());
            this->bestPreviousNode.resize(g.numberOfVertices());
            queue.cleanup();
            debug("distancesFromSource", distancesFromSource);
            std::fill(this->distancesFromSource.begin(), this->distancesFromSource.end(), cost_t::INFTY);
            std::fill(this->bestPreviousNode.begin(), this->bestPreviousNode.end(), 0);
        }
    public:
        virtual std::string getName() const {
            return "Dijkstra";
        }
        virtual void setupSearch(const nodeid_t* start, const nodeid_t* goal) {
            this->cleanup();
        }
        virtual void tearDownSearch() {
        }
    protected:
        virtual nodeid_t performSearch(nodeid_t& start, const nodeid_t* goal) {
            this->distancesFromSource[start] = 0;
            nodeid_t actualGoal = *goal;

            finest("Starting DIJKSTRA start is", start, "goal is", actualGoal);
            if (start == actualGoal) {
                return actualGoal;
            }

            for(int i=0; i<g.getOutDegree(start); ++i) {
                OutEdge<cost_t> edge = g.getOutEdge(start, i);
                this->reach(edge.getSinkId(), 0 + edge.getPayload(), start);
            }

            while(!queue.isEmpty()) {
                nodeid_t v = queue.pop();
                finest("popped from queue", v);

                if (v == actualGoal) {
                    finest(v, "is equal to the goal", actualGoal);
                    //goal found
                    return actualGoal;
                }

                for(auto edge : g.getOutEdges(v)) {
                    finest("edge is", edge);
                    this->reach(edge.getSinkId(), distancesFromSource[v] + edge.getPayload(), v);
                }
            }

            DO_ON_DEBUG {
                //verify triangular disequality
                for(nodeid_t b=0; b<g.numberOfVertices(); ++b) {
                    for(auto edge : g.getOutEdges(b)) {
                        nodeid_t c = edge.getSinkId();
                        cost_t ab = distancesFromSource[b];
                        cost_t ac = distancesFromSource[c];
                        cost_t bc = edge.getPayload();
                        finest(ab, bc, ac, ((ab + bc) >= (ac)));
                        if (!((ab + bc) >= (ac))) {
                            log_error("nodes A=", start, ", B=", b, ", C=", c, ": triangular disequality not verified: AB=", ab, " BC=", bc, " AC=", ac, "(AB + BC) >= AC");
                            throw cpp_utils::exceptions::ImpossibleException{};
                        }
                    }
                }
            }
            //no solution found
            throw search::SolutionNotFoundException{};
        }
        virtual std::unique_ptr<ISolutionPath<nodeid_t, nodeid_t>> buildSolutionFromGoalFetched(nodeid_t start, nodeid_t actualGoal, const nodeid_t* goal) {
            auto result = new GraphSolutionPath<G, V>{this->g};

            nodeid_t tmp = actualGoal;
            result->addHead(tmp);
            while (tmp != start) {
                nodeid_t prev = this->bestPreviousNode[tmp];
                assert(prev != tmp);
                result->addHead(prev);
                tmp = prev;
            }

            return std::unique_ptr<GraphSolutionPath<G, V>>{result};
        }
        virtual cost_t getSolutionCostFromGoalFetched(nodeid_t start, nodeid_t actualGoal, const nodeid_t* goal) const {
            return this->distancesFromSource[actualGoal];
        }
    public:
        //we need to specify a goal
        virtual std::unique_ptr<ISolutionPath<nodeid_t, nodeid_t>> search(nodeid_t start, bool performSetup=true, bool performTearDown=true) {
            throw cpp_utils::exceptions::ImpossibleException{};
        }

        //we need to specify a goal
        virtual cost_t getSolutionCost(nodeid_t start, bool performSetup=true, bool performTearDown=true) {
            throw cpp_utils::exceptions::ImpossibleException{};
        }
    private:
        void reach(nodeid_t v, cost_t d, nodeid_t bestPrevious) {
            if(d < distancesFromSource[v]) {
                finest(v, "could be reached in", distancesFromSource[v], " but now we can reach it via ", bestPrevious, " in ", d, "! update!!!");
                queue.pushOrDecrease(v, d);
                distancesFromSource[v] = d;
                this->bestPreviousNode[v] = bestPrevious;
            }
        }
    };

}

#endif