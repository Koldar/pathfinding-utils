#ifndef _PATHFINDING_UTILS_DIJKSTRA_ALGORITHM_HEADER__
#define _PATHFINDING_UTILS_DIJKSTRA_ALGORITHM_HEADER__

#include <cpp-utils/ICleanable.hpp>
#include <cpp-utils/igraph.hpp>
#include <cpp-utils/KHeaps.hpp>
#include <vector>

namespace pathfinding::algorithms {

    using namespace cpp_utils;
    using namespace cpp_utils::graphs;

    /**
     * @brief A standar dimplementation of Dijkstra algorithm
     * 
     * Use it like this:
     * 
     * @code
     * DijkstraAlgorithm dijkstra{graph};
     * dijkstra.run(3); //3 is the id of the node where dijkstra starts
     * //poll value
     * dijkstra.getBestMoveToOptimallyReach(6); //get best move to perform to reach 6 from 3
     * dijkstra.getDistanceFromSource(6); //get distance from 3 to 6
     * //run for another use
     * dijkstra.run(4);
     * @endcode
     * 
     * If you want to use Dijkstra as a search algorithm on pathfinding, please use DijkstraSearchAlgorithm instead
     * 
     * @tparam G 
     * @tparam V 
     */
    template <typename G, typename V>
	class DijkstraAlgorithm: public ICleanable {
		using This = DijkstraAlgorithm<G, V>;
	private:
		/**
		 * @brief the graph where we need to run dijkstra over
		 * 
		 */
		const IImmutableGraph<G, V, cost_t>& g;
		/**
		 * @brief queue of dijkstra algorithm
		 * 
		 */
		cpp_utils::min_id_heap<nodeid_t, cost_t> q;
		/**
		 * distance array
		 *
		 * node ids are indexes of this array. The value in each cell represents the minimum distance got up until now between the source node and the given one.
		 * For example if in index 4 there is the value 60 it means the minimum path distance from source_node and the vertex with id 4 is 60.
		 *
		 */
		std::vector<cost_t> dist;
		/**
		 * reachable array
		 *
		 * an array where the index is a node id while the values are the best direction to keep if you want to optimally go to the given dpf::nodeid_t
		 * starting from source_node. the number should be interpreted as a bit set. A 1 in a given bit \f$ i \f$ means that you can optimally reach
		 * the node id vertex following the direction \f$ i \f$ of the AdjGraph.
		 */
		std::vector<moveid_t> allowed;
	public:
		DijkstraAlgorithm(const IImmutableGraph<G,V,cost_t>& g): g{g}, q(g.numberOfVertices()), dist(g.numberOfVertices()), allowed(g.numberOfVertices()) {

		}
		virtual ~DijkstraAlgorithm() {

		}
		DijkstraAlgorithm(const This& o): g{o.g}, q(o.q), dist(o.dist), allowed(o.allowed) {

		}
		DijkstraAlgorithm(This&& o): g{::std::move(o.g)}, q{::std::move(o.q)}, dist{::std::move(o.dist)}, allowed{::std::move(o.allowed)} {

		}
		This& operator =(const This& o) {
			this->q = o.q;
			this->dist = o.dist;
			this->allowed = o.allowed;
			return *this;
		}
		This& operator =(This&& o) {
			this->q = ::std::move(o.q);
			this->dist = ::std::move(o.dist);
			this->allowed = ::std::move(o.allowed);
			return *this;
		}
	public:
		/**
		 * execute dijkstra algorithm
		 *
		 * @param[in] source_node id of the node where we start disjkstra from
		 * @return
		 * 	@li an array containing the best direction to use to reach every node starting from @c source_node;
		 * 	@li if a cell has "0" there is no best directions;
		 */
		void run(nodeid_t source_node) {
            this->cleanup();

			dist[source_node] = 0;		
			allowed[source_node] = 0;

			auto reach = [&](nodeid_t v, cost_t d, moveid_t first_move){
				if(d < dist[v]) {
					debug("we reached", v, "faster! It was", dist[v], "but now it is ", d);
					q.pushOrDecrease(v, d);
					dist[v] = d;
					allowed[v] = first_move;
				} else if(d == dist[v]) {
					debug("we reached", v, "as fast as before! It was", dist[v], "but also ", d);
					allowed[v] |= first_move;
				}
			};

			for(moveid_t i=0; i<g.getOutDegree(source_node); ++i){
				auto a = g.getOutEdge(source_node, i);
				reach(a.getSinkId(), a.getPayload(), 1 << i);
			}

			while(!q.isEmpty()) {
				nodeid_t currentNode = q.pop();
				debug("popped from queue", currentNode, "distance", dist[currentNode]);

				for(moveid_t i=0; i<g.getOutDegree(currentNode); ++i) {
					OutEdge<cost_t> outEdge = g.getOutEdge(currentNode, i);
					reach(outEdge.getSinkId(), dist[currentNode] + outEdge.getPayload(), allowed[currentNode]);
				}
			}

			DO_ON_DEBUG {
				for(nodeid_t u=0; u<g.numberOfVertices(); ++u) {
					finest("out degree of", u, "(", g.getVertex(u), ") is", g.getOutDegree(u));
					for(moveid_t i=0; i<g.getOutDegree(u); ++i) {
						OutEdge<cost_t> uv = g.getOutEdge(u, i);
						nodeid_t v = uv.getSinkId();
						cost_t uvCost = uv.getPayload();

						finest("source is ", source_node, "(", g.getVertex(source_node), ") edge from ", u, "(", g.getVertex(u), ") to", v, "(", g.getVertex(v), ") is", uvCost, "distance[", source_node, ", ", u, "]=", dist[u], "distance[", source_node, ", ", v, "]=", dist[v]);

						if(!((dist[u] + uvCost).greaterOrEqualThan(dist[v]))) {
							log_error("source:", source_node, "u:", u, " v:", v, "dist[u] + uv.weight >= dist[v] (", dist[u], "+", uvCost, ">=", dist[v], ")");
							throw cpp_utils::exceptions::ImpossibleException{};
						}
					}
				}
			}
		}

        cost_t getDistanceFromSource(nodeid_t target) const {
            return this->dist[target];
        }

        cost_t getBestMoveToOptimallyReach(nodeid_t target) const {
            return this->allowed[target];
        }

        /**
         * @brief Get the Distances object
         * 
         * @return const std::vector<cost_t>&  avector, indexed by vertices of graph where in each cell there is the distance
         * of the given vertex from `source` of the last `run` call.
         */
        const std::vector<cost_t>& getDistances() const {
            return this->dist;
        }

		cost_t distance(nodeid_t target) const {
			return dist[target];
		}
    public:
        void cleanup() {
            std::fill(dist.begin(), dist.end(), cost_t::INFTY);
			std::fill(allowed.begin(), allowed.end(), 0);
        }
	};

}

#endif