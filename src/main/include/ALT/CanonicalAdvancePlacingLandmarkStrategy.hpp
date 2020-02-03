#ifndef _PATHFINDING_UTILS_CANONICAL_ADVANCE_PLACNIG_LANDMARK_STRATEGY_HEADER__
#define _PATHFINDING_UTILS_CANONICAL_ADVANCE_PLACNIG_LANDMARK_STRATEGY_HEADER__

#include "AbstractLandmarkPlacingStrategy.hpp"

namespace pathfinding::search {

    template <typename G, typename V>
	class CanonicalAdvancePlacingLandmarkStrategy : public AbstractLandmarkPlacingStrategy<G, V> {
	public:
		CanonicalAdvancePlacingLandmarkStrategy(int landmarkToCreate) : AbstractLandmarkPlacingStrategy{landmarkToCreate}, unmarkedNodes{} {

		}
		virtual ~CanonicalAdvancePlacingLandmarkStrategy() {

		}
	public:
		virtual std::vector<nodeid_t> getLandmarks(const IImmutableGraph<G, V, cost_t>& graph, MapPlus<nodeid_t, std::vector<uint32_t>>& getDistancesFromLandmarksToNodes) {
			debug("computing landmarks of graph");

			std::vector<nodeid_t> result;

			//set all nodes as unmarked
			for (nodeid_t n=0; n<graph.node_count(); ++n) {
				this->unmarkedNodes.push_back(n);
			}

			int landmarkCreated = 0;
			do {
				if (this->unmarkedNodes.size() == 0) {
					warning("there are left no states!");
					throw ImpossibleException{"no states left for creating %ld landmarks!", this->landmarkToCreate};
				}

				//pick a start vertex
				nodeid_t startNode = this->getRandomUnMarkedNode(graph);
				//do the BFS
				(graph, startNode, graph.node_count()/this->landmarkToCreate);
				//the new landmark is the seed of the BFS
				landmarkCreated += 1;
				result.push_back(startNode);
			} while ((landmarkCreated + 0.0)/(0.0 + this->landmarkToCreate) <= 0.9); //we're within the 10% of the intended landmarkToCreate

			getDistancesFromLandmarksToNodes = std::unordered_map<nodeid_t, std::vector<distance_t>>{};

			return result;
		}
	public:
		void cleanup() {
			this->unmarkedNodes.clear();
		}
	private:
		bool isNodeUnMarked(nodeid_t n) const {
			return std::find(this->unmarkedNodes.begin(), this->unmarkedNodes.end(), n) != this->unmarkedNodes.end();
		}

		nodeid_t getRandomUnMarkedNode(const IImmutableGraph<G, V, cost_t>& g) const {
			default_random_engine eng;
			std::uniform_int_distribution<nodeid_t> startVertexDistribution(0, this->unmarkedNodes.size()-1);

			return startVertexDistribution(eng);
		}

		void markNode(nodeid_t n) {
			assert(this->isNodeUnMarked(n));

			auto it = std::find(this->unmarkedNodes.begin(), this->unmarkedNodes.end(), n);
			assert(it !=this->unmarkedNodes.end());

			this->unmarkedNodes.erase(it);
		}
		void _markNodesInBreadthFirstSearch(const IImmutableGraph<G, V, cost_t>& g, nodeid_t n, safe_int statesToMark, std::vector<nodeid_t>& bfsQueue) {
			if (statesToMark == 0) {
				return;
			}

			this->markNode(n);

			for (int i=0; i<g.out_deg(n); ++i) {
				bfsQueue.push_back(static_cast<nodeid_t>(g.getIthOutArc(n, i).target));
			}

			//pop first element of the vector
			nodeid_t head = bfsQueue.front();
			bfsQueue.erase(bfsQueue.begin());

			//recursive call
			this->_markNodesInBreadthFirstSearch(g, head, statesToMark - 1, bfsQueue);
		}

		void markNodesInBreadthFirstSearch(const IImmutableGraph<G, V, cost_t>& g, nodeid_t start, safe_int statesToMark) {
			std::vector<nodeid_t> bfsQueue;
			this->_markNodesInBreadthFirstSearch(g, start, statesToMark, bfsQueue);
		}
	private:
		std::vector<nodeid_t> unmarkedNodes;
	};

}

#endif