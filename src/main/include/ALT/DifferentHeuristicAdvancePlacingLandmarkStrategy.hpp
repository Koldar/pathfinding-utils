#ifndef _PATHFINDING_UTILS_DIFFERENT_HEURISTIC_ADVANCE_PLACING_LANDMARK_STRATEGY_HEADER__
#define _PATHFINDING_UTILS_DIFFERENT_HEURISTIC_ADVANCE_PLACING_LANDMARK_STRATEGY_HEADER__

#include <random>

#include "DijkstraAlgorithm.hpp"
#include "AbstractLandmarkPlacingStrategy.hpp"

namespace pathfinding::search {

    template <typename G, typename V>
	class DifferentHeuristicAdvancePlacingLandmarkStrategy : public AbstractLandmarkPlacingStrategy<G, V> {
		using Super = AbstractLandmarkPlacingStrategy<G,V>;
		using This = DifferentHeuristicAdvancePlacingLandmarkStrategy<G,V>;
	public:
		DifferentHeuristicAdvancePlacingLandmarkStrategy(int landmarkToCreate): Super{landmarkToCreate} {

		}
		virtual ~DifferentHeuristicAdvancePlacingLandmarkStrategy() {

		}
		DifferentHeuristicAdvancePlacingLandmarkStrategy(const This& o): Super{o} {

		}
		DifferentHeuristicAdvancePlacingLandmarkStrategy(This&& o): Super{o} {
			
		}
		This& operator =(const This& o) {
			Super::operator =(o);
			return *this;
		}
		This& operator =(This&& o) {
			Super::operator =(::std::move(o));
			return *this;
		}
	public:
		virtual std::vector<nodeid_t> getLandmarks(const IImmutableGraph<G, V, cost_t>& graph, std::unordered_map<nodeid_t, std::vector<distance_t>>& getDistancesFromLandmarksToNodes) {
			debug("generating landmarks via DifferentHeuristicAdvancePlacingLandmarkStrategy!");
			std::vector<nodeid_t> result;

			//get random state
			nodeid_t randomState = this->getRandomNode(graph);
			//get first canonical state by getting the farthest node from randomState
			std::vector<cost_t> distances = this->getDistancesFrom(graph, randomState);
			nodeid_t newLandmark;
			cost_t maxDistance;
			/**
			 * key: landmarks
			 * value: vector representing the optimal distance from the landmark till the i-th node
			 */
			std::unordered_map<nodeid_t, std::vector<cost_t>> distancesFromLandmarks;

			debug("picked random node", randomState);
			newLandmark = this->breakFarthestNodesTie(this->getFarthestNodes(distances, randomState, maxDistance));
			distancesFromLandmarks[newLandmark] = this->getDistancesFrom(graph, newLandmark);
			result.push_back(newLandmark);

			//get the nodes which maximize the minimum distance from landmarks
			for (int landmark_id=1; landmark_id<this->landmarkToCreate; ++landmark_id) {
				debug("finding landmark #", landmark_id, "/", this->landmarkToCreate);
				debug("the current landmarks are", result);

				cost_t maximum = 0;
				std::vector<nodeid_t> possibleLandmarks;
				for (nodeid_t source=0; source<graph.numberOfVertices(); ++source) {
					debug("********** considering source", source, "... what is the minimum?");
					cost_t value = this->getMinimumDistanceFromLandmarks(graph, source, distancesFromLandmarks);
					//TODO support disconnected maps (value might be infty)
					debug("the value is ", value);
					if (value.isInfinity()) {
						//the node we have chosen is disconnected to every other landmark. Ignoe
						continue;
					}
					if (value > maximum) {
						debug("new maximum! source=", source, "value", value);
						maximum = value;
						possibleLandmarks.clear();
						possibleLandmarks.push_back(source);
					} else if (value == maximum) {
						possibleLandmarks.push_back(source);
					}
				}

				newLandmark = this->breakFarthestNodesTie(possibleLandmarks);
				distancesFromLandmarks[newLandmark] = this->getDistancesFrom(graph, newLandmark);
				debug("the new landmark is", newLandmark);

				result.push_back(newLandmark);

			}

			//update the input-output parameter
			getDistancesFromLandmarksToNodes = std::unordered_map<nodeid_t, std::vector<distance_t>>{};
			for (auto it=distancesFromLandmarks.begin(); it!= distancesFromLandmarks.end(); ++it) {
				getDistancesFromLandmarksToNodes[it->first] = this->convertCostTInto4Bytes(it->second);
			}
			return result;
		}
	public:
		virtual void cleanup() {

		}
	private:
		std::vector<uint32_t> convertCostTInto4Bytes(const std::vector<cost_t>& costVector) const {
			std::vector<uint32_t> result{};
			for (auto i=0; i<costVector.size(); ++i) {
				if (costVector[i].isInfinity()) {
					//we need to save infinity. Let's save UINT32_MAX and hope for the best!
					//TODO find a better method: we should be able to save in the landmark database infinity itself!
					result.push_back(std::numeric_limits<uint32_t>::max());
				} else if (costVector[i] > static_cast<cost_t>(UINT32_MAX)) {
					log_error("trying to compress ", costVector[i]);
					throw cpp_utils::exceptions::ImpossibleException{"trying to compress a number into a 32bit which will generate overflow!"};
				} else {
					result.push_back(costVector.at(i).toUInt32());
				}
				
			}
			return result;
		}
		nodeid_t getRandomNode(const IImmutableGraph<G, V, cost_t>& g) const {
			std::default_random_engine eng;
			std::uniform_int_distribution<nodeid_t> startVertexDistribution(0, g.numberOfVertices()-1);

			return startVertexDistribution(eng);
		}
		std::vector<cost_t> getDistancesFrom(const IImmutableGraph<G, V, cost_t>& g, nodeid_t n) const {
			pathfinding::algorithms::DijkstraAlgorithm<G,V> dijkstra{g};
			dijkstra.run(n);
			return dijkstra.getDistances();
		}
		std::vector<nodeid_t> getFarthestNodes(const std::vector<cost_t>& distances, nodeid_t n, cost_t& maxDistance) const {
			std::vector<nodeid_t> result;

			maxDistance = 0;
			for (nodeid_t i=0; i<distances.size(); ++i) {
				//distances can be infinity. Need to deal with it
				if (distances[i].isInfinity()) {
					continue;
				}
				if (distances[i] > maxDistance) {
					maxDistance = distances[i];
					result.clear();
					result.push_back(i);
				} else if (distances[i] == maxDistance) {
					result.push_back(i);
				}
			}

			return result;
		}
		nodeid_t breakFarthestNodesTie(const std::vector<nodeid_t>& farthestNodes) const {
			return farthestNodes[0];
		}
		cost_t getMinimumDistanceFromLandmarks(const IImmutableGraph<G,V, cost_t>& g, nodeid_t n, const std::unordered_map<nodeid_t, std::vector<cost_t>>& distancesFromLandmarks) const {
			cost_t result = cost_t::INFTY;

			for (auto it=distancesFromLandmarks.begin(); it!=distancesFromLandmarks.end(); ++it) {
				nodeid_t landmark = it->first;

				cost_t distanceFromLandmark = it->second[n];
				debug("distance is ", distanceFromLandmark);
				//values can be infinities
				if (distanceFromLandmark.lessThan(result, false)) {
					debug("new minimum! source=", n, "value=", distanceFromLandmark);
					result = distanceFromLandmark;
				}
			}

			return result;
		}
	};

}

#endif