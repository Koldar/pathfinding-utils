#ifndef _PATHFINDING_UTILS_ABSTRACT_LANDMARK_PLACING_STRATEGY_HEADER__
#define _PATHFINDING_UTILS_ABSTRACT_LANDMARK_PLACING_STRATEGY_HEADER__

#include <unordered_map>

#include <cpp-utils/igraph.hpp>
#include <cpp-utils/ICleanable.hpp>

namespace pathfinding::search {

    /**
	 * Landmark explicitly model distances as a 32bit integer
	 * 
	 * This is essential in the preprocessing space
	 * 
	 */
	using distance_t = uint32_t;

    using namespace cpp_utils::graphs;

	/**
	 * @brief interface you need to extends in order to have a strategy to use to fetch landmarks in ALT algorithms
	 * 
	 */
	template <typename G, typename V>
	class AbstractLandmarkPlacingStrategy: public ICleanable {
		using This = AbstractLandmarkPlacingStrategy<G,V>;
	public:
		AbstractLandmarkPlacingStrategy(int landmarkToCreate): landmarkToCreate{landmarkToCreate} {

		}
		virtual ~AbstractLandmarkPlacingStrategy() {

		}
		AbstractLandmarkPlacingStrategy(const This& o): landmarkToCreate{o.landmarkToCreate} {

		}
		AbstractLandmarkPlacingStrategy(This&& o): landmarkToCreate{o.landmarkToCreate} {

		}
		This& operator =(const This& o) {
			this->landmarkToCreate = o.landmarkToCreate;
			return *this;
		}
		This& operator =(This&& o) {
			this->landmarkToCreate = o.landmarkToCreate;
			return *this;
		}
	public:
		virtual std::vector<nodeid_t> getLandmarks(const IImmutableGraph<G, V, cost_t>& graph, std::unordered_map<nodeid_t, std::vector<distance_t>>& getDistancesFromLandmarksToNodes) = 0;
	protected:
		/**
		 * @brief the landmark we need to create
		 * 
		 */
		const int landmarkToCreate;
	};

}


#endif