#include "catch.hpp"

#include <cpp-utils/adjacentGraph.hpp>

#include "OctileHeuristic.hpp"
#include "ALTHeuristic.hpp"

#include "IPathFindingMapReader.hpp"
#include "GridMap.hpp"
#include "GridMapGraphConverter.hpp"
#include "MovingAIGridMapReader.hpp"
#include "AStar.hpp"
#include "DijkstraSearchAlgorithm.hpp"
#include "DifferentHeuristicAdvancePlacingLandmarkStrategy.hpp"

using namespace pathfinding;
using namespace pathfinding::search;
using namespace cpp_utils::graphs;
using namespace cpp_utils;

SCENARIO("test search algorithms") {

	maps::MovingAIGridMapReader reader{
		'.', cost_t{100},
		'T', cost_t::INFTY,
		'@', cost_t::INFTY
	};
	maps::GridMap map{reader.load(boost::filesystem::path{"./square03.map"})};

	/*
	* MAP
	* 
	*	.....
	*  ...@@
	*  ..@@.
	*  ...@@
	*  .....
	* 
	*/


	maps::GridMapGraphConverter converter{maps::GridBranching::EIGHT_CONNECTED};
	graphs::AdjacentGraph<std::string, xyLoc, cost_t> graph{*converter.toGraph(map)};
	// auto image = graph.getPPM();
	// image->savePNG("square03-image");
	// delete image;

	GIVEN("testing A*") {
		//h computer
		search::OctileHeuristic heuristic{maps::GridBranching::EIGHT_CONNECTED};
		//goal checker
		search::SAPFGridMapGoalChecker goalChecker{};
		//state generator
		search::GridMapStateSupplier<std::string> supplier{graph};
		//successor generator
		search::SimpleGridMapStateExpander<std::string> expander{graph};
		//pruner
		search::PruneIfExpanded<search::GridMapState> pruner{};
		//meta heuristic search
		search::NoCloseListSingleGoalAstar<search::GridMapState, graphs::nodeid_t> searchAlgorithm{
			heuristic, 
			goalChecker, 
			supplier, 
			expander, 
			pruner
		};

		WHEN("start is the same of goal") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{0,0};

			search::GridMapState& start = supplier.getState(graph.idOfVertex(startLoc));
			search::GridMapState& goal = supplier.getState(graph.idOfVertex(goalLoc));
			auto solution{searchAlgorithm.search(start, goal, false)};
			REQUIRE(solution->map<xyLoc>([&](const search::GridMapState* x) {return x->getFirstData();}) == vectorplus<xyLoc>::make(xyLoc{0,0}));
			REQUIRE(solution->getCost() == 0);
		}

		WHEN("goal is just below start") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{0,1};
			search::GridMapState& start = supplier.getState(graph.idOfVertex(startLoc));
			search::GridMapState& goal = supplier.getState(graph.idOfVertex(goalLoc));
			auto solution{searchAlgorithm.search(start, goal, false)};
			REQUIRE(solution->map<xyLoc>([&](const search::GridMapState* x) {return x->getFirstData();}) == vectorplus<xyLoc>::make(xyLoc{0,0}, xyLoc{0,1}));
			REQUIRE(solution->getCost() == 100);
		}

		WHEN("goal is just diagonally reachable") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{1,1};
			search::GridMapState& start = supplier.getState(graph.idOfVertex(startLoc));
			search::GridMapState& goal = supplier.getState(graph.idOfVertex(goalLoc));
			auto solution{searchAlgorithm.search(start, goal, false)};
			REQUIRE(solution->map<xyLoc>([&](const search::GridMapState* x) {return x->getFirstData();}) == vectorplus<xyLoc>::make(xyLoc{0,0}, xyLoc{1,1}));
			REQUIRE(solution->getCost() == 141);
		}

		WHEN("goal is far but reachable") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{4,4};
			search::GridMapState& start = supplier.getState(graph.idOfVertex(startLoc));
			search::GridMapState& goal = supplier.getState(graph.idOfVertex(goalLoc));
			auto solution{searchAlgorithm.search(start, goal, false)};
			REQUIRE(solution->map<xyLoc>([&](const search::GridMapState* x) {return x->getFirstData();}) == vectorplus<xyLoc>::make(xyLoc{0,0}, xyLoc{0,1}, xyLoc{0,2}, xyLoc{1,3}, xyLoc{2,4}, xyLoc{3,4}, xyLoc{4,4}));
			REQUIRE(solution->getCost() == (4*100 + 2*141));
		}

		WHEN("goal is un reachable") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{4,2};
			search::GridMapState& start = supplier.getState(graph.idOfVertex(startLoc));
			search::GridMapState& goal = supplier.getState(graph.idOfVertex(goalLoc));
			REQUIRE_THROWS(searchAlgorithm.search(start, goal, false));
		}
	}

	GIVEN("testing dijkstra") {

		search::DijkstraSearchAlgorithm<std::string, xyLoc> searchAlgorithm{graph};
		
		WHEN("start is the same of goal") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{0,0};

			cpp_utils::graphs::nodeid_t start = graph.idOfVertex(startLoc);
			cpp_utils::graphs::nodeid_t goal = graph.idOfVertex(goalLoc);
			info("testing dijkstra", start, goal);
			auto solution{searchAlgorithm.search(start, goal)};
			REQUIRE(solution->map<xyLoc>([&,graph](const cpp_utils::graphs::nodeid_t x) {return graph.getVertex(x);}) == vectorplus<xyLoc>::make(xyLoc{0,0}));
			REQUIRE(solution->getCost() == 0);
		}

		WHEN("goal is just below start") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{0,1};
			cpp_utils::graphs::nodeid_t start = graph.idOfVertex(startLoc);
			cpp_utils::graphs::nodeid_t goal = graph.idOfVertex(goalLoc);
			auto solution{searchAlgorithm.search(start, goal)};
			REQUIRE(solution->map<xyLoc>([&,graph](const cpp_utils::graphs::nodeid_t x) {return graph.getVertex(x);}) == vectorplus<xyLoc>::make(xyLoc{0,0}, xyLoc{0,1}));
			REQUIRE(solution->getCost() == 100);
		}

		WHEN("goal is just diagonally reachable") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{1,1};
			cpp_utils::graphs::nodeid_t start = graph.idOfVertex(startLoc);
			cpp_utils::graphs::nodeid_t goal = graph.idOfVertex(goalLoc);
			auto solution{searchAlgorithm.search(start, goal)};
			REQUIRE(solution->map<xyLoc>([&,graph](const cpp_utils::graphs::nodeid_t x) {return graph.getVertex(x);}) == vectorplus<xyLoc>::make(xyLoc{0,0}, xyLoc{1,1}));
			REQUIRE(solution->getCost() == 141);
		}

		WHEN("goal is far but reachable") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{4,4};
			cpp_utils::graphs::nodeid_t start = graph.idOfVertex(startLoc);
			cpp_utils::graphs::nodeid_t goal = graph.idOfVertex(goalLoc);
			auto solution{searchAlgorithm.search(start, goal)};
			REQUIRE(solution->map<xyLoc>([&,graph](const cpp_utils::graphs::nodeid_t x) {return graph.getVertex(x);}) == vectorplus<xyLoc>::make(xyLoc{0,0}, xyLoc{0,1}, xyLoc{0,2}, xyLoc{1,3}, xyLoc{2,4}, xyLoc{3,4}, xyLoc{4,4}));
			REQUIRE(solution->getCost() == (4*100 + 2*141));
		}

		WHEN("goal is un reachable") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{4,2};
			cpp_utils::graphs::nodeid_t start = graph.idOfVertex(startLoc);
			cpp_utils::graphs::nodeid_t goal = graph.idOfVertex(goalLoc);
			REQUIRE_THROWS(searchAlgorithm.search(start, goal));
		}
	}

}

SCENARIO("test ALT") {

	//ALT implementation does not support disconnected maps
	maps::MovingAIGridMapReader reader{
		'.', cost_t{100},
		'T', cost_t::INFTY,
		'@', cost_t::INFTY
	};
	maps::GridMap map{reader.load(boost::filesystem::path{"./square03-allconnected.map"})};

	/*
	* MAP
	* 
	*	.....
	*  ...@@
	*  ..@@.
	*  ...@.
	*  .....
	* 
	*/


	maps::GridMapGraphConverter converter{maps::GridBranching::EIGHT_CONNECTED};
	graphs::AdjacentGraph<std::string, xyLoc, cost_t> graph{*converter.toGraph(map)};
	// auto image = graph.getPPM();
	// image->savePNG("square03-image");
	// delete image;

	GIVEN("testing ALT") {
		//strategy
		auto strategy = DifferentHeuristicAdvancePlacingLandmarkStrategy<std::string, xyLoc>{4};
		//landmark database path
		auto landmarkDatabase = LandmarkDatabase<std::string, xyLoc>::fetchOrCompute(
			graph, 
			strategy,
			boost::filesystem::path{"./database.landmarkdb"}
		);


		//h computer
		search::ALTHeuristic<search::GridMapState, std::string, xyLoc> heuristic{graph, landmarkDatabase};
		//goal checker
		search::SAPFGridMapGoalChecker goalChecker{};
		//state generator
		search::GridMapStateSupplier<std::string> supplier{graph};
		//successor generator
		search::SimpleGridMapStateExpander<std::string> expander{graph};
		//pruner
		search::PruneIfExpanded<search::GridMapState> pruner{};
		//meta heuristic search
		search::NoCloseListSingleGoalAstar<search::GridMapState, graphs::nodeid_t> searchAlgorithm{
			heuristic, 
			goalChecker, 
			supplier, 
			expander, 
			pruner
		};

		WHEN("start is the same of goal") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{0,0};

			search::GridMapState& start = supplier.getState(graph.idOfVertex(startLoc));
			search::GridMapState& goal = supplier.getState(graph.idOfVertex(goalLoc));
			auto solution{searchAlgorithm.search(start, goal, false)};
			REQUIRE(solution->map<xyLoc>([&](const search::GridMapState* x) {return x->getFirstData();}) == vectorplus<xyLoc>::make(xyLoc{0,0}));
			REQUIRE(solution->getCost() == 0);
		}

		WHEN("goal is just below start") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{0,1};
			search::GridMapState& start = supplier.getState(graph.idOfVertex(startLoc));
			search::GridMapState& goal = supplier.getState(graph.idOfVertex(goalLoc));
			auto solution{searchAlgorithm.search(start, goal, false)};
			REQUIRE(solution->map<xyLoc>([&](const search::GridMapState* x) {return x->getFirstData();}) == vectorplus<xyLoc>::make(xyLoc{0,0}, xyLoc{0,1}));
			REQUIRE(solution->getCost() == 100);
		}

		WHEN("goal is just diagonally reachable") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{1,1};
			search::GridMapState& start = supplier.getState(graph.idOfVertex(startLoc));
			search::GridMapState& goal = supplier.getState(graph.idOfVertex(goalLoc));
			auto solution{searchAlgorithm.search(start, goal, false)};
			REQUIRE(solution->map<xyLoc>([&](const search::GridMapState* x) {return x->getFirstData();}) == vectorplus<xyLoc>::make(xyLoc{0,0}, xyLoc{1,1}));
			REQUIRE(solution->getCost() == 141);
		}

		WHEN("goal is far but reachable") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{4,4};
			search::GridMapState& start = supplier.getState(graph.idOfVertex(startLoc));
			search::GridMapState& goal = supplier.getState(graph.idOfVertex(goalLoc));
			auto solution{searchAlgorithm.search(start, goal, false)};
			REQUIRE(solution->getCost() == (4*100 + 2*141));
			REQUIRE((
				(solution->map<xyLoc>([&](const search::GridMapState* x) {return x->getFirstData();}) == vectorplus<xyLoc>::make(xyLoc{0,0}, xyLoc{0,1}, xyLoc{0,2}, xyLoc{1,3}, xyLoc{2,4}, xyLoc{3,4}, xyLoc{4,4}))
				||
				(solution->map<xyLoc>([&](const search::GridMapState* x) {return x->getFirstData();}) == vectorplus<xyLoc>::make(xyLoc{0,0}, xyLoc{1,1}, xyLoc{1,2}, xyLoc{1,3}, xyLoc{2,4}, xyLoc{3,4}, xyLoc{4,4}))
			));
		}
	}
}