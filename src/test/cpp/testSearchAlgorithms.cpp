#include "catch.hpp"

#include <cpp-utils/adjacentGraph.hpp>

#include "OctileHeuristic.hpp"
#include "ALTHeuristic.hpp"

#include "map_base_reason_e.hpp"

#include "IPathFindingMapReader.hpp"
#include "GridMap.hpp"
#include "GridMapGraphConverter.hpp"
#include "MovingAIGridMapReader.hpp"
#include "AStar.hpp"
#include "DijkstraSearchAlgorithm.hpp"
#include "DifferentHeuristicAdvancePlacingLandmarkStrategy.hpp"
#include "StandardLocationGoalChecker.hpp"
#include "StandardStateExpander.hpp"

using namespace pathfinding;
using namespace pathfinding::search;
using namespace cpp_utils::graphs;
using namespace cpp_utils;

SCENARIO("test search algorithms") {

	maps::MovingAIGridMapReader reader{
		'.', cost_t{100}, color_t::WHITE,
		'T', cost_t::INFTY, color_t::BLACK,
		'@', cost_t::INFTY, color_t::BLACK
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
		search::OctileHeuristic<search::GridMapState<map_base_reason_e>> heuristic{maps::GridBranching::EIGHT_CONNECTED};
		//goal checker
		search::StandardLocationGoalChecker<search::GridMapState<map_base_reason_e>> goalChecker{};
		//state generator
		search::GridMapStateSupplier<std::string, map_base_reason_e> supplier{graph};
		//successor generator
		cpp_utils::function_t<cost_t, cost_t> costFunction = [&](const cost_t& c) { return c;};
		search::StandardStateExpander<search::GridMapState<map_base_reason_e>, std::string, xyLoc, cost_t, map_base_reason_e> expander{graph, costFunction};
		//pruner
		search::PruneIfExpanded<search::GridMapState<map_base_reason_e>> pruner{};
		//meta heuristic search
		search::NoCloseListSingleGoalAstar<search::GridMapState<map_base_reason_e>, graphs::nodeid_t, map_base_reason_e> searchAlgorithm{
			heuristic, 
			goalChecker, 
			supplier, 
			expander, 
			pruner
		};

		WHEN("start is the same of goal") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{0,0};

			auto start = supplier.getState(graph.idOfVertex(startLoc), map_base_reason_e::INPUT);
			auto goal = supplier.getState(graph.idOfVertex(goalLoc), map_base_reason_e::INPUT);
			auto solution{searchAlgorithm.search(start, goal, false)};
			REQUIRE(solution->map<xyLoc>([&](auto x) {return x.getFirstData();}) == vectorplus<xyLoc>::make(xyLoc{0,0}));
			REQUIRE(solution->getCost() == 0);
		}

		WHEN("goal is just below start") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{0,1};
			auto start = supplier.getState(graph.idOfVertex(startLoc), map_base_reason_e::INPUT);
			auto goal = supplier.getState(graph.idOfVertex(goalLoc), map_base_reason_e::INPUT);
			auto solution{searchAlgorithm.search(start, goal, false)};
			REQUIRE(solution->map<xyLoc>([&](auto x) {return x.getFirstData();}) == vectorplus<xyLoc>::make(xyLoc{0,0}, xyLoc{0,1}));
			REQUIRE(solution->getCost() == 100);
		}

		WHEN("goal is just diagonally reachable") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{1,1};
			auto start = supplier.getState(graph.idOfVertex(startLoc), map_base_reason_e::INPUT);
			auto goal = supplier.getState(graph.idOfVertex(goalLoc), map_base_reason_e::INPUT);
			auto solution{searchAlgorithm.search(start, goal, false)};
			REQUIRE(solution->map<xyLoc>([&](auto x) {return x.getFirstData();}) == vectorplus<xyLoc>::make(xyLoc{0,0}, xyLoc{1,1}));
			REQUIRE(solution->getCost() == 141);
		}

		WHEN("goal is far but reachable") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{4,4};
			auto start = supplier.getState(graph.idOfVertex(startLoc), map_base_reason_e::INPUT);
			auto goal = supplier.getState(graph.idOfVertex(goalLoc), map_base_reason_e::INPUT);
			auto solution{searchAlgorithm.search(start, goal, false)};
			REQUIRE(solution->map<xyLoc>([&](auto x) {return x.getFirstData();}) == vectorplus<xyLoc>::make(xyLoc{0,0}, xyLoc{0,1}, xyLoc{0,2}, xyLoc{1,3}, xyLoc{2,4}, xyLoc{3,4}, xyLoc{4,4}));
			REQUIRE(solution->getCost() == (4*100 + 2*141));
		}

		WHEN("goal is un reachable") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{4,2};
			auto start = supplier.getState(graph.idOfVertex(startLoc), map_base_reason_e::INPUT);
			auto goal = supplier.getState(graph.idOfVertex(goalLoc), map_base_reason_e::INPUT);
			REQUIRE_THROWS(searchAlgorithm.search(start, goal, false));
		}
	}

	GIVEN("testing dijkstra") {

		search::DijkstraSearchAlgorithm<std::string, xyLoc, cost_t> searchAlgorithm{graph, GetCost<cost_t>{}};
		
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

	maps::MovingAIGridMapReader reader{
		'.', cost_t{100}, color_t::WHITE,
		'T', cost_t::INFTY, color_t::BLACK,
		'@', cost_t::INFTY, color_t::BLACK
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
		search::ALTHeuristic<search::GridMapState<map_base_reason_e>, std::string, xyLoc> heuristic{graph, landmarkDatabase};
		//goal checker
		search::StandardLocationGoalChecker<search::GridMapState<map_base_reason_e>> goalChecker{};
		//state generator
		search::GridMapStateSupplier<std::string, map_base_reason_e> supplier{graph};
		//successor generator
		cpp_utils::function_t<cost_t, cost_t> costFunction = [&](const cost_t& c) { return c;};
		search::StandardStateExpander<search::GridMapState<map_base_reason_e>, std::string, xyLoc, cost_t, map_base_reason_e> expander{graph, costFunction};
		//pruner
		search::PruneIfExpanded<search::GridMapState<map_base_reason_e>> pruner{};
		//meta heuristic search
		search::NoCloseListSingleGoalAstar<search::GridMapState<map_base_reason_e>, graphs::nodeid_t, map_base_reason_e> searchAlgorithm{
			heuristic, 
			goalChecker, 
			supplier, 
			expander, 
			pruner
		};

		WHEN("start is the same of goal") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{0,0};

			auto start = supplier.getState(graph.idOfVertex(startLoc));
			auto goal = supplier.getState(graph.idOfVertex(goalLoc));
			auto solution{searchAlgorithm.search(start, goal, false)};
			REQUIRE(solution->map<xyLoc>([&](auto x) {return x.getFirstData();}) == vectorplus<xyLoc>::make(xyLoc{0,0}));
			REQUIRE(solution->getCost() == 0);
		}

		WHEN("goal is just below start") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{0,1};
			auto start = supplier.getState(graph.idOfVertex(startLoc));
			auto goal = supplier.getState(graph.idOfVertex(goalLoc));
			auto solution{searchAlgorithm.search(start, goal, false)};
			REQUIRE(solution->map<xyLoc>([&](auto x) {return x.getFirstData();}) == vectorplus<xyLoc>::make(xyLoc{0,0}, xyLoc{0,1}));
			REQUIRE(solution->getCost() == 100);
		}

		WHEN("goal is just diagonally reachable") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{1,1};
			auto start = supplier.getState(graph.idOfVertex(startLoc));
			auto goal = supplier.getState(graph.idOfVertex(goalLoc));
			auto solution{searchAlgorithm.search(start, goal, false)};
			REQUIRE(solution->map<xyLoc>([&](auto x) {return x.getFirstData();}) == vectorplus<xyLoc>::make(xyLoc{0,0}, xyLoc{1,1}));
			REQUIRE(solution->getCost() == 141);
		}

		WHEN("goal is far but reachable") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{4,4};
			auto start = supplier.getState(graph.idOfVertex(startLoc));
			auto goal = supplier.getState(graph.idOfVertex(goalLoc));
			auto solution{searchAlgorithm.search(start, goal, false)};
			REQUIRE(solution->getCost() == (4*100 + 2*141));
			REQUIRE((
				(solution->map<xyLoc>([&](auto x) {return x.getFirstData();}) == vectorplus<xyLoc>::make(xyLoc{0,0}, xyLoc{0,1}, xyLoc{0,2}, xyLoc{1,3}, xyLoc{2,4}, xyLoc{3,4}, xyLoc{4,4}))
				||
				(solution->map<xyLoc>([&](auto x) {return x.getFirstData();}) == vectorplus<xyLoc>::make(xyLoc{0,0}, xyLoc{1,1}, xyLoc{1,2}, xyLoc{1,3}, xyLoc{2,4}, xyLoc{3,4}, xyLoc{4,4}))
			));
		}

		WHEN("goal is un reachable") {
			xyLoc startLoc{0,0};
			xyLoc goalLoc{4,2};
			auto start = supplier.getState(graph.idOfVertex(startLoc));
			auto goal = supplier.getState(graph.idOfVertex(goalLoc));
			REQUIRE_THROWS(searchAlgorithm.search(start, goal, false));
		}
	}
}