#include "catch.hpp"

#include "xyLoc.hpp"
#include "OctileHeuristic.hpp"
#include "ManhattanHeuristic.hpp"
#include "GridMapState.hpp"
#include "GridBranching.hpp"
#include "types.hpp"
#include <boost/filesystem.hpp>
#include "IPathFindingMapReader.hpp"
#include "GridMap.hpp"
#include "MovingAIGridMapReader.hpp"
#include "AStar.hpp"
#include "GridMapGraphConverter.hpp"
#include <cpp-utils/adjacentGraph.hpp>
#include "DijkstraSearchAlgorithm.hpp"
#include "StandardLocationGoalChecker.hpp"
#include "StandardStateExpander.hpp"
#include "GraphState.hpp"
#include <functional>

using namespace pathfinding;
using namespace cpp_utils;

struct OtherCost {
	cost_t cost;
	bool isDirty;

	static cost_t getCost(const OtherCost& c) {
		return c.cost;
	}

	friend bool operator ==(const OtherCost& a, const OtherCost& b) {
		return a.cost == b.cost && a.isDirty == b.isDirty;
	}
	friend std::ostream& operator <<(std::ostream& ss, const OtherCost& a) {
		ss << "{" << a.cost  <<", " << a.isDirty << "}";
		return ss;
	}
};




SCENARIO("test GraphState") {

	maps::MovingAIGridMapReader reader{
		'.', cost_t{1000},
		'T', cost_t{1500},
		'@', cost_t::INFTY
	};

	GIVEN("traversable map") {

		maps::GridMap map{reader.load(boost::filesystem::path{"./square03.map"})};
		maps::GridMapGraphConverter converter{maps::GridBranching::EIGHT_CONNECTED};
		graphs::AdjacentGraph<std::string, xyLoc, cost_t> graph{*converter.toGraph(map)};
		graphs::ListGraph<std::string, xyLoc, OtherCost> graph2{"hello"};

		WHEN("GrapState with cost_t") {
			search::GraphState<std::string, xyLoc, cost_t> state{0, graph, 0};
			search::GraphStateSupplier<std::string, xyLoc, cost_t> supplier{graph}; 
			search::StandardStateExpander<search::GraphState<std::string, xyLoc, cost_t>, std::string, xyLoc, cost_t> expander{graph};
			search::GraphLocationGoalChecker<search::GraphState<std::string, xyLoc, cost_t>> goalChecker{};
			REQUIRE(state.getId() == 0);
		}

		WHEN("GrapState with default") {
			search::GraphState<std::string, xyLoc> state{0, graph, 0};
			search::GraphStateSupplier<std::string, xyLoc, cost_t> supplier{graph}; 
			search::StandardStateExpander<search::GraphState<std::string, xyLoc>, std::string, xyLoc, cost_t> expander{graph};
			search::GraphLocationGoalChecker<search::GraphState<std::string, xyLoc>> goalChecker{};
			REQUIRE(state.getId() == 0);
		}

		WHEN("GrapState with other") {
			search::GraphState<std::string, xyLoc, OtherCost> state{0, graph2, 0};
			search::GraphStateSupplier<std::string, xyLoc, OtherCost> supplier{graph2}; 
			search::StandardStateExpander<search::GraphState<std::string, xyLoc>, std::string, xyLoc, OtherCost, OtherCost::getCost> expander{graph2};
			search::GraphLocationGoalChecker<search::GraphState<std::string, xyLoc, OtherCost>> goalChecker{};
			REQUIRE(state.getId() == 0);
		}
		
	}

}

SCENARIO("test moving ai gridmap loader") {

	maps::MovingAIGridMapReader reader{
		'.', cost_t{1000},
		'T', cost_t{1500},
		'@', cost_t::INFTY
	};

	GIVEN("traversable map") {

		maps::GridMap map{reader.load(boost::filesystem::path{"./combat.map"})};

		REQUIRE(map.getWidth() == 177);
		REQUIRE(map.getHeight() == 193);
		REQUIRE(map.getCellCost({0,0}) == 1000);
		REQUIRE(map.getCellCost({12,15}) == 1500);
		REQUIRE(map.getCellCost({12,14}) == 1000);
	}

	GIVEN("map with some untraversabnle cells") {
		maps::GridMap map{reader.load(boost::filesystem::path{"./den000d.map"})};

		REQUIRE(map.getWidth() == 503);
		REQUIRE(map.getHeight() == 351);
		REQUIRE(map.getCellCost({0,0}) == cost_t::INFTY);
	}
}

SCENARIO("test xyLoc") {

    GIVEN("testing locations") {

		WHEN("location has same x") {

			REQUIRE(xyLoc{5,6}.isAdjacentTo(xyLoc{5,5}));
			REQUIRE(xyLoc{5,6}.isAdjacentTo(xyLoc{5,5}));

			REQUIRE(!xyLoc{5,6}.isAdjacentTo(xyLoc{5,8}));
			REQUIRE(!xyLoc{5,8}.isAdjacentTo(xyLoc{5,6}));
		}

		WHEN("location has same y") {
			REQUIRE(xyLoc{5,6}.isAdjacentTo(xyLoc{4,6}));
			REQUIRE(xyLoc{4,6}.isAdjacentTo(xyLoc{5,6}));

			REQUIRE(!xyLoc{5,6}.isAdjacentTo(xyLoc{3,6}));
			REQUIRE(!xyLoc{3,6}.isAdjacentTo(xyLoc{5,6}));
		}

		WHEN("locations are diagonal") {
			REQUIRE(xyLoc{5,6}.isAdjacentTo(xyLoc{6,7}));
			REQUIRE(xyLoc{6,7}.isAdjacentTo(xyLoc{5,6}));

			REQUIRE(!xyLoc{5,6}.isAdjacentTo(xyLoc{6,8}));
			REQUIRE(!xyLoc{6,8}.isAdjacentTo(xyLoc{5,6}));
		}

		WHEN("nearby locations") {
			critical("output is", xyLoc::getNearbyDiagonalCells(xyLoc{4, 5}, xyLoc{5, 6}));
			REQUIRE(xyLoc::getNearbyDiagonalCells(xyLoc{4, 5}, xyLoc{5, 6}) == std::pair<xyLoc, xyLoc>{xyLoc{4, 6}, xyLoc{5, 5}});
			REQUIRE(xyLoc::getNearbyDiagonalCells(xyLoc{4, 5}, xyLoc{3, 6}) == std::pair<xyLoc, xyLoc>{xyLoc{4, 6}, xyLoc{3, 5}});
			REQUIRE(xyLoc::getNearbyDiagonalCells(xyLoc{4, 5}, xyLoc{3, 4}) == std::pair<xyLoc, xyLoc>{xyLoc{4, 4}, xyLoc{3, 5}});
		}
	}

	GIVEN("testing fetching adjacent cells") {
		xyLoc current{3, 6};

		REQUIRE(current.getAdjacent(Direction::NORTH) == xyLoc{3, 5});
		REQUIRE(current.getAdjacent(Direction::NORTHEAST) == xyLoc{4, 5});
		REQUIRE(current.getAdjacent(Direction::NORTHWEST) == xyLoc{2, 5});
		REQUIRE(current.getAdjacent(Direction::EAST) == xyLoc{4, 6});
		REQUIRE(current.getAdjacent(Direction::WEST) == xyLoc{2, 6});
		REQUIRE(current.getAdjacent(Direction::SOUTH) == xyLoc{3, 7});
		REQUIRE(current.getAdjacent(Direction::SOUTHEAST) == xyLoc{4, 7});
		REQUIRE(current.getAdjacent(Direction::SOUTHWEST) == xyLoc{2, 7});
	}
	
	GIVEN("testing adjacent cells") {

		WHEN("top left corner") {
			xyLoc loc{0,0};
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTH, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTH, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::EAST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::WEST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTHEAST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTHWEST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTHEAST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTHWEST, xyLoc{10,10}) == false);
		}

		WHEN("top right corner") {
			xyLoc loc{10,0};
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTH, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTH, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::EAST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::WEST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTHEAST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTHWEST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTHEAST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTHWEST, xyLoc{10,10}) == true);
		}

		WHEN("top corner") {
			xyLoc loc{5,0};
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTH, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTH, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::EAST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::WEST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTHEAST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTHWEST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTHEAST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTHWEST, xyLoc{10,10}) == true);
		}

		WHEN("on left") {
			xyLoc loc{0,5};
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTH, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTH, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::EAST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::WEST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTHEAST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTHWEST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTHEAST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTHWEST, xyLoc{10,10}) == false);
		}

		WHEN("on right") {
			xyLoc loc{10,5};
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTH, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTH, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::EAST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::WEST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTHEAST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTHWEST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTHEAST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTHWEST, xyLoc{10,10}) == true);
		}

		WHEN("bottom left corner") {
			xyLoc loc{0,10};
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTH, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTH, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::EAST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::WEST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTHEAST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTHWEST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTHEAST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTHWEST, xyLoc{10,10}) == false);
		}

		WHEN("bottom") {
			xyLoc loc{5,10};
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTH, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTH, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::EAST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::WEST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTHEAST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTHWEST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTHEAST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTHWEST, xyLoc{10,10}) == false);
		}

		WHEN("bottom right corner") {
			xyLoc loc{10,10};
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTH, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTH, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::EAST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::WEST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTHEAST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTHWEST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTHEAST, xyLoc{10,10}) == false);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTHWEST, xyLoc{10,10}) == false);
		}

		WHEN("center") {
			xyLoc loc{3,5};
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTH, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTH, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::EAST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::WEST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTHEAST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::NORTHWEST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTHEAST, xyLoc{10,10}) == true);
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTHWEST, xyLoc{10,10}) == true);
		}

		WHEN("other cases") {
			xyLoc loc{4,4};
			REQUIRE(loc.isThereLocationInDirectionOf(Direction::SOUTH, xyLoc{4,4}) == false);
		}
	}

}


SCENARIO("test octile") {

    search::OctileHeuristic h{maps::GridBranching::EIGHT_CONNECTED};

	REQUIRE(h.getHeuristic(search::GridMapState{0, 0, xyLoc{0,0}}, search::GridMapState{0, 0, xyLoc{0, 1}}) == 1L);
	REQUIRE(h.getHeuristic(search::GridMapState{0, 0, xyLoc{0,1}}, search::GridMapState{0, 0, xyLoc{0, 1}}) == 0L);
    REQUIRE(h.getHeuristic(search::GridMapState{0, 0, xyLoc{10,10}}, search::GridMapState{0, 0, xyLoc{10, 10}}) == 0L);
    REQUIRE(h.getHeuristic(search::GridMapState{0, 0, xyLoc{10,10}}, search::GridMapState{0, 0, xyLoc{10, 0}}) == 10L);
    REQUIRE(h.getHeuristic(search::GridMapState{0, 0, xyLoc{10,10}}, search::GridMapState{0, 0, xyLoc{0, 10}}) == 10L);
    
    REQUIRE(h.getHeuristic(search::GridMapState{0, 0, xyLoc{100,100}}, search::GridMapState{0, 0, xyLoc{150, 130}}) == (20L + 42L));

    REQUIRE(h.getHeuristic(search::GridMapState{0, 0, xyLoc{100,100}}, search::GridMapState{0, 0, xyLoc{150, 170}}) == (20L + 70L));
}

SCENARIO("test manhattan") {

    search::ManhattanHeuristic h{maps::GridBranching::FOUR_CONNECTED};

    REQUIRE(h.getHeuristic(search::GridMapState{0, 0, xyLoc{10,10}}, search::GridMapState{0, 0, xyLoc{10, 10}}) == 0L);
    REQUIRE(h.getHeuristic(search::GridMapState{0, 0, xyLoc{10,10}}, search::GridMapState{0, 0, xyLoc{10, 0}}) == 10L);
    REQUIRE(h.getHeuristic(search::GridMapState{0, 0, xyLoc{10,10}}, search::GridMapState{0, 0, xyLoc{0, 10}}) == 10L);
    
    REQUIRE(h.getHeuristic(search::GridMapState{0, 0, xyLoc{100,100}}, search::GridMapState{0, 0, xyLoc{150, 130}}) == (50L + 30L));

    REQUIRE(h.getHeuristic(search::GridMapState{0, 0, xyLoc{100,100}}, search::GridMapState{0, 0, xyLoc{150, 170}}) == (50L + 70L));
}

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