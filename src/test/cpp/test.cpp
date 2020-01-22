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
#include "ISolutionPath.hpp"
#include <functional>
#include "utils.hpp"
#include "pathValidators.hpp"

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
		'.', cost_t{1000}, color_t::WHITE,
		'T', cost_t{1500}, color_t::GREEN,
		'@', cost_t::INFTY, color_t::BLACK
	};

	GIVEN("traversable map") {

		maps::GridMap map{reader.load(boost::filesystem::path{"./square03.map"})};
		maps::GridMapGraphConverter converter{maps::GridBranching::EIGHT_CONNECTED};
		graphs::AdjacentGraph<std::string, xyLoc, cost_t> graph{*converter.toGraph(map)};
		graphs::ListGraph<std::string, xyLoc, OtherCost> graph2{*graph.mapEdges<OtherCost>([&] (const cost_t& c) { return OtherCost{c, false};})};

		WHEN("GrapState with cost_t") {
			search::GraphState<std::string, xyLoc, cost_t> state{0, graph, 0};
			search::GraphStateSupplier<std::string, xyLoc, cost_t> supplier{graph}; 
			search::StandardStateExpander<search::GraphState<std::string, xyLoc, cost_t>, std::string, xyLoc, cost_t> expander{graph};
			search::StandardLocationGoalChecker<search::GraphState<std::string, xyLoc, cost_t>> goalChecker{};
			REQUIRE(state.getId() == 0);
		}

		WHEN("GrapState with default") {
			search::GraphState<std::string, xyLoc> state{0, graph, 0};
			search::GraphStateSupplier<std::string, xyLoc, cost_t> supplier{graph}; 
			search::StandardStateExpander<search::GraphState<std::string, xyLoc>, std::string, xyLoc, cost_t> expander{graph};
			search::StandardLocationGoalChecker<search::GraphState<std::string, xyLoc>> goalChecker{};
			REQUIRE(state.getId() == 0);
		}

		WHEN("GrapState with other") {
			search::GraphState<std::string, xyLoc, OtherCost> state{0, graph2, 0};
			search::GraphStateSupplier<std::string, xyLoc, OtherCost> supplier{graph2}; 
			search::StandardStateExpander<search::GraphState<std::string, xyLoc>, std::string, xyLoc, OtherCost, OtherCost::getCost> expander{graph2};
			search::StandardLocationGoalChecker<search::GraphState<std::string, xyLoc, OtherCost>> goalChecker{};
			REQUIRE(state.getId() == 0);
		}
		
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



SCENARIO("test validator") {

	maps::MovingAIGridMapReader reader{
		'.', cost_t{100}, color_t::WHITE,
		'T', cost_t::INFTY, color_t::BLACK,
		'@', cost_t::INFTY, color_t::BLACK
	};
	maps::GridMap map{reader.load(boost::filesystem::path{"./square03.map"})};

	/*
	* MAP
	* 
	*  .....
	*  ...@@
	*  ..@@.
	*  ...@@
	*  .....
	* 
	*/


	maps::GridMapGraphConverter converter{maps::GridBranching::EIGHT_CONNECTED};
	graphs::AdjacentGraph<std::string, xyLoc, cost_t> graph{*converter.toGraph(map)};

	GIVEN("optimality") {
		search::GraphSolutionPath<std::string, xyLoc, cost_t> path{graph, GetCost<cost_t>{}};
		path.add(
			graph.idOfVertex(xyLoc{0,0}),
			graph.idOfVertex(xyLoc{1,0}),
			graph.idOfVertex(xyLoc{2,0}),
			graph.idOfVertex(xyLoc{3,0})
		);

		validator::checkPathValid<std::string, xyLoc, cost_t, graphs::nodeid_t, graphs::nodeid_t>(graph, path, [&](graphs::nodeid_t id) { return id; });
		validator::checkIfPathOptimal<std::string, xyLoc, cost_t, graphs::nodeid_t, graphs::nodeid_t>(
			graph, 
			graph.idOfVertex(xyLoc{0,0}), graph.idOfVertex(xyLoc{3,0}), 
			path,
			GetCost<cost_t>{},
			[&](graphs::nodeid_t v) { return v; }
		);

		REQUIRE_THROWS(validator::checkIfPathOptimal<std::string, xyLoc, cost_t, graphs::nodeid_t, graphs::nodeid_t>(
			graph, 
			graph.idOfVertex(xyLoc{0,0}), graph.idOfVertex(xyLoc{4,0}), 
			path,
			GetCost<cost_t>{},
			[&](graphs::nodeid_t v) { return v; }
		));
	}

	GIVEN("suboptimality") {

		WHEN("check optimal path") {
			search::GraphSolutionPath<std::string, xyLoc, cost_t> optimalPath{graph, GetCost<cost_t>{}};
			optimalPath.add(
				graph.idOfVertex(xyLoc{0,0}),
				graph.idOfVertex(xyLoc{1,0}),
				graph.idOfVertex(xyLoc{2,0}),
				graph.idOfVertex(xyLoc{3,0})
			);

			validator::checkIfPathSuboptimalityBound<std::string, xyLoc, cost_t, graphs::nodeid_t, graphs::nodeid_t>(
				2,
				graph,
				graph.idOfVertex(xyLoc{0,0}), graph.idOfVertex(xyLoc{3,0}), 
				optimalPath,
				GetCost<cost_t>{},
				[&](graphs::nodeid_t v) { return v; }
			);
		}

		WHEN("checking compliant suboptimal path") {
			search::GraphSolutionPath<std::string, xyLoc, cost_t> path{graph, GetCost<cost_t>{}};
			path.add(
				graph.idOfVertex(xyLoc{0,0}),
				graph.idOfVertex(xyLoc{1,1}),
				graph.idOfVertex(xyLoc{2,0}),
				graph.idOfVertex(xyLoc{3,0})
			);

			validator::checkIfPathSuboptimalityBound<std::string, xyLoc, cost_t, graphs::nodeid_t, graphs::nodeid_t>(
				2,
				graph,
				graph.idOfVertex(xyLoc{0,0}), graph.idOfVertex(xyLoc{3,0}), 
				path,
				GetCost<cost_t>{},
				[&](graphs::nodeid_t v) { return v; }
			);
		}

		WHEN("checking uncompliant suboptimal path") {
			search::GraphSolutionPath<std::string, xyLoc, cost_t> path{graph, GetCost<cost_t>{}};
			path.add(
				graph.idOfVertex(xyLoc{0,0}),
				graph.idOfVertex(xyLoc{1,0}),
				graph.idOfVertex(xyLoc{2,1}),
				graph.idOfVertex(xyLoc{3,0})
			);

			REQUIRE_THROWS(validator::checkIfPathSuboptimalityBound<std::string, xyLoc, cost_t, graphs::nodeid_t, graphs::nodeid_t>(
				1.001,
				graph,
				graph.idOfVertex(xyLoc{0,0}), graph.idOfVertex(xyLoc{3,0}), 
				path,
				GetCost<cost_t>{},
				[&](graphs::nodeid_t v) { return v; }
			));
		}

		WHEN("checking a path not ending in goal") {

			search::GraphSolutionPath<std::string, xyLoc, cost_t> path{graph, GetCost<cost_t>{}};
			path.add(
				graph.idOfVertex(xyLoc{0,0}),
				graph.idOfVertex(xyLoc{1,0}),
				graph.idOfVertex(xyLoc{2,0})
			);

			REQUIRE_THROWS(validator::checkIfPathSuboptimalityBound<std::string, xyLoc, cost_t, graphs::nodeid_t, graphs::nodeid_t>(
				2,
				graph,
				graph.idOfVertex(xyLoc{0,0}), graph.idOfVertex(xyLoc{3,0}), 
				path,
				GetCost<cost_t>{},
				[&](graphs::nodeid_t v) { return v; }
			));
		}

		WHEN("checking a path not starting in start") {

			search::GraphSolutionPath<std::string, xyLoc, cost_t> path{graph, GetCost<cost_t>{}};
			path.add(
				graph.idOfVertex(xyLoc{1,0}),
				graph.idOfVertex(xyLoc{2,0}),
				graph.idOfVertex(xyLoc{3,0})
			);

			REQUIRE_THROWS(validator::checkIfPathSuboptimalityBound<std::string, xyLoc, cost_t, graphs::nodeid_t, graphs::nodeid_t>(
				2,
				graph,
				graph.idOfVertex(xyLoc{0,0}), graph.idOfVertex(xyLoc{3,0}), 
				path,
				GetCost<cost_t>{},
				[&](graphs::nodeid_t v) { return v; }
			));
		}

		WHEN("checking a path jumping an edge") {

			search::GraphSolutionPath<std::string, xyLoc, cost_t> path{graph, GetCost<cost_t>{}};
			path.add(
				graph.idOfVertex(xyLoc{0,0}),
				graph.idOfVertex(xyLoc{1,0}),
				graph.idOfVertex(xyLoc{3,0})
			);

			REQUIRE_THROWS(validator::checkIfPathSuboptimalityBound<std::string, xyLoc, cost_t, graphs::nodeid_t, graphs::nodeid_t>(
				2,
				graph,
				graph.idOfVertex(xyLoc{0,0}), graph.idOfVertex(xyLoc{3,0}), 
				path,
				GetCost<cost_t>{},
				[&](graphs::nodeid_t v) { return v; }
			));

		}
	}
}