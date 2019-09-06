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

using namespace pathfinding;
using namespace cpp_utils;

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

    REQUIRE(h.getHeuristic(search::GridMapState::make(xyLoc{10,10}), search::GridMapState::make(xyLoc{10, 10})) == 0L);
    REQUIRE(h.getHeuristic(search::GridMapState::make(xyLoc{10,10}), search::GridMapState::make(xyLoc{10, 0})) == 10L);
    REQUIRE(h.getHeuristic(search::GridMapState::make(xyLoc{10,10}), search::GridMapState::make(xyLoc{0, 10})) == 10L);
    
    REQUIRE(h.getHeuristic(search::GridMapState::make(xyLoc{100,100}), search::GridMapState::make(xyLoc{150, 130})) == (20L + 42L));

    REQUIRE(h.getHeuristic(search::GridMapState::make(xyLoc{100,100}), search::GridMapState::make(xyLoc{150, 170})) == (20L + 70L));
}

SCENARIO("test manhattan") {

    search::ManhattanHeuristic h{maps::GridBranching::FOUR_CONNECTED};

    REQUIRE(h.getHeuristic(search::GridMapState::make(xyLoc{10,10}), search::GridMapState::make(xyLoc{10, 10})) == 0L);
    REQUIRE(h.getHeuristic(search::GridMapState::make(xyLoc{10,10}), search::GridMapState::make(xyLoc{10, 0})) == 10L);
    REQUIRE(h.getHeuristic(search::GridMapState::make(xyLoc{10,10}), search::GridMapState::make(xyLoc{0, 10})) == 10L);
    
    REQUIRE(h.getHeuristic(search::GridMapState::make(xyLoc{100,100}), search::GridMapState::make(xyLoc{150, 130})) == (50L + 30L));

    REQUIRE(h.getHeuristic(search::GridMapState::make(xyLoc{100,100}), search::GridMapState::make(xyLoc{150, 170})) == (50L + 70L));
}

SCENARIO("test A*") {

	maps::MovingAIGridMapReader reader{
		'.', cost_t{100},
		'T', cost_t::INFTY,
		'@', cost_t::INFTY
	};
	maps::GridMap map{reader.load(boost::filesystem::path{"./square03.map"})};
	maps::GridMapGraphConverter converter{maps::GridBranching::EIGHT_CONNECTED};
	graphs::AdjacentGraph<std::string, xyLoc, cost_t> graph{*converter.toGraph(map)};

	search::OctileHeuristic heuristic{maps::GridBranching::EIGHT_CONNECTED};
	search::SAPFGridMapGoalChecker goalChecker{};
	search::GridMapStateSupplier supplier{};
	search::GridMapStateExpander<std::string> expander{graph};
	search::PruneIfExpanded<search::GridMapState> pruner{};
    search::NoCloseListSingleGoalAstar<search::GridMapState, xyLoc> astar{heuristic, goalChecker, supplier, expander, pruner};

	search::GridMapState start{graph.idOfVertex({0,0}), {0,0}};
	search::GridMapState goal{graph.idOfVertex({0,0}), {0,0}};
	REQUIRE(astar.search(start, goal) == search::SolutionPath<const search::GridMapState*>{});
	REQUIRE(astar.getSolutionCost(start, goal) == 0);
}