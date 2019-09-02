#include "catch.hpp"

#include "xyLoc.hpp"
#include "OctileHeuristic.hpp"

using namespace pathfinding;
using namespace heuristics;

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

}


SCNEARIO("test octile") {

    OctileHeuristic h{};

    REQUIRE(h.getHeuristic(GridMapState::make(xyLoc{10,10}), GridMapState::make(xyLoc{10, 10})) == 0L);
    REQUIRE(h.getHeuristic(GridMapState::make(xyLoc{10,10}), GridMapState::make(xyLoc{10, 0})) == 10L);
    REQUIRE(h.getHeuristic(GridMapState::make(xyLoc{10,10}), GridMapState::make(xyLoc{0, 10})) == 10L);
    
    REQUIRE(h.getHeuristic(GridMapState::make(xyLoc{100,100}), GridMapState::make(xyLoc{150, 130})) == (20L + 42L));

    REQUIRE(h.getHeuristic(GridMapState::make(xyLoc{100,100}), GridMapState::make(xyLoc{150, 170})) == (20L + 70L));
}

SCENARIO("test A*") {
    
}