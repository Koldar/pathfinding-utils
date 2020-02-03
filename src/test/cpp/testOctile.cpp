#include "catch.hpp"

#include "GridBranching.hpp"
#include "OctileHeuristic.hpp"
#include "map_base_reason_e.hpp"

using namespace pathfinding;

SCENARIO("test octile") {

    search::OctileHeuristic<search::GridMapState<map_base_reason_e>> h{maps::GridBranching::EIGHT_CONNECTED};

	REQUIRE(h.getHeuristic(search::GridMapState<map_base_reason_e>{0, 0, xyLoc{0,0}, map_base_reason_e::INPUT}, search::GridMapState<map_base_reason_e>{0, 0, xyLoc{0, 1}, map_base_reason_e::INPUT}) == 1L);
	REQUIRE(h.getHeuristic(search::GridMapState<map_base_reason_e>{0, 0, xyLoc{0,1}, map_base_reason_e::INPUT}, search::GridMapState<map_base_reason_e>{0, 0, xyLoc{0, 1}, map_base_reason_e::INPUT}) == 0L);
    REQUIRE(h.getHeuristic(search::GridMapState<map_base_reason_e>{0, 0, xyLoc{10,10}, map_base_reason_e::INPUT}, search::GridMapState<map_base_reason_e>{0, 0, xyLoc{10, 10}, map_base_reason_e::INPUT}) == 0L);
    REQUIRE(h.getHeuristic(search::GridMapState<map_base_reason_e>{0, 0, xyLoc{10,10}, map_base_reason_e::INPUT}, search::GridMapState<map_base_reason_e>{0, 0, xyLoc{10, 0}, map_base_reason_e::INPUT}) == 10L);
    REQUIRE(h.getHeuristic(search::GridMapState<map_base_reason_e>{0, 0, xyLoc{10,10}, map_base_reason_e::INPUT}, search::GridMapState<map_base_reason_e>{0, 0, xyLoc{0, 10}, map_base_reason_e::INPUT}) == 10L);
    
    REQUIRE(h.getHeuristic(search::GridMapState<map_base_reason_e>{0, 0, xyLoc{100,100}, map_base_reason_e::INPUT}, search::GridMapState<map_base_reason_e>{0, 0, xyLoc{150, 130}, map_base_reason_e::INPUT}) == (20L + 42L));

    REQUIRE(h.getHeuristic(search::GridMapState<map_base_reason_e>{0, 0, xyLoc{100,100}}, search::GridMapState<map_base_reason_e>{0, 0, xyLoc{150, 170}}) == (20L + 70L));
}