#include "catch.hpp"
#include "MovingAIGridMapReader.hpp"

using namespace cpp_utils;
using namespace pathfinding;
using namespace pathfinding::maps;

SCENARIO("test moving ai gridmap loader") {

	maps::MovingAIGridMapReader reader{'.', cost_t{1000}, color_t::WHITE};
    reader.addTerrain('T', cost_t{1500}, color_t::GREEN);
    reader.addTerrain('@', cost_t::INFTY, color_t::BLACK);

	GIVEN("traversable map") {

		maps::GridMap map{reader.load(boost::filesystem::path{"./combat.map"})};

		REQUIRE(map.getWidth() == 177);
		REQUIRE(map.getHeight() == 193);
		REQUIRE(map.getCellCost({0,0}) == 1000);
		REQUIRE(map.getCellCost({12,15}) == 1500);
		REQUIRE(map.getCellCost({12,14}) == 1000);
	}

    GIVEN("generate gridmap") {

		maps::GridMap map{reader.load(boost::filesystem::path{"./square03.map"})};
        map.saveBMP("testgridMap.bmp");
	}

	GIVEN("map with some untraversabnle cells") {
		maps::GridMap map{reader.load(boost::filesystem::path{"./den000d.map"})};

		REQUIRE(map.getWidth() == 503);
		REQUIRE(map.getHeight() == 351);
		REQUIRE(map.getCellCost({0,0}) == cost_t::INFTY);
	}
}
