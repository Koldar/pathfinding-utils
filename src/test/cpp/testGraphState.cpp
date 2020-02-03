#include "catch.hpp"

#include "GraphState.hpp"
#include "xyLoc.hpp"
#include "MovingAIGridMapReader.hpp"
#include "GridMapGraphConverter.hpp"
#include "OtherCost.hpp"
#include "StandardStateExpander.hpp"
#include "StandardLocationGoalChecker.hpp"

using namespace pathfinding;
using namespace pathfinding::search;
using namespace pathfinding::maps;

SCENARIO("test GraphState") {

	using TestState = search::GraphState<std::string, xyLoc, cost_t, bool>;

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

        const cpp_utils::function_t<cost_t,cost_t> fromCostToCost = [&](auto c) { return c;};
        const cpp_utils::function_t<OtherCost,cost_t> fromOtherCostToCost = [&](auto c) { return c.cost;};

		WHEN("GrapState with cost_t") {
			TestState state{0, graph, 0, false};
			search::GraphStateSupplier<std::string, xyLoc, cost_t, bool> supplier{graph}; 
			search::StandardStateExpander<TestState, std::string, xyLoc, cost_t, bool> expander{graph, fromCostToCost};
			search::StandardLocationGoalChecker<TestState> goalChecker{};
			REQUIRE(state.getId() == 0);
		}

		WHEN("GrapState with default") {
			TestState state{0, graph, 0, false};
			search::GraphStateSupplier<std::string, xyLoc, cost_t, bool> supplier{graph}; 
			search::StandardStateExpander<TestState, std::string, xyLoc, cost_t, bool> expander{graph, fromCostToCost};
			search::StandardLocationGoalChecker<TestState> goalChecker{};
			REQUIRE(state.getId() == 0);
		}

		WHEN("GrapState with other") {
			search::GraphState<std::string, xyLoc, OtherCost, bool> state{0, graph2, 0, false};
			search::GraphStateSupplier<std::string, xyLoc, OtherCost, bool> supplier{graph2}; 
			search::StandardStateExpander<TestState, std::string, xyLoc, OtherCost, bool> expander{graph2, fromOtherCostToCost};
			search::StandardLocationGoalChecker<search::GraphState<std::string, xyLoc, OtherCost, bool>> goalChecker{};
			REQUIRE(state.getId() == 0);
		}
		
	}

}