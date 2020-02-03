#include "catch.hpp"

#include "GraphState.hpp"

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

		WHEN("GrapState with cost_t") {
			TestState state{0, graph, 0};
			search::GraphStateSupplier<std::string, xyLoc, cost_t> supplier{graph}; 
			search::StandardStateExpander<TestState, std::string, xyLoc, cost_t> expander{graph};
			search::StandardLocationGoalChecker<TestState> goalChecker{};
			REQUIRE(state.getId() == 0);
		}

		WHEN("GrapState with default") {
			search::GraphState<std::string,  xyLoc, cost_t> state{0, graph, 0};
			search::GraphStateSupplier<std::string, xyLoc, cost_t> supplier{graph}; 
			search::StandardStateExpander<search::GraphState<std::string,  xyLoc, cost_t>, std::string, xyLoc, cost_t> expander{graph};
			search::StandardLocationGoalChecker<TestState, cost_t> goalChecker{};
			REQUIRE(state.getId() == 0);
		}

		WHEN("GrapState with other") {
			search::GraphState<std::string, xyLoc, OtherCost> state{0, graph2, 0};
			search::GraphStateSupplier<std::string, xyLoc, OtherCost> supplier{graph2}; 
			search::StandardStateExpander<search::GraphState<std::string,  xyLoc, cost_t>, std::string, xyLoc, OtherCost, OtherCost::getCost> expander{graph2};
			search::StandardLocationGoalChecker<search::GraphState<std::string, xyLoc, OtherCost>> goalChecker{};
			REQUIRE(state.getId() == 0);
		}
		
	}

}