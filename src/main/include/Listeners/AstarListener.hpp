#ifndef PATHFINDINGUTILS_ASTARLISTENER_HEADER__
#define PATHFINDINGUTILS_ASTARLISTENER_HEADER__

#include <cpp-utils/ICleanable.hpp>

namespace pathfinding::search::listeners {

    using namespace cpp_utils;

    /**
     * @brief Allows you to further refines the behavior of an A* algorithm
     * 
     */
    template <typename STATE>
    class AstarListener: public ICleanable {
    public:

        /**
         * @brief called whenever a new search has started
         * 
         * By contract this is called before anything else
         * 
         * @param start start state
         * @param goal pattern of the goal to fetch
         */
        //TODO put it here as pure virtual. Now it isn't because I have lots of listener to update
        virtual void onNewSearchStarted(const STATE& start, const STATE* goal);

        /**
         * @brief called whenever a state is pruned by the pruner
         * 
         * @param iteration iteration where the state is discarded
         * @param state the state discarded
         */
        //TODO put it here as pure virtual. Now it isn't because I have lots of listener to update
        virtual void onNodePruned(int iteration, const STATE& state);

        /**
         * @brief called whenever a state is popped from the opern list
         * 
         * @param iteration iteration where the state is popped
         * @param state the state popposed from open
         */
        //TODO put it here as pure virtual. Now it isn't because I have lots of listener to update
        virtual void onNodePoppedFromOpen(int iteration, const STATE& state);

        /**
         * @brief called whenever we examine a state which is also in open list but the state in open list has a g-value which is smaller than the current one
         * 
         * @param iteration iteraton when the event happens
         * @param state stat einvolved
         * @param inOpenG the g value the state in the open list has
         * @param outOpenG the g value we have found by exploring the search state
         */
        //TODO put it here as pure virtual. Now it isn't because I have lots of listener to update
        virtual void onNodeInOpenListHasBetterG(int iteration, const STATE& state, cost_t inOpenG, cost_t outOpenG);

        /**
         * @brief called whenever we examine a state which is also in open list but the state in open list has a g-value which is greater than the current one
         * 
         * @param iteration iteraton when the event happens
         * @param state stat einvolved
         * @param inOpenG the g value the state in the open list has
         * @param outOpenG the g value we have found by exploring the search state
         */
        //TODO put it here as pure virtual. Now it isn't because I have lots of listener to update
        virtual void onNodeInOpenListHasWorseG(int iteration, const STATE& state, cost_t inOpenG, cost_t outOpenG);

        /**
         * @brief called whenever we examine a state which is also in closed list but the state in closed list has a g-value which is smaller than the current one
         * 
         * @param iteration iteraton when the event happens
         * @param state state involved
         * @param inCloseG the g value the state in the closed list has
         * @param outCloseG the g value we have found by exploring the search state
         */
        //TODO put it here as pure virtual. Now it isn't because I have lots of listener to update
        virtual void onNodeInClosedListHasBetterG(int iteration, const STATE& state, cost_t inCloseG, cost_t outCloseG);

        /**
         * @brief called whenever we examine a state which is also in close list but the state in close list has a g-value which is greater than the current one
         * 
         * @param iteration iteraton when the event happens
         * @param state stat einvolved
         * @param inCloseG the g value the state in the closed list has
         * @param outCloseG the g value we have found by exploring the search state
         */
        //TODO put it here as pure virtual. Now it isn't because I have lots of listener to update
        virtual void onNodeInClosedListHasWorseG(int iteration, const STATE& state, cost_t inCloseG, cost_t outCloseG);

        /**
         * @brief brief when we have exausted the open list
         * 
         * @param iteration iterations when the event happens
         */
        //TODO put it here as pure virtual. Now it isn't because I have lots of listener to update
        virtual void onNoSolutionFound(int iteration);

        /**
         * @brief called whenever a new node is expanded from the open list
         * 
         * @param iteration search algorithm iteration when the changes has happended. Iterations start from 0
         * @param node the node expanded
         */
        virtual void onNodeExpanded(int iteration, const STATE& node) = 0;
        /**
         * @brief called whenever a new node is generated from the search. 
         * 
         * Nodes generated by the supplier or from some other sources won't be count.
         * 
         * @param iteration search algorithm iteration when the changes has happended. Iterations start from 0
         * @param node 
         */
        //TODO renamed in "onNodePushedInOpen"
        virtual void onNodeGenerated(int iteration, const STATE& node) = 0;

        /**
         * @brief called each time we start computing the heuristic value of a state
         * 
         * @param iteration search algorithm iteration when the changes has happended. Iterations start from 0
         * @param s 
         */
        virtual void onStartingComputingHeuristic(int iteration, const STATE& s) = 0;

        /**
         * @brief called each time we stop computing the heuristic value of a state
         * 
         * @param iteration search algorithm iteration when the changes has happended. Iterations start from 0
         * @param s 
         */
        virtual void onEndingComputingHeuristic(int iteration, const STATE& s) = 0;
        
        /**
         * @brief called whenever a new goal is found
         * 
         * @param iteration search algorithm iteration when the changes has happended. Iterations start from 0
         * @param goal the goal we have reached
         */
        virtual void onSolutionFound(int iteration, const STATE& goal) = 0;
    };

}

#endif