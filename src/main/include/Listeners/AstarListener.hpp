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
         * @brief called whenever a new node is expanded from the open list
         * 
         * @param node 
         */
        virtual void onNodeExpanded(const STATE& node) = 0;
        /**
         * @brief called whenever a new node is generated from the search. 
         * 
         * Nodes generated by the supplier or from some other sources won't be count.
         * 
         * @param node 
         */
        virtual void onNodeGenerated(const STATE& node) = 0;

        /**
         * @brief called each time we start computing the heuristic value of a state
         * 
         * @param s 
         */
        virtual void onStartingComputingHeuristic(const STATE& s) = 0;

        /**
         * @brief called each time we stop computing the heuristic value of a state
         * 
         * @param s 
         */
        virtual void onEndingComputingHeuristic(const STATE& s) = 0;
        
        /**
         * @brief called whenever a new goal is found
         * 
         * @param goal the goal we have reached
         */
        virtual void onSolutionFound(const STATE& goal) = 0;
    };

}

#endif