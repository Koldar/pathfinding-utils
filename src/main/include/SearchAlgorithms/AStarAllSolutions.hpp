#ifndef _ASTAR_HEADER__
#define _ASTAR_HEADER__

#include <cpp-utils/StaticPriorityQueue.hpp>
#include <cpp-utils/log.hpp>
#include <cpp-utils/listeners.hpp>
#include <cpp-utils/imemory.hpp>
#include <cpp-utils/commons.hpp>
#include <cpp-utils/profiling.hpp>

#include "IHeuristic.hpp"
#include "ISearchAlgorithm.hpp"
#include "IStateExpander.hpp"
#include "IStatePruner.hpp"
#include "IStateSupplier.hpp"
#include "IGoalChecker.hpp"

#include "AstarListener.hpp"


namespace pathfinding::search {

    using namespace cpp_utils;
    using namespace pathfinding::search::listeners;

    /**
     * @brief an impementation of A* which find all the optimal solutions there are in a problem
     * 
     * The class has been inspired by Daniel Harabor implementation.
     * 
     * This A* star algorithm is perfect in the following case:
     *  - the heuristic is consistent (no need for close list);
     *  - the goal can be represented by a single state (goal known apriori): for example
     *      in single agent pathfinding this is ensured while in classical planning this is not.
     * 
     * A* implementation that allows arbitrary combinations of 
     * (weighted) heuristic functions and node expansion policies.
     * This implementation uses a binary heap for the open_ list
     * and a bit array for the closed_ list (under the form of `expanded` flag in ::IState)
     * 
     * @author: dharabor
     * @created: 21/08/2012
     */
    template <typename STATE, typename... STATE_IMPORTANT_TYPES>
    class AStarAllSolutions: public IMemorable, public ISearchAlgorithm<STATE, const STATE*, const STATE&>, public ISingleListenable<AstarListener<STATE>> {
    public:
        using This = AStarAllSolutions<STATE, STATE_IMPORTANT_TYPES...>;
        using Listener = AstarListener<STATE>;
        using Expander = IStateCostExpander<STATE, STATE_IMPORTANT_TYPES...>;
    public:
        AStarAllSolutions(IHeuristic<STATE>& heuristic, IGoalChecker<STATE>& goalChecker, IStateSupplier<STATE, STATE_IMPORTANT_TYPES...>& supplier, Expander& expander, IStatePruner<STATE>& pruner,  unsigned int openListCapacity = 1024) : 
            ISingleListenable<Listener>{}, 
            heuristic{heuristic}, goalChecker{goalChecker}, supplier{supplier}, expander{expander}, pruner{pruner},
            openList{nullptr} {
                if (!heuristic.isConsistent()) {
                    throw cpp_utils::exceptions::InvalidArgumentException{"the heuristic is not consistent!"};
                }
                this->openList = new StaticPriorityQueue<STATE>{openListCapacity, true};
            }

        virtual ~AStarAllSolutions() {
            this->tearDownSearch();
            delete this->openList;
        }
        //the class cannot be copied whatsoever
        AStarAllSolutions(const This& other) = delete;
        AStarAllSolutions& operator=(const This& other) = delete;

    public:
        virtual MemoryConsumption getByteMemoryOccupied() const {
            throw cpp_utils::exceptions::NotYetImplementedException{__func__};
            //return
                // memory for the priority quete
                //this->openList->getByteMemoryOccupied() + 
                // gridmap size and other stuff needed to expand nodes
                //this->heuristic.getByteMemoryOccupied() +
                // this->goalChecker.getByteMemoryOccupied() +
                // this->expander.getByteMemoryOccupied() +
                // this->supplier.getByteMemoryOccupied() +
                // this->pruner.getByteMemoryOccupied() +
                // // misc
                // MemoryConsumption{sizeof(*this), MemoryConsumptionEnum::BYTE};
        }
    private:
        IHeuristic<STATE>& heuristic;
        IGoalChecker<STATE>& goalChecker;
        Expander& expander;
        IStateSupplier<STATE, STATE_IMPORTANT_TYPES...>& supplier;
        IStatePruner<STATE>& pruner;
        StaticPriorityQueue<STATE>* openList;
    protected:
        virtual cost_t computeF(cost_t g, cost_t h) const {
            return g + h;
        }
    public:
        virtual std::string getName() const {
            return std::string{"A* all solutions"};
        }
        virtual void setupSearch(const STATE* start, const STATE* goal) {
            this->doOnObserver([&](Listener& l) { l.cleanup();});
            //cleanup before running since at the end we may want to poll information on the other structures
            this->heuristic.cleanup();
            this->expander.cleanup();
            this->supplier.cleanup();
            this->pruner.cleanup();
            this->openList->clear();
        }
        virtual void tearDownSearch() {
        }
    protected:
        virtual std::unique_ptr<ISolutionPath<STATE>> buildSolutionFromGoalFetched(const STATE& start, const STATE& actualGoal, const STATE* goal) {
            auto result = new StateSolutionPath<STATE>{};
            const STATE* tmp = &actualGoal;
            while (tmp != nullptr) {
                info("adding ", *tmp, "to solution!");
                result->addHead(*tmp);
                tmp = tmp->getParent();
            }
            return std::unique_ptr<StateSolutionPath<STATE>>{result};
        }
        virtual cost_t getSolutionCostFromGoalFetched(const STATE& start, const STATE& actualGoal, const STATE* goal) const {
            return actualGoal.getCost();
        }
        virtual const STATE& performSearch(STATE& start, const STATE* expectedGoal) {
            if (expectedGoal != nullptr) {
                info("starting A*! start = ", start, "goal = ", *expectedGoal);
            } else {
                info("starting A*! start = ", start, "goal = ", "none");
            }
            
            int aStarIteration = 0;
            STATE* goal = nullptr;
            cost_t optimalSolutionCost = 0;

            start.setG(0);
            this->fireEvent([&start, aStarIteration](Listener& l) { l.onStartingComputingHeuristic(aStarIteration, start); });
            start.setH(this->heuristic.getHeuristic(start, expectedGoal));
            this->fireEvent([&start, aStarIteration](Listener& l) { l.onEndingComputingHeuristic(aStarIteration, start); });
            start.setF(this->computeF(start.getG(), start.getH()));

            this->openList->push(start);
            while (!this->openList->isEmpty()) {
                STATE& current = this->openList->peek();
                info("state ", current, "popped from open list f=", current.getF(), "g=", current.getG(), "h=", current.getH());

                if (this->goalChecker.isGoal(current, expectedGoal)) {
                    info("state ", current, "is a goal!");
                    goal = &current;

                    this->openList->pop();
                    current.markAsExpanded();
                    optimalSolutionCost = current.getF();

                    this->fireEvent([&current, aStarIteration](Listener& l) { l.onSolutionFound(aStarIteration, current); });
                    continue;
                }

                this->openList->pop();

                current.markAsExpanded();
                this->fireEvent([&current, aStarIteration](Listener& l) { l.onNodeExpanded(aStarIteration, current); });

                // if a goal has already been found, we prune away all the states which have f greater than the oe we have found
                if ((goal != nullptr) && (current.getF() > optimalSolutionCost)) {
                    warning("state", current, "pruned since it is beyond the optimal solution cost! (", current.getF(), ">", optimalSolutionCost, ")");
                    continue;
                }

                info("computing successors of state ", current, "...");
                for(auto outcome: this->expander.getSuccessors(current, this->supplier)) {
                    STATE& successor = outcome.getState();
                    cost_t current_to_successor_cost = outcome.getCostToReachState();

                    // if we have found a solution, we need to reopen a closed state if the newg <= oldg
                    // if we haven't found a ssolution yet, we need to reopen a closed state if newg < oldg

                    if ((goal != nullptr) && (current.getF() > optimalSolutionCost)) {
                        warning("state", current, "pruned since it is beyond the optimal solution cost! (", current.getF(), ">", optimalSolutionCost, ")");
                        continue;
                    }

                    if (this->pruner.shouldPrune(successor)) {
                        info("child", successor, "of state ", current, "should be pruned since it doesn't respect prune criterion!");
                        //skip neighbours already expanded
                        continue;
                    }

                    if (this->openList->contains(successor)) {
                        //state inside the open list. Check if we need to update the path
                        cost_t gval = current.getG() + current_to_successor_cost;
                        if (gval < successor.getG()) {
                            info("child", successor, "of state ", current, "present in open list and has a lower g. update its parent!");

                            //update successor information
                            successor.setG(gval);
                            successor.setF(this->computeF(gval, successor.getH()));
                            const STATE* oldParent = successor.getParent();
                            successor.setParent(&current);

                            this->openList->decrease_key(successor);
                        }
                    } else if (successor.isExpanded()) {
                        //state belong to closed list
                        cost_t gval = current.getG() + current_to_successor_cost;

                        //check if there is scenario where we can ignore the state
                        if (goal != nullptr) {
                            //a solution has been already found
                            if (gval > successor.getG()) {
                                continue;
                            }
                        } else {
                            // no solution has been found yet
                            if (gval >= successor.getG()) {
                                continue;
                            }
                        }

                        //otherwise reput in openlist
                        //reput in open list
                        successor.setG(gval);
                        successor.setF(this->computeF(gval, successor.getH()));
                        const STATE* oldParent = successor.getParent();
                        successor.setParent(&current);
                        this->openList->push(successor);
                    } else {
                        //state is not present in open list. Add to it
                        cost_t gval = current.getG() + current_to_successor_cost;
                        this->fireEvent([&successor, aStarIteration](Listener& l) { l.onStartingComputingHeuristic(aStarIteration, successor); });
                        cost_t hval = this->heuristic.getHeuristic(successor, expectedGoal);
                        this->fireEvent([&successor, aStarIteration](Listener& l) { l.onEndingComputingHeuristic(aStarIteration, successor); });
                        successor.setG(gval);
                        successor.setH(hval);
                        successor.setF(this->computeF(gval, hval));
                        successor.setParent(&current);

                        this->fireEvent([&current, &successor, aStarIteration](Listener& l) {l.onNodeGenerated(aStarIteration, successor); });
                        info("child", successor, "of state ", current, "not present in open list. Add it f=", successor.getF(), "g=", successor.getG(), "h=", successor.getH());
                        this->openList->push(successor);
                        
                    }

                    aStarIteration += 1;
                }
            }
            info("we have popped everything from openlist!");
            if (goal == nullptr) {
                //no solution has been found whatsoever
                info("found no solutions!");
                throw SolutionNotFoundException{};
            }

            return *goal;
        }



    };

}

#endif