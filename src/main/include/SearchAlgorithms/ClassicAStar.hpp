/**
 * @file 
 * @author Massimo Bono
 * @brief standard A* implementation. This version can be fine tuned by overriding specific methods
 * @version 0.1
 * @date 2020-02-07
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef _PATHFINDINGUTILS_PLUGGABLEASTAR_HEADER__
#define _PATHFINDINGUTILS_PLUGGABLEASTAR_HEADER__

#include "ISearchAlgorithm.hpp"

namespace pathfinding::search {

    /**
     * @brief A generic A\* implementation where the main events can be override by subclassing. However, the main algorithm cycle **cannot** be updated.
     * 
     * This is by design. If you need to alter the algorithm cycle, consider copying AStar.hpp file or create a new version altogether: this version only allows small modifications to the A\* loop
     * 
     */
    template <typename STATE, typename... STATE_IMPORTANT_TYPES>
    class ClassicAStar: public IMemorable, public ISearchAlgorithm<STATE, const STATE*, const STATE&>, public ISingleListenable<listeners::AstarListener<STATE>> {
    public:
        using Listener = listeners::AstarListener<STATE>;
        using Expander = IStateCostExpander<STATE, STATE_IMPORTANT_TYPES...>;
        using Supplier =  IStateSupplier<STATE, STATE_IMPORTANT_TYPES...>;
        using This = ClassicAStar<STATE, STATE_IMPORTANT_TYPES...>;
        using Super1 = ISearchAlgorithm<STATE, const STATE*, cosnt STATE&>;
    public:
        ClassicAStar(IHeuristic<STATE>& heuristic, IGoalChecker<STATE>& goalChecker, Supplier& supplier, Expander& expander, IStatePruner<STATE>& pruner,  unsigned int openListCapacity = 1024) : 
            ISingleListenable<Listener>{}, 
            heuristic{heuristic}, goalChecker{goalChecker}, supplier{supplier}, expander{expander}, pruner{pruner},
            openList{nullptr} {
                this->openList = new StaticPriorityQueue<STATE>{openListCapacity, true};
            }

        virtual ~ClassicAStar() {
            this->tearDownSearch();
            delete this->openList;
        }
        //the class cannot be copied whatsoever
        ClassicAStar(const This& o) = delete;
        This& operator=(const This& o) = delete;
        ClassicAStar(This&& o) = delete;
        This& operator=(This&& o) = delete;
    public:
        virtual MemoryConsumption getByteMemoryOccupied() const {
            throw cpp_utils::exceptions::NotYetImplementedException{__func__};
        }
    private:
        IHeuristic<STATE>& heuristic;
        IGoalChecker<STATE>& goalChecker;
        Expander& expander;
        Supplier& supplier;
        IStatePruner<STATE>& pruner;
        StaticPriorityQueue<STATE>* openList;
    protected:
        /**
         * @brief function used to compute the evaluation of a state
         * 
         * @param g g value of the state
         * @param h h value of the state
         * @return cost_t the f value of the state
         */
        virtual cost_t computeF(cost_t g, cost_t h) const {
            return g + h;
        }
    public:
        virtual std::string getName() const {
            return std::string{"A*-classic"};
        }
        virtual void setupSearch(const STATE* start, const STATE* goal) {
            this->doOnObserver([&](auto& l) { l.cleanup();});
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
            return std::unique_ptr{result};
        }
        virtual cost_t getSolutionCostFromGoalFetched(const STATE& start, const STATE& actualGoal, const STATE* goal) const {
            return actualGoal.getCost();
        }
        virtual const STATE& performSearch(STATE& start, const STATE* expectedGoal) {
            this->fireEvent([&start, &expectedGoal](auto& l) { l.onNewSearchStarted(start, expecteGoal)});
            
            int aStarIteration = 0;
            STATE* goal = nullptr;

            start.setG(0);
            this->fireEvent([&start, aStarIteration](Listener& l) { l.onStartingComputingHeuristic(aStarIteration, start); });
            start.setH(this->heuristic.getHeuristic(start, expectedGoal));
            this->fireEvent([&start, aStarIteration](Listener& l) { l.onEndingComputingHeuristic(aStarIteration, start); });
            start.setF(this->computeF(start.getG(), start.getH()));

            this->openList->push(start);
            this->fireEvent([&start, aStarIteration](auto& l) { l.onNodeGenerated(aStarIteration, start)});
            while (!this->openList->isEmpty()) {
                STATE& current = this->openList->peek();
                this->fireEvent([&current, aStarIteration](auto& l) { l.onNodePoppedFromOpen(aStarIteration, current)});

                if (this->goalChecker.isGoal(current, expectedGoal)) {
                    goal = &current;

                    this->fireEvent([&current, aStarIteration](Listener& l) { l.onSolutionFound(aStarIteration, current); });
                    goto goal_found;
                }

                this->openList->pop();
                current.markAsExpanded();

                this->fireEvent([&current, aStarIteration](Listener& l) { l.onNodeExpanded(aStarIteration,current); });


                this->fireEvent([&current, aStarIteration](Listener& l) { l.onStartingComputingSuccessors(aStarIteration, current); });
                auto successors = this->expander.getSuccessors(current, this->supplier);
                this->fireEvent([&current, aStarIteration](Listener& l) { l.onEndingComputingSuccessors(aStarIteration, current); });
                for(auto outcome: successors) {
                    STATE& successor = std::get<0>(outcome);
                    cost_t current_to_successor_cost = std::get<1>(outcome);

                    if (this->pruner.shouldPrune(successor)) {
                        //skip neighbours already expanded
                        this->fireEvent([&current, aStarIteration](Listener& l) { l.onNodePruned(aStarIteration, successor);})
                        continue;
                    }

                    if (this->openList->contains(successor)) {
                        //state inside the open list. Check if we need to update the path
                        cost_t gval = current.getG() + current_to_successor_cost;
                        if (gval < successor.getG()) {
                            //update successor information
                            successor.setG(gval);
                            successor.setF(this->computeF(gval, successor.getH()));
                            const STATE* oldParent = successor.getParent();
                            successor.setParent(&current);

                            this->openList->decrease_key(successor);

                            this->fireEvent([&successor, aStarIteration](Listener& l) { l.onNodeInOpenListHasWorseG(aStarIteration, successor, successor.getG(), gval);});
                        } else {
                            this->fireEvent([&successor, aStarIteration](Listener& l) { l.onNodeInOpenListHasBetterG(aStarIteration, successor, successor.getG(), gval);});
                        }
                    } else if (successor.isExpanded()) {
                        //state in closed list
                        cost_t gval = current.getG() + current_to_successor_cost;
                        if (gval < successor.getG()) {
                            //this is impossible when the heuristic is admissible

                            //update successor information
                            successor.setG(gval);
                            successor.setF(this->computeF(gval, successor.getH()));
                            const STATE* oldParent = successor.getParent();
                            successor.setParent(&current);
                            successor.setExpanded(false);

                            this->openList->decrease_key(successor);

                            this->fireEvent([&successor, aStarIteration](Listener& l) { l.onNodeInClosedListHasWorseG(aStarIteration, successor, successor.getG(), gval);});
                        } else {
                            this->fireEvent([&successor, aStarIteration](Listener& l) { l.onNodeInClosedListHasBetterG(aStarIteration, successor, successor.getG(), gval);});
                        }
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

                        this->openList->push(successor);

                        this->fireEvent([&current, &successor, aStarIteration](Listener& l) {l.onNodeGenerated(aStarIteration, successor); });
                    }
                }

                aStarIteration += 1;
            }
            this->fireEvent([aStarIteration](auto& l) { l.onNoSolutionFound(aStariteration); });
            throw SolutionNotFoundException{};

            goal_found:
            return *goal;

        }

    };

}

#endif