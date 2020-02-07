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
     * This is by design. If you need to alter the algorithm cycle, consider copying AStar.hpp file.
     * 
     */
    template <typename STATE, typename... STATE_IMPORTANT_TYPES>
    class ClassicAStar: public IMemorable, public ISearchAlgorithm<STATE, const STATE*, const STATE&>, public ISingleListenable<listeners::AstarListener<STATE>> {
    public:
        using Listener = listeners::AstarListener<STATE>;
        using Expander = IStateExpander<STATE, STATE_IMPORTANT_TYPES...>;
        using Supplier =  IStateSupplier<STATE, STATE_IMPORTANT_TYPES...>;
    public:
        ClassicAStar(IHeuristic<STATE>& heuristic, IGoalChecker<STATE>& goalChecker, Supplier& supplier, Expander& expander, IStatePruner<STATE>& pruner,  unsigned int openListCapacity = 1024) : 
            ISingleListenable<Listener>{}, 
            heuristic{heuristic}, goalChecker{goalChecker}, supplier{supplier}, expander{expander}, pruner{pruner},
            openList{nullptr} {
                if (!heuristic.isConsistent()) {
                    throw cpp_utils::exceptions::InvalidArgumentException{"the heuristic is not consistent!"};
                }
                this->openList = new StaticPriorityQueue<STATE>{openListCapacity, true};
            }

        virtual ~ClassicAStar() {
            this->tearDownSearch();
            delete this->openList;
        }
        //the class cannot be copied whatsoever
        NoCloseListSingleGoalAstar(const NoCloseListSingleGoalAstar& other) = delete;
        NoCloseListSingleGoalAstar& operator=(const NoCloseListSingleGoalAstar& other) = delete;

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
        IStateExpander<STATE, STATE_IMPORTANT_TYPES...>& expander;
        IStateSupplier<STATE, STATE_IMPORTANT_TYPES...>& supplier;
        IStatePruner<STATE>& pruner;
        StaticPriorityQueue<STATE>* openList;
    protected:
        virtual cost_t computeF(cost_t g, cost_t h) const {
            return g + h;
        }
    public:
        virtual std::string getName() const {
            return std::string{"A*"};
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

                    this->fireEvent([&current, aStarIteration](Listener& l) { l.onSolutionFound(aStarIteration, current); });
                    goto goal_found;
                }

                this->openList->pop();

                current.markAsExpanded();
                this->fireEvent([&current, aStarIteration](Listener& l) { l.onNodeExpanded(aStarIteration,current); });

                info("computing successors of state ", current, "...");
                for(auto pair: this->expander.getSuccessors(current, this->supplier)) {
                    STATE& successor = pair.first;
                    cost_t current_to_successor_cost = pair.second;

                    if (this->pruner.shouldPrune(successor)) {
                        info("child", successor, "of state ", current, "should be pruned!");
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
                }

                aStarIteration += 1;
            }
            info("found no solutions!");
            throw SolutionNotFoundException{};

            goal_found:
            return *goal;

        }

    };

}

#endif