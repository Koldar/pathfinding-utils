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

    /**
     * @brief an impementation of A*
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
    class NoCloseListSingleGoalAstar: public IMemorable, public ISearchAlgorithm<STATE, const STATE*, const STATE&>, public ISingleListenable<listeners::AstarListener<STATE>> {
    public:
        using Listener = listeners::AstarListener<STATE>;
    public:
        NoCloseListSingleGoalAstar(IHeuristic<STATE>& heuristic, IGoalChecker<STATE>& goalChecker, IStateSupplier<STATE, STATE_IMPORTANT_TYPES...>& supplier, IStateExpander<STATE, STATE_IMPORTANT_TYPES...>& expander, IStatePruner<STATE>& pruner,  unsigned int openListCapacity = 1024) : 
            ISingleListenable<Listener>{}, 
            heuristic{heuristic}, goalChecker{goalChecker}, supplier{supplier}, expander{expander}, pruner{pruner},
            openList{nullptr} {
                if (!heuristic.isConsistent()) {
                    throw cpp_utils::exceptions::InvalidArgumentException{"the heuristic is not consistent!"};
                }
                this->openList = new StaticPriorityQueue<STATE>{openListCapacity, true};
            }

        virtual ~NoCloseListSingleGoalAstar() {
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
        virtual std::unique_ptr<ISolutionPath<const STATE*, const STATE&>> buildSolutionFromGoalFetched(const STATE& start, const STATE& actualGoal, const STATE* goal) {
            auto result = new StateSolutionPath<STATE>{};
            const STATE* tmp = &actualGoal;
            while (tmp != nullptr) {
                info("adding ", *tmp, "to solution!");
                result->addHead(tmp);
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
            

            STATE* goal = nullptr;

            start.setG(0);
            this->fireEvent([&start](Listener& l) { l.onStartingComputingHeuristic(start); });
            start.setH(this->heuristic.getHeuristic(start, expectedGoal));
            this->fireEvent([&start](Listener& l) { l.onEndingComputingHeuristic(start); });
            start.setF(this->computeF(start.getG(), start.getH()));

            this->openList->push(start);
            while (!this->openList->isEmpty()) {
                STATE& current = this->openList->peek();
                info("state ", current, "popped from open list f=", current.getF(), "g=", current.getG(), "h=", current.getH());

                if (this->goalChecker.isGoal(current, expectedGoal)) {
                    info("state ", current, "is a goal!");
                    goal = &current;

                    this->fireEvent([&current](Listener& l) { l.onSolutionFound(current); });
                    goto goal_found;
                }

                this->openList->pop();

                current.markAsExpanded();
                this->fireEvent([&current](Listener& l) { l.onNodeExpanded(current); });

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
                        this->fireEvent([&successor](Listener& l) { l.onStartingComputingHeuristic(successor); });
                        cost_t hval = this->heuristic.getHeuristic(successor, expectedGoal);
                        this->fireEvent([&successor](Listener& l) { l.onEndingComputingHeuristic(successor); });
                        successor.setG(gval);
                        successor.setH(hval);
                        successor.setF(this->computeF(gval, hval));
                        successor.setParent(&current);

                        this->fireEvent([&current, &successor](Listener& l) {l.onNodeGenerated(successor); });
                        info("child", successor, "of state ", current, "not present in open list. Add it f=", successor.getF(), "g=", successor.getG(), "h=", successor.getH());
                        this->openList->push(successor);
                        
                        
                    }
                }
            }
            info("found no solutions!");
            throw SolutionNotFoundException{};

            goal_found:
            return *goal;

        }

    };

}

#endif