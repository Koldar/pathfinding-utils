#ifndef _PATHFINDING_UTILS_WASTAR_HEADER__
#define _PATHFINDING_UTILS_WASTAR_HEADER__

#include <cmath>

#include <cpp-utils/StaticPriorityQueue.hpp>
#include <cpp-utils/log.hpp>
#include <cpp-utils/listeners.hpp>
#include <cpp-utils/imemory.hpp>
#include <cpp-utils/commons.hpp>

#include "IHeuristic.hpp"
#include "ISearchAlgorithm.hpp"
#include "IStateExpander.hpp"
#include "IStatePruner.hpp"
#include "IStateSupplier.hpp"
#include "IGoalChecker.hpp"
#include "AStar.hpp"

namespace pathfinding::search {

    using namespace cpp_utils;

    /**
     * @brief Allows you to further refines the behavior of an A* algorithm
     * 
     */
    template <typename STATE>
    class WAstarListener: public AstarListener<STATE> {
    };

    /**
     * @brief an impementation of WA*
     * 
     * The class has been inspired by Daniel Harabor implementation.
     * 
     * This WA* star algorithm is perfect in the following case:
     *  - the heuristic is consistent (no need for close list);
     *  - the goal can be represented by a single state (goal known apriori): for example
     *      in single agent pathfinding this is ensured while in classical planning this is not.
     * 
     * WA* implementation that allows arbitrary combinations of 
     * (weighted) heuristic functions and node expansion policies.
     * This implementation uses a binary heap for the open_list
     * and a bit array for the closed_list (under the form of `expanded` flag in ::IState)
     * 
     * @author: dharabor
     * @created: 21/08/2012
     */
    template <typename STATE, typename... STATE_IMPORTANT_TYPES>
    class NoCloseListSingleGoalWAstar: public IMemorable, public ISearchAlgorithm<STATE, const STATE*, const STATE&>, public ISingleListenable<WAstarListener<STATE>> {
        using This = NoCloseListSingleGoalWAstar<STATE, STATE_IMPORTANT_TYPES...>;
    public:
        NoCloseListSingleGoalWAstar(double weight, IHeuristic<STATE>& heuristic, IGoalChecker<STATE>& goalChecker, IStateSupplier<STATE, STATE_IMPORTANT_TYPES...>& supplier, IStateExpander<STATE, STATE_IMPORTANT_TYPES...>& expander, IStatePruner<STATE>& pruner,  unsigned int openListCapacity = 1024) : 
            ISingleListenable<WAstarListener<STATE>>{}, 
            weight{weight},
            heuristic{heuristic}, goalChecker{goalChecker}, supplier{supplier}, expander{expander}, pruner{pruner},
            openList{nullptr} {
                if (!heuristic.isConsistent()) {
                    throw cpp_utils::exceptions::InvalidArgumentException{"the heuristic is not consistent!"};
                }
                this->openList = new StaticPriorityQueue<STATE>{openListCapacity, true};
                getRatioOf(weight, this->nweight, this->dweight, 1e-6, 5);
            }

        virtual ~NoCloseListSingleGoalWAstar() {
            this->tearDownSearch();
            delete this->openList;
        }
        //the class cannot be copied whatsoever
        NoCloseListSingleGoalWAstar(const This& other) = delete;
        NoCloseListSingleGoalWAstar& operator=(const This& other) = delete;

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
        double weight;
        /**
         * @brief numerator of the fraction representing the weight
         * 
         */
        cost_t nweight;
        /**
         * @brief denominator of the fraction representing the weight
         * 
         */
        cost_t dweight;
        IHeuristic<STATE>& heuristic;
        IGoalChecker<STATE>& goalChecker;
        IStateExpander<STATE, STATE_IMPORTANT_TYPES...>& expander;
        IStateSupplier<STATE, STATE_IMPORTANT_TYPES...>& supplier;
        IStatePruner<STATE>& pruner;
        StaticPriorityQueue<STATE>* openList;
    protected:
        virtual cost_t computeF(cost_t g, cost_t h) const {
            //weight is double. If rounding is applied we need to make sure that the rounding is on the floor,
            //to ensure the w-admissibility of the algorithm
            return floor(static_cast<double>(g) + this->weight * static_cast<double>(h));
        }
    public:
        virtual std::string getName() const {
            return "WA*";
        }
        virtual void setupSearch(const STATE* start, const STATE* goal) {
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
            this->fireEvent([&start](WAstarListener<STATE>& l) { l.onStartingComputingHeuristic(start); });
            start.setH(this->heuristic.getHeuristic(start, expectedGoal));
            this->fireEvent([&start](WAstarListener<STATE>& l) { l.onEndingComputingHeuristic(start); });
            start.setF(this->computeF(start.getG(), start.getH()));

            this->openList->push(start);
            while (!this->openList->isEmpty()) {
                STATE& current = this->openList->peek();
                info("state ", current, "popped from open list f=", current.getF(), "g=", current.getG(), "h=", current.getH());

                if (this->goalChecker.isGoal(current, expectedGoal)) {
                    info("state ", current, "is a goal!");
                    goal = &current;
                    goto goal_found;
                }

                this->openList->pop();

                current.markAsExpanded();
                this->fireEvent([&current](WAstarListener<STATE>& l) { l.onNodeExpanded(current); });

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
                        this->fireEvent([&successor](WAstarListener<STATE>& l) { l.onStartingComputingHeuristic(successor); });
                        cost_t hval = this->heuristic.getHeuristic(successor, expectedGoal);
                        this->fireEvent([&successor](WAstarListener<STATE>& l) { l.onEndingComputingHeuristic(successor); });
                        successor.setG(gval);
                        successor.setH(hval);
                        successor.setF(this->computeF(gval, hval));
                        successor.setParent(&current);

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