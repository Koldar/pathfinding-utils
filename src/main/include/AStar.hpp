#ifndef _ASTAR_HEADER__
#define _ASTAR_HEADER__

#include "ISearchAlgorithm.hpp"
#include <cpp-utils/StaticPriorityQueue.hpp>
#include <cpp-utils/log.hpp>
#include "IHeuristic.hpp"

namespace pathfinding::search {

/**
 * @brief a class whose job is to generate a new search state
 * 
 */
class IStateSupplier {
	/**
	 * @brief get a search state with the given id
	 * 
	 * this function should generate a new IState instance if no state with `id` has never been generated or
	 * the previously existent state with the given `id` otherwise
	 * 
	 * @param id the id of the state which uniquely identifies the state
	 * @return IState& the state with such id
	 */
	virtual IState& getState(stateid_t id) = 0;
}

/**
 * @brief a class whose job is to compute all the successors of a given state
 * 
 */
class IStateExpander {
public:
    virtual std::vector<std::pair<IState&, cost_t>> getSuccessors(IState& state) = 0;
};

/**
 * @brief a class whose job is to check if a state under analysis should be pruned for search or not
 * 
 */
class IStatePruner {
public:
    virtual bool shouldPrune(IState& state) const = 0;
};

/**
 * @brief Allows you to further refines the behavior of an A* algorithm
 * 
 */
class AstarListener {
public:
	/**
	 * @brief called whenever a new node is expanded from the open list
	 * 
	 * @param node 
	 */
    virtual void onNodeExpanded(const IState& node) = 0;
	/**
	 * @brief called when the start state has been added in the open list
	 * 
	 * @param node 
	 */
    virtual void onInitialNodeAddedInOpenList(const IState& node) = 0;
	/**
	 * @brief called when a state has been just added in the open list
	 * 
	 * @param parent 
	 * @param node 
	 */
    virtual void onNewNodeAddedInOpenList(const IState& parent, const IState& node) = 0;
    virtual void onNodeParentRevised(const IState& node, const IState& oldParent, const IState& newParent) = 0;
    virtual void onSolutionFound() = 0;
    virtual void onOpenListEmpty() = 0;
};

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
template <typename STATE>
class NoCloseListSingleGoalAstar: public ISearchAlgorithm, public IMemorable, public Listenable<AstarListener> {
public:
    AStarAlgorithm(IHeuristic<STATE>& heuristic, IStateSupplier& supplier, IStateExpander& expander, IStatePruner& pruner,  unsigned int openListCapacity = 1024) : Listenable{}, 
        heuristic{heuristic}, supplier{supplier}, expander{expander}, pruner{pruner},
        openList{nullptr} {
            if (!heuristic.isConsistent()) {
                throw cpp_utils::exceptions::InvalidArgumentException{"the heuristic is not consistent!"};
            }
            this->openList = new StaticPriorityQueue<STATE>{openListCapacity, true};
        }

    ~AStarAlgorithm() {
        this->tearDownSearch();
        delete this->openList;
    }
    //the class cannot be copied whatsoever
    AStarAlgorithm(const flexible_astar& other) = delete;
	AStarAlgorithm& operator=(const flexible_astar& other) = delete;

public:
    MemoryConsumption getByteMemoryOccupied() const {
        return
            // memory for the priority quete
            this->openList->getByteMemoryOccupied() + 
            // gridmap size and other stuff needed to expand nodes
            this->expander->getByteMemoryOccupied() +
            // misc
            MemoryConsumption{sizeof(*this), MemoryConsumptionEnum::BYTE};
    }
private:
    IHeuristic<STATE>& heuristic;
    IStateExpander& expander;
	IStateSupplier& supplier;
    IStatePruner& pruner;
    StaticPriorityQueue<STATE>* openList;
protected:
    virtual cost_t computeF(cost_t g, cost_t h) const {
        return g + h;
    }

    virtual const STATE& _search(stateid_t startId, const stateid_t* goalId) {
        if (goalId == nullptr) {
            throw cpp_utils::exceptions::InvalidArgumentException{"goalId must be not null!"};
        }
        info("starting A*! startId = ", startId, "goalId = ", ELVIS(goalId, "none"));

        STATE* goal = nullptr;
        STATE& start = this->supplier.getState(startId);

        start.setG(0);
        start.setH(this->heuristic.getHeuristic(start, goalId));
        start.setF(this->computeF(start.getG(), start.getH()));

        this->openList->push(start);
        this->fireEvent([&current](const AstarListener& l) { l.onNewNodeAddedInOpenList(current); });
        while (!this->openList->isEmpty()) {
            STATE& current = this->openList->peek();

            if (current.getId() == *goalId) {
                goal = &current;
                goto goal_found;
            }

            this->fireEvent([&current](const AstarListener& l) -> { l.onNodeExpanded(current); });
            this->openList->pop();
            info("state popped from open list f=", current.getF(), "g=", current.getG(), "h=", current.getH(), "state=", current);

            current.markAsExpanded();

            for(auto pair: this->expander.getSuccessors(current)) {
                IState& successor = pair.first;
                cost_t current_to_successor_cost = pair.second;

                this->fireEvent([&current, &successor](const AstarListener& l) {l.onSuccessorExpanded(current, successor)});
                if (this->pruner.shouldPrune(successor)) {
                    //skip neighbours already expanded
                    continue;
                }

                if (this->openList->contains(successor)) {
                    //state inside the open list. Check if we need to update the path
                    cost_t gval = current.getG() + current_to_successor_cost;
                    if (gval < successor.getG()) {

                        //update successor information
                        successor.setG(gval);
                        successor.setF(this->computeF(gval, successor.getH()));
                        State& oldParent = successor.getParent();
                        successor.setParent(current);

                        this->openList->decrease_key(successor);

                        this->fireEvent([&current](const AstarListener& l) { l.onNodeParentRevised(successor, oldParent, current); });
                    }
                } else {
                    //state is not present in open list. Add to it
                    cost_t gval = current.getG() + current_to_successor_cost;
                    cost_t hval = this->heuristic.getHeuristic(successor, goalId);
                    successor.setG(gval);
                    successor.setH(hval);
                    successor.setF(this->computeF(gval, hval));
                    successor.setParent(current);

                    this->openList->push(successor);
                    
                    this->fireEvent([&current, &successor](const AstarListener& l) {l.onNewNodeAddedInOpenList(current, successor); })
                }
            }
        }

        throw SolutionNotFoundException{};

        goal_found:
        return goal;

    }

};


    








public:
		flexible_astar(H* heuristic, E* expander)
			: heuristic_(heuristic), expander_(expander)
		{
			open_ = new warthog::pqueue(1024, true);
			verbose_ = false;
            hscale_ = 1.0;
		}

		~flexible_astar()
		{
			cleanup();
			delete open_;
		}

		inline std::stack<uint32_t>
		get_path(uint32_t startid, uint32_t goalid)
		{
			std::stack<uint32_t> path;
			warthog::search_node* goal = search(startid, goalid);
			if(goal)
			{
				// follow backpointers to extract the path
				assert(goal->get_id() == goalid);
				for(warthog::search_node* cur = goal;
						cur != 0;
					    cur = cur->get_parent())
				{
					path.push(cur->get_id());
				}
				assert(path.top() == startid);
			}
			cleanup();
			return path;
		}

		double
		get_length(uint32_t startid, uint32_t goalid)
		{
			warthog::search_node* goal = search(startid, goalid);
			warthog::cost_t len = warthog::INF;
			if(goal)
			{
				assert(goal->get_id() == goalid);
				len = goal->get_g();
			}

#ifndef NDEBUG

			if(verbose_)
			{
				std::stack<warthog::search_node*> path;
				warthog::search_node* current = goal;
				while(current != 0)	
				{
					path.push(current);
					current = current->get_parent();
				}

				while(!path.empty())
				{
					warthog::search_node* n = path.top();
					uint32_t x, y;
					y = (n->get_id() / expander_->mapwidth());
					x = n->get_id() % expander_->mapwidth();
					std::cerr << "final path: ("<<x<<", "<<y<<")...";
					n->print(std::cerr);
					std::cerr << std::endl;
					path.pop();
				}
			}
#endif
			cleanup();
			return len / (double)warthog::ONE;
		}

		inline size_t
		mem()
		{
			size_t bytes = 
				// memory for the priority quete
				open_->mem() + 
				// gridmap size and other stuff needed to expand nodes
				expander_->mem() +
				// misc
				sizeof(*this);
			return bytes;
		}

		inline uint32_t 
		get_nodes_expanded() { return nodes_expanded_; }

		inline uint32_t
		get_nodes_generated() { return nodes_generated_; }

		inline uint32_t
		get_nodes_touched() { return nodes_touched_; }

		inline double
		get_search_time() { return search_time_; }


		inline bool
		get_verbose() { return verbose_; }

		inline void
		set_verbose(bool verbose) { verbose_ = verbose; } 

        inline double
        get_hscale() { return hscale_; } 

        inline void
        set_hscale(double hscale) { hscale_ = hscale; } 



	private:
		H* heuristic_;
		E* expander_;
		warthog::pqueue* open_;
		bool verbose_;
		static uint32_t searchid_;
		uint32_t nodes_expanded_;
		uint32_t nodes_generated_;
		uint32_t nodes_touched_;
		double search_time_;
        double hscale_; // heuristic scaling factor

		// no copy
		flexible_astar(const flexible_astar& other) { } 
		flexible_astar& 
		operator=(const flexible_astar& other) { return *this; }

		warthog::search_node*
		search(uint32_t startid, uint32_t goalid)
		{
			nodes_expanded_ = nodes_generated_ = nodes_touched_ = 0;
			search_time_ = 0;

			warthog::timer mytimer;
			mytimer.start();

			#ifndef NDEBUG
			if(verbose_)
			{
				std::cerr << "search: startid="<<startid<<" goalid=" <<goalid
					<< std::endl;
			}
			#endif

			warthog::problem_instance instance;
			instance.set_goal(goalid);
			instance.set_start(startid);
			instance.set_searchid(searchid_++);

			warthog::search_node* goal = 0;
			warthog::search_node* start = expander_->generate(startid);
			start->reset(instance.get_searchid());
			start->set_g(0);
			start->set_f(heuristic_->h(startid, goalid) * hscale_);
			open_->push(start);

			while(open_->size())
			{
				nodes_touched_++;
				if(open_->peek()->get_id() == goalid)
				{
					#ifndef NDEBUG
					if(verbose_)
					{
						uint32_t x, y;
						warthog::search_node* current = open_->peek();
						y = (current->get_id() / expander_->mapwidth());
						x = current->get_id() % expander_->mapwidth();
						std::cerr << "goal found ("<<x<<", "<<y<<")...";
						current->print(std::cerr);
						std::cerr << std::endl;
					}
					#endif
					goal = open_->peek();
					break;
				}
				nodes_expanded_++;

				warthog::search_node* current = open_->pop();
				#ifndef NDEBUG
				if(verbose_)
				{
					uint32_t x, y;
					y = (current->get_id() / expander_->mapwidth());
					x = current->get_id() % expander_->mapwidth();
					std::cerr << "expanding ("<<x<<", "<<y<<")...";
					current->print(std::cerr);
					std::cerr << std::endl;
				}
				#endif
				current->set_expanded(true); // NB: set this before calling expander_ 
				assert(current->get_expanded());
				expander_->expand(current, &instance);

				warthog::search_node* n = 0;
				warthog::cost_t cost_to_n = warthog::INF;
				for(expander_->first(n, cost_to_n); 
						n != 0;
					   	expander_->next(n, cost_to_n))
				{
					nodes_touched_++;
					if(n->get_expanded())
					{
						// skip neighbours already expanded
						continue;
					}

					if(open_->contains(n))
					{
						// update a node from the fringe
						warthog::cost_t gval = current->get_g() + cost_to_n;
						if(gval < n->get_g())
						{
							n->relax(gval, current);
							open_->decrease_key(n);
							#ifndef NDEBUG
							if(verbose_)
							{
								uint32_t x, y;
								y = (n->get_id() / expander_->mapwidth());
								x = n->get_id() % expander_->mapwidth();
								std::cerr << "  updating ("<<x<<", "<<y<<")...";
								n->print(std::cerr);
								std::cerr << std::endl;
							}
							#endif
						}
						else
						{
							#ifndef NDEBUG
							if(verbose_)
							{
								uint32_t x, y;
								y = (n->get_id() / expander_->mapwidth());
								x = n->get_id() % expander_->mapwidth();
								std::cerr << "  updating ("<<x<<", "<<y<<")...";
								n->print(std::cerr);
								std::cerr << std::endl;
							}
							#endif
						}
					}
					else
					{
						// add a new node to the fringe
						warthog::cost_t gval = current->get_g() + cost_to_n;
						n->set_g(gval);
						n->set_f(gval + heuristic_->h(n->get_id(), goalid) * hscale_);
					   	n->set_parent(current);
						open_->push(n);
						#ifndef NDEBUG
						if(verbose_)
						{
							uint32_t x, y;
							y = (n->get_id() / expander_->mapwidth());
							x = n->get_id() % expander_->mapwidth();
							std::cerr << "  generating ("<<x<<", "<<y<<")...";
							n->print(std::cerr);
							std::cerr << std::endl;
						}
						#endif
						nodes_generated_++;
					}
				}
				#ifndef NDEBUG
				if(verbose_)
				{
					uint32_t x, y;
					y = (current->get_id() / expander_->mapwidth());
					x = current->get_id() % expander_->mapwidth();
					std::cerr <<"closing ("<<x<<", "<<y<<")...";
					current->print(std::cerr);
					std::cerr << std::endl;
			}
			#endif
			}

			mytimer.stop();
			search_time_ = mytimer.elapsed_time_micro();
			return goal;
		}

		void
		cleanup()
		{
			open_->clear();
			expander_->clear();
		}

};


}

#endif