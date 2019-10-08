#ifndef _ISTATESUPPLIER_HEADER__
#define _ISTATESUPPLIER_HEADER__

#include "ISearchState.hpp"
#include <cpp-utils/pool.hpp>
#include <cpp-utils/imemory.hpp>
#include <cpp-utils/ICleanable.hpp>
#include <memory>

namespace pathfinding::search {

template <typename STATE, typename... STATE_IMPORTANT_TYPES>
class IStateSupplier;

/**
 * @brief a class whose job is to generate a new search state
 * 
 * @tparam STATE the IState implementation we will yield
 * @tparam STATE_IMPORTANT_TYPES additional parameters required to generate a state
 */
template <typename STATE, typename... STATE_IMPORTANT_TYPES>
class IStateSupplier: public ICleanable, IMemorable {
public:
	/**
	 * @brief get a search state without providing an id
	 * 
	 * This will likely to generate a new state almost always
	 * 
	 * @param args additional information required to generate states
	 * @return STATE& the new state
	 */
	virtual STATE& getState(STATE_IMPORTANT_TYPES... args) = 0;
};

// template<typename STATE, typename... STATE_IMPORTANT_TYPES>
// class AbstractStateSupplier: public IStateSupplier<STATE, STATE_IMPORTANT_TYPES...> {
// private:
//     cpp_utils::cpool<STATE> statePool;
//     std::vector<STATE*> stateIdToState;
//     stateid_t nextId;
// public:
//     AbstractStateSupplier(size_t maxStates = 10000): statePool{20}, stateIdToState{maxStates}, nextId{0} {
// 		std::fill(stateIdToState.begin(), stateIdToState.end(), nullptr);
//     }
//     AbstractStateSupplier(const AbstractStateSupplier& other) = delete;
//     ~AbstractStateSupplier() {
//     }
// 	AbstractStateSupplier& operator =(const AbstractStateSupplier& other) = delete;
// 	AbstractStateSupplier(AbstractStateSupplier&& other): statePool{::std::move(other.statePool)}, stateIdToState{::std::move(other.stateIdToState)}, nextId{other.nextId} {

// 	}
// 	AbstractStateSupplier& operator =(AbstractStateSupplier&& other) {
// 		this->statePool = ::std::move(other.statePool);
// 		this->stateIdToState = ::std::move(other.stateIdToState);
// 		this->nextId = other.nextId;
// 		return *this;
// 	}
// protected:
//     /**
// 	 * @brief generates a new state
// 	 * 
// 	 * @param id id of the state to generate
// 	 * @param args additional information to build the state
// 	 * @return STATE a new state
// 	 */
// 	virtual std::unique_ptr<STATE> fetchNewInstance(stateid_t id, STATE_IMPORTANT_TYPES... args) = 0;
// public:
//     virtual STATE& getState(STATE_IMPORTANT_TYPES... args) {
// 		nextId += 1;
// 		auto id = nextId - 1;

// 		debug("requesting state id=", id);
// 		if (id >= stateIdToState.size()) {
// 			debug("increase state index...");
// 			stateIdToState.resize(2*stateIdToState.size());
// 		}
// 		if (stateIdToState[id] == nullptr) {
// 			info("allocating new space for state", id, "...");
// 			stateIdToState[id] = new (this->statePool.allocate()) STATE{*fetchNewInstance(id, args...)};
// 		}
// 		debug("yielding id", id, "(", stateIdToState[id], ")");
// 		return *stateIdToState[id];
//     }
//     virtual void cleanup() {
//         nextId = 0;
// 		std::fill(stateIdToState.begin(), stateIdToState.end(), nullptr);
//         this->statePool.reclaim();
//     }
// public:
// 	virtual MemoryConsumption getByteMemoryOccupied() const {
// 		//TODO update this
//         return 
// 			MemoryConsumption{sizeof(*this), MemoryConsumptionEnum::BYTE};
//     }
// };

}

#endif