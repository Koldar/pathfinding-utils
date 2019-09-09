#ifndef _ISTATESUPPLIER_HEADER__
#define _ISTATESUPPLIER_HEADER__

#include "IState.hpp"
#include <cpp-utils/pool.hpp>
#include <cpp-utils/imemory.hpp>
#include <cpp-utils/ICleanable.hpp>
#include <memory>

namespace pathfinding::search {

template <typename STATE, typename... OTHER>
class IStateSupplier;

/**
 * @brief a class whose job is to generate a new search state
 * 
 * @tparam STATE the IState implementation we will yield
 * @tparam OTHER additional parameters required to generate a state
 */
template <typename STATE, typename... OTHER>
class IStateSupplier: public ICleanable, IMemorable {
public:
	/**
	 * @brief get a search state with the given id
	 * 
	 * this function should generate a new IState instance if no state with `id` has never been generated or
	 * the previously existent state with the given `id` otherwise
	 * 
	 * @param id the id of the state which uniquely identifies the state
     * @param args additional information required to generate states
	 * @return IState& the state with such id
	 */
	virtual STATE& getState(stateid_t id, OTHER... args) = 0;
	/**
	 * @brief get a search state without providing an id
	 * 
	 * This will likely to generate a new state almost always
	 * 
	 * @param args additional information required to generate states
	 * @return STATE& the new state
	 */
	virtual STATE& getState(OTHER... args) = 0;
};

template<typename STATE, typename... OTHER>
class AbstractStateSupplier: public IStateSupplier<STATE, OTHER...> {
private:
    cpp_utils::cpool<STATE> statePool;
    std::vector<STATE*> stateIdToState;
    stateid_t nextId;
public:
    AbstractStateSupplier(size_t maxStates = 10000): statePool{20}, stateIdToState{maxStates}, nextId{0} {
		std::fill(stateIdToState.begin(), stateIdToState.end(), nullptr);
    }
    AbstractStateSupplier(const AbstractStateSupplier& other) = delete;
    ~AbstractStateSupplier() {
    }
protected:
    /**
	 * @brief generates a new state
	 * 
	 * @param id id of the state to generate
	 * @param args additional information to build the state
	 * @return STATE a new state
	 */
	virtual std::unique_ptr<STATE> fetchNewInstance(stateid_t id, OTHER... args) = 0;
public:
    virtual STATE& getState(stateid_t id, OTHER... args) {
		debug("requesting state id=", id);
		if (id >= stateIdToState.size()) {
			debug("increase state index...");
			stateIdToState.resize(2*stateIdToState.size());
		}
		if (stateIdToState[id] == nullptr) {
			info("allocating new space for state", id, "...");
			stateIdToState[id] = new (this->statePool.allocate()) STATE{*fetchNewInstance(id, args...)};
		}
		debug("yielding id", id, "(", stateIdToState[id], ")");
		return *stateIdToState[id];
    }
    virtual STATE& getState(OTHER... args) {
		nextId += 1;
		return getState(nextId - 1, args...);
    }
    virtual void cleanup() {
        nextId = 0;
		std::fill(stateIdToState.begin(), stateIdToState.end(), nullptr);
        this->statePool.reclaim();
    }
public:
	virtual MemoryConsumption getByteMemoryOccupied() const {
		//TODO update this
        return 
			MemoryConsumption{sizeof(*this), MemoryConsumptionEnum::BYTE};
    }
public:
    AbstractStateSupplier& operator =(const AbstractStateSupplier<STATE>& other) = delete;
};

}

#endif