#include "GridMapBlockList.hpp"

namespace pathfinding::search {

template <maps::GridBranching GRIDBRANCHING>
GridMapBlockList<GRIDBRANCHING>::GridMapBlockList(ucood_t mapWidth, ucood_t mapHeight) : blocks_{nullptr}, pool_{nullptr} {
	num_blocks_ = ((mapWidth*mapHeight) >> LOG2_NBS) + 1;
	blocks_ = new GridMapState**[num_blocks_];
	for(uint32_t i=0; i < num_blocks_; i++) {
		blocks_[i] = nullptr;
	}
	//this->blockspool_ = new cpp_utils::cpool{sizeof(void*)*warthog::blocklist_ns::NBS, 1};
    //this->pool_ = new warthog::mem::cpool(sizeof(warthog::search_node));
    this->blockspool_ = new cpp_utils::cpool<void*[NBS]>{1};
	this->pool_ = new cpp_utils::cpool<GridMapState>{};
}

template <maps::GridBranching GRIDBRANCHING>
GridMapBlockList<GRIDBRANCHING>::~GridMapBlockList() {
	this->clear();
	delete pool_;
	delete blockspool_;
}

template <maps::GridBranching GRIDBRANCHING>
GridMapState& GridMapBlockList<GRIDBRANCHING>::generate(stateid_t stateId, xyLoc loc) {
    //e.g., 65 block_id = 1 listid = 1
	uint32_t block_id = stateId >> LOG2_NBS;
	uint32_t list_id = stateId & NBS_MASK;
	assert(block_id <= this->num_blocks_);
    GridMapState* result = nullptr;

	if (blocks_[block_id] == nullptr) {
		// add a new block of nodes
		debug("generating block: ", block_id);
		GridMapState** stateArray = new (this->blockspool_->allocate()) GridMapState*[NBS];
		for(uint32_t listIdInvolved= 0; listIdInvolved < NBS; listIdInvolved += maps::GridBranchingMethods::getBranching<GRIDBRANCHING>()) {
            for (int branching=0; branching<maps::GridBranchingMethods::getBranching<GRIDBRANCHING>(); ++branching) {
			    stateArray[listIdInvolved + branching] = nullptr;
            }
		}
		// generate node_id
		GridMapState* result = new (pool_->allocate()) GridMapState{stateId, loc};
		stateArray[list_id] = result;
		this->blocks_[block_id] = stateArray;
	} else {
        // look for node_id in an existing block
        GridMapState* result = blocks_[block_id][list_id];
        if(result == nullptr) {
            // not in any existing block; generate it
            result = new (pool_->allocate()) GridMapState{stateId, loc};
            this->blocks_[block_id][list_id] = result;
        }
    }
    return *result;
}

template <maps::GridBranching GRIDBRANCHING>
void GridMapBlockList<GRIDBRANCHING>::clear() {
	for(uint32_t i=0; i < num_blocks_; i++) {
		if(blocks_[i] != nullptr) {
			blocks_[i] = nullptr;
		}
	}
	pool_->reclaim();
	blockspool_->reclaim();
}

template <maps::GridBranching GRIDBRANCHING>
uint32_t GridMapBlockList<GRIDBRANCHING>::mem() const {
	uint32_t bytes = sizeof(*this) + blockspool_->mem() +
		pool_->mem() + num_blocks_*sizeof(GridMapState**);

	return bytes;
}

}