#ifndef _GRIDMAPBLOCKLIST_HEADER__
#define _GRIDMAPBLOCKLIST_HEADER__

#include <climits>
#include <cstdint>
#include <cpp-utils/imemory.hpp>
#include <cpp-utils/ICleanable.hpp>
#include "GridBranching.hpp"
#include "GridMapState.hpp"
#include <cpp-utils/math.hpp>

namespace pathfinding::search {

    using namespace pathfinding;

/**
 * @brief 
 * 
 * A heap for allocated warthog::search_node objects.
 * Stores addresses of allocated nodes and also acts as
 * a closed list of sorts.
 * This implementation is specialised for spatial networks.
 * Addresses can be stored in one of several buckets where each
 * bucket represents a cell in a square subdivision of the 
 * spatial net. 
 * The idea here is to improve performance when looking up the
 * addresses of nearby nodes. For example, two nodes may
 * which may be in close spatial proximity should also have
 * their addresses stored in close proximity in memory.
 * This is not the case when using a simple 2d flattened
 * array.
 * 
 * The class works only in grid maps 8-connected in order to exploit such spatial relation among nodes.
 * 
 * @note
 * The class can only work when the state id of a state is the same as the vertex id
 * 
 * @note
 * this class has been copied from Daniel Harabor source code of Warthog and tweaked a little bit.
 * Hence the author name will remain the same.
 *
 * @author: dharabor
 * @created: 02/09/2012
 * 
 */
template <maps::GridBranching GRIDBRANCHING>
class GridMapBlockList: public cpp_utils::ICleanable, public cpp_utils::IMemorable {
public:
    /**
     * @brief node block size
     * 
     * @note
     * this value should be set to something \f$x \leq GRIDBRANCHING \f$, a multiple of GRIDBRANCHING
     * (because GRIDBRANCHING is the maximum number of successors in a GRIDBRANCHING-connected grid map)
     * and a power of 2.
     */
    //TODO refactor to BLOCKS_NUMBER
    static const constexpr uint32_t NBS = pow2GreaterThan(maps::GridBranchingMethods::getBranching<GRIDBRANCHING>() * 8);
    /**
     * @brief \f$log_{2}(NBS)\f$
     * 
     */
	static const constexpr uint32_t LOG2_NBS = log2(NBS);
    /**
     * @brief mask of NBS 
     * 
     * NBS acts as a bit threshold. this mask is obtained by setting all the bits on the right of NBS to 1 and all the
     * bits on the left of NBS (NBS included) as 0. So it's something like `00011111`.
     */
	static const uint32_t NBS_MASK = (0xFFFFFFFF << static_cast<uint32_t>(log2(NBS) + 1) ^ ~NBS); //63;
private:
    /**
     * @brief size of the array ::blocks_
     * 
     */
    uint32_t num_blocks_;
    /**
     * @brief array where at each cell contains an array of GridMapState pointers aggregated by GRIDBRANCHING blocks
     * 
     */
    GridMapState*** blocks_;
    /**
     * @brief pool where the several lists of super sqaures are saved
     * 
     * In each cell of the pool there is an array NBS cell long, containing NBS/max successors number
     * 
     */
    cpp_utils::cpool<GridMapState*[NBS]>* blockspool_;
    /**
     * @brief pool where all the GridMapState are saved
     * 
     */
    cpp_utils::cpool<GridMapState>* pool_;
public:
    /**
     * @brief Construct a new Block List object
     * 
     * @param mapWidth width of the grid map
     * @param mapHeight height of the grid map
     */
    GridMapBlockList(ucood_t mapWidth, ucood_t mapHeight);
    ~GridMapBlockList();

    /**
     * @brief fetch the search state with the given state id or creates a new one from scratch
     * 
     * if the node has already been generated, return a pointer to the 
     * previous instance; otherwise allocate memory for a new object.
     * 
     * @param stateId the id of the search state to generate
     * @param log the location of this grid state
     * @return warthog::search_node* address of the involved state.
     */
    GridMapState& generate(stateid_t stateId, xyLoc loc);

    /**
     * @brief remove all the GridMapState saved up until now
     * 
     */
    void clear();
public:
    virtual void cleanup();
public:
    virtual MemoryConsumption getByteMemoryOccupied() const;

	
};

}

#endif