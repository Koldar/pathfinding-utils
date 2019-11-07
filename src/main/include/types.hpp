#ifndef PATHFINDINGUTILS_TYPES_HEADER__
#define PATHFINDINGUTILS_TYPES_HEADER__

//#include <cpp-utils/wrapped_number.hpp>
#include <cpp-utils/safe_inf_uint.hpp>

namespace pathfinding {

    /**
     * @brief represents a coordinate inside a system which has to be at least 0
     * 
     */
    typedef uint64_t ucood_t;

    /**
     * @brief represents a coordinate inside a system which can be negative
     * 
     */
    typedef int64_t cood_t;

    /**
     * @brief a number representing a search cost
     * 
     */
    using cost_t = cpp_utils::safe_inf_uint;
    //TODO remove using cost_t = cpp_utils::wrapped_number<u_int64_t, 0x0, 0x0FFFFFFFFFFFFFFF>;
    /**
     * @brief a number representing an id of a state
     * 
     */
    using stateid_t = u_int64_t;

}

#endif
