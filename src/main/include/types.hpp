#ifndef PATHFINDINGUTILS_TYPES_HEADER__
#define PATHFINDINGUTILS_TYPES_HEADER__

//#include <cpp-utils/wrapped_number.hpp>
#include <cpp-utils/safe_inf_uint.hpp>
#include <cpp-utils/fractional_number.hpp>

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

    /**
     * @brief alias for ::fractional_number<cost_t>
     * 
     */
    using fractional_cost = cpp_utils::fractional_number<cost_t>;

    //TODO remove using cost_t = cpp_utils::wrapped_number<u_int64_t, 0x0, 0x0FFFFFFFFFFFFFFF>;
    /**
     * @brief a number representing an id of a state
     * 
     */
    using stateid_t = u_int64_t;

    /**
     * @brief represents a function which convert a type into a cost_t
     * 
     * Useful when dealing with general maps
     * 
     * @tparam E the type of the label of an edge
     */
    template <typename E>
    using costFunction_t = std::function<cost_t(const E&)>;

}

#endif
