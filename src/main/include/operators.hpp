#ifndef _PATHFINDING_UTILS_OPERATORS_HEADER__
#define _PATHFINDING_UTILS_OPERATORS_HEADER__

#include <cpp-utils/exceptions.hpp>

#include "types.hpp"

namespace pathfinding {

    //TODO remove superseded by std::function. Remember to create a local variable of type std::function if you want to pass a lambda
    /**
     * @brief Generic template function used to fetch a `cost_t` from the label of an edge
     * 
     * Specialize this for the type you wish to use. Here's an example of specialization:
     * 
     * @code
     * namespace pathfinding {
     *  template <>
     *  struct GetCost<MyAwesomeType> {
     *      cost_t operator() (const MyAwesomeType& label) const {
     *          return label.getCost();
     *      }
     *  };
     * }
     * @endcode
     * 
     * The specialization is required to be put in pathfinding namespace
     * 
     * @tparam E edge label type
     */
    template <typename E>
    struct GetCost {
        cost_t operator ()(const E& label) {
            throw cpp_utils::exceptions::NotYetImplementedException{"You need to specialize GetCost with your custom data"};
        }
    };

    template <>
    struct GetCost<cost_t> {
        cost_t operator ()(const cost_t& label) const {
            return label;
        }
    };


}

#endif