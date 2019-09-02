#ifndef _IPATH_HEADER__
#define _IPATH_HEADER__

#include <string>
#include <iostream>
#include <cpp-utils/vectorplus.hpp>

using namespace cpp_utils;

namespace pathfinding {

/**
 * @brief a type identifying a move (following the edge from its source to its sink) in the graph
 * 
 */
typedef u_int64_t moveid_t;

/**
 * @brief An agent path.
 * 
 * paths can either be over nodes in the graph or by movements
 * 
 * For example a path can be defined by the nodes of the underlying graph we want to travel over or by the movement we want to have
 * 
 */
template<typename ELEMENT>
class AbstractPath {
public:
    bool isEmpty() const = 0;
    const ELEMENT& at(int index) const = 0;
    ELEMENT& at(int index) = 0;
    const std::vector<ELEMENT> at(int start, int end) const = 0;
    size_t size() const = 0;
    
    std::ostream& operator << (std::ostream& out, const AbstractPath<>& path) = 0;
public:
    bool hasAtLeastOneElement() const {
        return !this->isEmpty();
    }
    const ELEMENT& operator [](int index) const {
        return this->at(index);
    }
    ELEMENT& operator [](int index) {
        return this->at(index);
    }
    const std::vector<ELEMENT> at(int start, int end) const {
        return this->at(start, end);
    }
    bool startsIn(const ELEMENT& el) const {
        if (this->isEmpty()) {
            return false;
        }
        return path[0] == el;
    }
    bool endsIn(const ELEMENT& el) const {
        if (this->isEmpty()) {
            return false;
        }
        return path[-1] == el;
    }
}

/**
 * @brief a path over a graph
 * 
 */
class NodePath : AbstractPath<node_id>, vectorplus<node_id> {    
};

/**
 * @brief a path over a graph which use edge movement to determine the path
 * 
 */
class MovePath: AbstractPath<moveid_t>, vectorplus<moveid_t> {

};

};

#endif 