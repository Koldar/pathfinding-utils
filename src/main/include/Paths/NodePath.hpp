#ifndef _PATHFINDINGUTILS_NODEPATH_HEADER__
#define _PATHFINDINGUTILS_NODEPATH_HEADER__

#include <cpp-utils/vectorplus.hpp>

#include "IPath.hpp"

namespace pathfinding {
    
    using namespace cpp_utils;
    using namespace cpp_utils::graphs;

    /**
     * @brief a path over a graph
     * 
     * a node path contains all the vertices where the agent was located.
     * Hence if the agent does not move, the nodepath will have size 1. If the agent moves one time, the path will have size 2 and so on.
     */
    class NodePath : public AbstractPath<nodeid_t>, vectorplus<nodeid_t> {
    public:
        using This = NodePath;
        using Super1 = AbstractPath<nodeid_t>;
        using Super2 = vectorplus<nodeid_t>;
    public:
        explicit NodePath(): Super1{}, Super2{} {

        }
        explicit NodePath(const AbstractPath& path): Super1{}, Super2{path.toVector()} {
        }
        NodePath(const Super2& v): Super1{}, Super2{v} {

        }
        virtual ~NodePath() {

        }
        NodePath(const This& o): Super1{o}, Super2{o} {
            debug("call NodePath assignment constructor!");
        }
        NodePath(This&& o): Super1{o}, Super2{o} {
            debug("call NodePath move constructor");
        }
        This& operator =(const This& o) {
            debug("call NodePath copy =");
            Super1::operator =(o);
            Super2::operator =(o);
            return *this;
        }
        This& operator =(This&& o) {
            debug("call Super1 move");
            Super1::operator =(o);
            debug("call Super2 move");
            Super2::operator =(o);
            debug("return *this");
            return *this;
        }
        NodePath asNodePath() const {
            return *this;
        }
    public:
        virtual bool isEmpty() const {
            return Super2::isEmpty();
        }
        /**
         * @brief get the n-th concept in the path
         * 
         * @param index 
         * @return const ELEMENT& 
         */
        virtual const nodeid_t& at(int index) const {
            return Super2::at(index);
        }
        /**
         * @brief get the n-th concept in the path
         * 
         * @param index 
         * @return ELEMENT& 
         */
        virtual nodeid_t& at(int index) {
            return Super2::at(index);
        }
        /**
         * @brief get a subpath
         * 
         * @param start start of the subpath (included). -1 values means we start from the end
         * @param end end fo the subpath (excluded). -1 values means we start from the end
         * @return const cpp_utils::vectorplus<ELEMENT> 
         */
        virtual cpp_utils::vectorplus<nodeid_t> at(int start, int end) const {
            return Super2::at(start, end);
        }
        /**
         * @brief number of concepts involved in this path
         * 
         * @return size_t 
         */
        virtual size_t size() const {
            return Super2::size();
        }
    };

}

#endif