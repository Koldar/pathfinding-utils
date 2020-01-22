#ifndef _IPATH_HEADER__
#define _IPATH_HEADER__

#include <string>
#include <iostream>

#include <cpp-utils/vectorplus.hpp>
#include <cpp-utils/igraph.hpp>

#include "types.hpp"

namespace pathfinding {

    using namespace cpp_utils;
    using namespace cpp_utils::graphs;

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
        typedef AbstractPath<ELEMENT> This;
    public:
        /**
         * @brief check if this path is empty
         * 
         * An empty path means that if an agent were to follow the path, it wouldn't move by a single bit
         * 
         * @return true 
         * @return false 
         */
        virtual bool isEmpty() const = 0;
        /**
         * @brief get the n-th concept in the path
         * 
         * @param index 
         * @return const ELEMENT& 
         */
        virtual const ELEMENT& at(int index) const = 0;
        /**
         * @brief get the n-th concept in the path
         * 
         * @param index 
         * @return ELEMENT& 
         */
        virtual ELEMENT& at(int index) = 0;
        /**
         * @brief get a subpath
         * 
         * @param start start of the subpath (included). -1 values means we start from the end
         * @param end end fo the subpath (excluded). -1 values means we start from the end
         * @return const cpp_utils::vectorplus<ELEMENT> 
         */
        virtual cpp_utils::vectorplus<ELEMENT> at(int start, int end) const = 0;
        /**
         * @brief number of concepts involved in this path
         * 
         * @return size_t 
         */
        virtual size_t size() const = 0;
    public:
        cpp_utils::vectorplus<ELEMENT> toVector() const {
            return this->at(0, this->size());
        }
        bool hasAtLeastOneElement() const {
            return !this->isEmpty();
        }
        const ELEMENT& operator [](int index) const {
            return this->at(index);
        }
        ELEMENT& operator [](int index) {
            return this->at(index);
        }
        bool startsIn(const ELEMENT& el) const {
            if (this->isEmpty()) {
                return false;
            }
            return (*this)[0] == el;
        }
        bool endsIn(const ELEMENT& el) const {
            if (this->isEmpty()) {
                return false;
            }
            return (*this)[-1] == el;
        }
        /**
         * @brief check if the path passes through a particular concept
         * 
         * @param el the concept involved
         * @return true if the path references such concept,
         * @return false otherwise
         */
        bool passesThrough(const ELEMENT& el) const {
            for (int i=0; i<this->size(); ++i) {
                if (this->at(i) == el) {
                    return true;
                }
            }
            return false;
        }
    public:
        friend std::ostream& operator << (std::ostream& out, const This& path) {
            out << "(size=" << path.size() << ")[";
            for (int i=0; i<path.size(); ++i) {
                out << path.at(i);
                if ((i+1)<path.size()) {
                    out << ", ";
                }
            }
            out << "]";
            return out;
        }
    };

    /**
     * @brief a path over a graph
     * 
     */
    class NodePath : public AbstractPath<nodeid_t>, vectorplus<nodeid_t> {
    public:
        typedef NodePath This;
        typedef AbstractPath<nodeid_t> Super1;
        typedef vectorplus<nodeid_t> Super2;
    public:
        explicit NodePath(): Super1{}, Super2{} {

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

    /**
     * @brief a path over a graph which use edge movement to determine the path
     * 
     */
    class MovePath: public AbstractPath<moveid_t>, vectorplus<moveid_t> {

    };

    /**
     * @brief a path over a graph which use edge to determine the path
     * 
     * @tparam E type of each label associated to an edge
     */
    template <typename E>
    class EdgePath: public AbstractPath<Edge<E>>, vectorplus<Edge<E>> {

    };
}

#endif 