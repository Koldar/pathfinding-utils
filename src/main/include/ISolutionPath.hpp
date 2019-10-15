#ifndef _ISOLUTION_PATH_HEADER__
#define _ISOLUTION_PATH_HEADER__

#include <cpp-utils/vectorplus.hpp>
#include <cpp-utils/igraph.hpp>

namespace pathfinding::search {

    using namespace cpp_utils::graphs;

    /**
     * @brief a vector which represents the solution path generated by A*
     * 
     * @tparam STATE type of state this solution will actually contain
     * @tparam CONST_REF constant reference of a STATE
     */
    template <typename STATE, typename CONST_REF>
    class ISolutionPath : public cpp_utils::vectorplus<STATE> {
    public:
        /**
         * @brief Get the solution cost
         * 
         * @return cost_t 
         */
        virtual cost_t getCost() const = 0;
    public:
        virtual CONST_REF getStart() const {
            return cpp_utils::Types<STATE>::cref(this->at(0));
        }
        virtual CONST_REF getGoal() const {
            return cpp_utils::Types<STATE>::cref(this->at(-1));
        }
    };

    /**
     * @brief A solution which contain search algorithm A* like states
     * 
     * This solution contains only pointers of the actual states
     * 
     * @tparam STATE the type of state inside this solution
     * @tparam CONST_REF constant reference of STATE
     */
    template <typename STATE>
    class StateSolutionPath: public ISolutionPath<const STATE*, const STATE&> {
    public:
        using ISolutionPath<const STATE*, const STATE&>::vectorplus;
        using ISolutionPath<const STATE*, const STATE&>::make;
        using ISolutionPath<const STATE*, const STATE&>::map;
        using ISolutionPath<const STATE*, const STATE&>::addHead;
        using ISolutionPath<const STATE*, const STATE&>::addTail;
        using ISolutionPath<const STATE*, const STATE&>::filter;
    public:
        virtual cost_t getCost() const {
            return this->getGoal().getCost();
        }
    };

    /**
     * @brief A solution which is a sequence of vertices inside a graph
     * 
     * @tparam G payload of the whole graph
     * @tparam V payload of an vertex
     */
    template <typename G, typename V>
    class GraphSolutionPath: public ISolutionPath<cpp_utils::graphs::nodeid_t, cpp_utils::graphs::nodeid_t> {
    public:
        using ISolutionPath<cpp_utils::graphs::nodeid_t, cpp_utils::graphs::nodeid_t>::vectorplus;
        using ISolutionPath<cpp_utils::graphs::nodeid_t, cpp_utils::graphs::nodeid_t>::make;
        using ISolutionPath<cpp_utils::graphs::nodeid_t, cpp_utils::graphs::nodeid_t>::map;
        using ISolutionPath<cpp_utils::graphs::nodeid_t, cpp_utils::graphs::nodeid_t>::addHead;
        using ISolutionPath<cpp_utils::graphs::nodeid_t, cpp_utils::graphs::nodeid_t>::addTail;
        using ISolutionPath<cpp_utils::graphs::nodeid_t, cpp_utils::graphs::nodeid_t>::filter;
    private:
        const cpp_utils::graphs::IImmutableGraph<G, V, cost_t>& g;
    public:
        GraphSolutionPath(const cpp_utils::graphs::IImmutableGraph<G, V, cost_t>& g): g{g} {

        }
    public:
        virtual cost_t getCost() const {
            if (this->size() <= 1) {
                return 0;
            }
            cost_t result = 0;
            for (int i=1; i<this->size(); ++i) {
                result += g.getEdge((*this)[i-1], (*this)[i]);
            }
            return result;
        }
    };

    /**
     * @brief check if a path is valid or not
     * 
     * 
     * @tparam G type of the graph payload
     * @tparam V 
     * @tparam E 
     * @param graph 
     * @param path 
     */
    template <typename STATE, typename CONST_REF, typename G, typename V, typename E>
    void checkPathValid(const IImmutableGraph<G, V, E>& graph, const ISolutionPath<STATE, CONST_REF>& path, std::function<nodeid_t(STATE)> mapper) {

        nodeid_t previous;
        for (int i=0; i<path.size(); ++i) {
            nodeid_t current = mapper(path[i]);
            if (!graph.containsVertex(current)) {
                    critical("node in ",i, " with id=", current, "is not in graph ", graph);
                    throw cpp_utils::exceptions::ImpossibleException{};
                }

            if (i == 0) {
                previous = current;
            } else {
                if (!graph.hasEdge(previous, current)) {
                    critical("there is not an edge from ", previous, " to ", current, "in graph ", graph);
                    throw cpp_utils::exceptions::ImpossibleException{};
                }
                previous = current;
            }
        }
    }

}

#endif