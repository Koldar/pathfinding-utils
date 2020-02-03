#ifndef _PATHFINDING_UTILS_ABSTRACT_SIMPLE_WEIGHTED_DIRECTED_GRAPH_STATE_SUPPLIER_HEADER__
#define _PATHFINDING_UTILS_ABSTRACT_SIMPLE_WEIGHTED_DIRECTED_GRAPH_STATE_SUPPLIER_HEADER__

#include <cpp-utils/pool.hpp>
#include <cpp-utils/vectorplus.hpp>

namespace pathfinding::search {

    using namespace cpp_utils::graphs;

     /**
     * @brief generates states which are based over a direct weighted graph
     * 
     * This class generates **a state for each node** in a direct weighted graph.
     * Hence there can be only up to N states (where N is the number of vertices in a graph)
     * 
     * @tparam G payload type of the whole graph
     * @tparam V paylaod type of each vertex
     * @tparam STATE_OTHER_IMPORTANT_TYPES additional types necessary (along the nodeid_t) to generate a new state
     */
    template <typename STATE, typename G, typename V, typename E, typename REASON, typename... STATE_OTHER_IMPORTANT_TYPES>
    class AbstractSimpleWeightedDirectedGraphStateSupplier: public IStateSupplier<STATE, nodeid_t, REASON, STATE_OTHER_IMPORTANT_TYPES...> {
        using This =  AbstractSimpleWeightedDirectedGraphStateSupplier<STATE,G, V, E, STATE_OTHER_IMPORTANT_TYPES...>;
    protected:
        /**
         * @brief heap memory where the states are concretely saved
         * 
         */
        cpp_utils::cpool<STATE>* statePool;
        /**
         * @brief vector containing as many cell as vertices in the graph, a pointer where the state is
         * 
         * If the state is not present, this vector contains @c nullptr
         * 
         */
        cpp_utils::vectorplus<STATE*> statePointers;
        /**
         * @brief the graph that will be used to construct GraphState
         * 
         */
        const IImmutableGraph<G, V, E>& graph;
    public:
        /**
         * @brief Construct a new Graph State Supplier object
         * 
         * @param graph the graph that will be put in ever state generated in this structure
         */
        AbstractSimpleWeightedDirectedGraphStateSupplier(const IImmutableGraph<G, V, E>& graph): graph{graph}, statePointers{graph.numberOfVertices(), nullptr}, statePool{nullptr} {
            this->statePool = new cpp_utils::cpool<STATE>{graph.numberOfVertices()};
        }
        virtual ~AbstractSimpleWeightedDirectedGraphStateSupplier() {
            debug("AbstractSimpleWeightedDirectedGraphStateSupplier destroyed!");
            //this->cleanup();
            debug("AbstractSimpleWeightedDirectedGraphStateSupplier cleanup called!");
            delete this->statePool;
        }
        AbstractSimpleWeightedDirectedGraphStateSupplier(const This& other) = delete;
        AbstractSimpleWeightedDirectedGraphStateSupplier(This&& other) : statePointers{::std::move(other.statePointers)}, statePool{::std::move(other.statePool)}, graph{other.graph} {
            debug("AbstractSimpleWeightedDirectedGraphStateSupplier MOVED");
        }
        This& operator = (const This& other) = delete;
        This& operator = (This&& other) {
            debug("AbstractSimpleWeightedDirectedGraphStateSupplier MOVED");
            this->statePointers = ::std::move(other.statePointers);
            this->statePool = other.statePool;
            this->graph = other.graph;

            other.statePool = nullptr;

            return *this;
        }
    protected:
        /**
         * @brief generate the state id of a new state to generate
         * 
         * @param location location of the state to generate
         * @return stateid_t the stateid_t this newly generated state will have
         */
        virtual stateid_t generateStateId(nodeid_t location, const REASON& reason, const STATE_OTHER_IMPORTANT_TYPES&... args) = 0;

        virtual STATE generateNewInstance(stateid_t id, nodeid_t location, const REASON& reason, const STATE_OTHER_IMPORTANT_TYPES&... args) = 0;
    public:
        virtual STATE& getState(const nodeid_t& location, const REASON& reason, const STATE_OTHER_IMPORTANT_TYPES&... args) {
            finest("location is ", location);
            //let's check if we have the requested timestamp in fromTimestampToLocationMap
            if (this->statePointers[location] == nullptr) {
                stateid_t newId = this->generateStateId(location, reason, args...);
                this->statePointers[location] = new (this->statePool->allocate()) STATE{generateNewInstance(newId, location, args...)};
            }
            return *this->statePointers[location];
        }
    public:
        virtual void cleanup() {
            this->statePool->cleanup();
            this->statePointers.fill(nullptr);
        }
    public:
        virtual MemoryConsumption getByteMemoryOccupied() const {
            throw cpp_utils::exceptions::NotYetImplementedException{""};
        }
    };

    

}

#endif 