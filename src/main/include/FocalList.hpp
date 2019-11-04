#ifndef _PATH_FINDING_UTILS_FOCAL_LIST_HEADER__
#define _PATH_FINDING_FOCAL_LIST_HEADER__

#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>

#include <cpp-utils/operators.hpp>

#include "types.hpp"

namespace pathfinding::data_structures {

    namespace internal {

        template <typename ITEM>
        using BinaryHeap = typename boost::heap::d_ary_heap<
            ITEM, 
            boost::heap::arity<2>, 
            boost::heap::mutable_<true>
            //implicitly "<" of HeapNode
        >;

        template <typename ITEM>
        using binary_heap_handle_t = typename BinaryHeap<ITEM>::handle_type;

    }

    namespace internal::open {

        template <typename ITEM, typename COMPARATOR>
        class OpenHeapNode {
            using This = OpenHeapNode<ITEM, COMPARATOR>;
        public:
            binary_heap_handle_t<This> priority; 
            ITEM* data;
        public:
            OpenHeapNode(ITEM* data): data{data}, priority{} {

            }
            virtual ~OpenHeapNode() {
                //we don't own the object "data", so we don't free it
            }
            OpenHeapNode(const This& o): data{o.data}, priority{o.priority} {

            }
            OpenHeapNode(This&& o): data{o.data}, priority{::std::move(o.priority)} {
                o.data = nullptr;
            }
            This& operator=(const This& o) {
                this->data = o.data;
                this->priority = o.priority;
                return *this;
            }
            This& operator=(This&& o) {
                this->data = o.data;
                o.data = nullptr;
                this->priority = ::std::move(o.priority);
                return *this;
            }
        public:
            friend bool operator <(const This& a, const This& b) {
                //boost queue are max-queue, but I want min-queue
                return COMPARATOR()(*(b.data), *(a.data));
            }
        };


    }

    namespace internal::focal {

        template <typename ITEM, typename COMPARATOR>
        class FocalHeapNode {
            using This = FocalHeapNode<ITEM, COMPARATOR>;
        public:
            ::pathfinding::data_structures::internal::binary_heap_handle_t<ITEM> openHandle;
        public:
            FocalHeapNode(::pathfinding::data_structures::internal::binary_heap_handle_t<ITEM> openHandle) : openHandle{openHandle} {

            }
            virtual ~FocalHeapNode() {

            }
            FocalHeapNode(const This& o): openHandle{o.openHandle} {

            }
            FocalHeapNode(This&& o): openHandle{::std::move(o.openHandle)} {

            }
            This& operator=(const This& o) {
                this->openHandle = o.openHandle;
                return *this;
            }
            This& operator=(This&& o) {
                this->openHandle = ::std::move(o.openHandle);
                return *this;
            }
        public:
            friend bool operator <(const FocalHeapNode&a , const FocalHeapNode& b) {
                //boost queue are max-queue, but I want min-queue
                return COMPARATOR()(*((*(b.openHandle)).data), *((*(a.openHandle)).data));
            }
        };
    }


    /**
     * @brief 
     * 
     * @code
     * struct OpenComparator {
     *  bool operator() (const Foo& a, const Foo& b) {
     *   //return true if a should be put nearer to top than b
     *  }
     * };
     * 
     * struct FocalComparator {
     *  bool operator() (const Foo& a, const Foo& b) {
     *   //return true if a should be put nearer to top than b
     *  }
     * };
     * @endcode
     * 
     * @tparam ITEM the type of the items we need t store. Notice that we will always store the **poiters** of the items, not the actual items
     * @tparam OPEN_COMPARATOR a function used to sort the items in the open list
     * @tparam FOCUS_COMPARATOR a function used to sort the item in the focal list
     * @tparam GET_COST a function used to retrieve a cost from ITEM
     * @tparam COST_TYPE the return value of GET_COST. Defaults to `cost_t`
     */
    template <typename ITEM, typename OPEN_COMPARATOR, typename FOCAL_COMPARATOR, typename GET_COST, typename COST_TYPE = cost_t>
    class FocalList {
    private:
        using This = FocalList<ITEM, OPEN_COMPARATOR, FOCAL_COMPARATOR, GET_COST, COST_TYPE>;
    private:
        std::unordered_map<ITEM*, internal::binary_heap_handle_t<internal::open::OpenHeapNode<ITEM, OPEN_COMPARATOR>>> stateToOpenQueue;
        internal::BinaryHeap<internal::open::OpenHeapNode<ITEM, OPEN_COMPARATOR>> openQueue;
        internal::BinaryHeap<internal::focal::FocalHeapNode<internal::open::OpenHeapNode<ITEM, OPEN_COMPARATOR>, FOCAL_COMPARATOR>> focalQueue;
        COST_TYPE w;
    public:
        FocalList(COST_TYPE w): openQueue{}, focalQueue{}, w{w}, stateToOpenQueue{} {

        }
        virtual ~FocalList() {

        }
        FocalList(const This& o): w{o.w}, openQueue{o.openQueue}, focalQueue{o.focalQueue}, stateToOpenQueue{o.stateToOpenQueue} {

        }
        FocalList(This&& o): w{o.w}, openQueue{::std::move(o.openQueue)}, focalQueue{::std::move(o.focalQueue)}, stateToOpenQueue{::std::move(o.stateToOpenQueue)} {

        }
        This& operator=(const This& o) {
            this->openQueue = o.openQueue;
            this->focalQueue = o.focalQueue;
            this->w = o.w;
            this->stateToOpenQueue = o.stateToOpenQueue;
            return *this;
        }
        This& operator=(This&& o) {
            this->openQueue = ::std::move(o.openQueue);
            this->focalQueue = ::std::move(o.focalQueue);
            this->w = o.w;
            this->stateToOpenQueue = ::std::move(o.stateToOpenQueue);
            return *this;
        }
    public:
        cost_t getW() const {
            return this->w;
        }
        /**
         * @brief Check if the open list is empty
         * 
         * @return true 
         * @return false 
         */
        virtual bool isOpenEmpty() const {
            return this->openQueue.empty();
        }

        virtual bool isFocalEmpty() const {
            return this->focalQueue.empty();
        }

        virtual void decreaseOpenListKey(ITEM& val) {
            auto handle = this->stateToOpenQueue[&val];
            this->openQueue.decrease(handle);
        }

        /**
         * @brief let the item to be put only in open but not in focal
         * 
         * @param val the item to be put. We will put the **pointer**, not the item itself
         */
        virtual void pushInOpen(ITEM& val) {
            auto handle = this->openQueue.push(internal::open::OpenHeapNode<ITEM, OPEN_COMPARATOR>{&val});
            (*handle).priority = handle;
            this->stateToOpenQueue.insert(std::make_pair<>(&val, handle));
        }

        /**
         * @brief promote the item which is in open to be in focal as well
         * 
         * @param val 
         */
        virtual void promoteToFocal(ITEM& val) {
            internal::binary_heap_handle_t<internal::open::OpenHeapNode<ITEM, OPEN_COMPARATOR>> handle = this->stateToOpenQueue[const_cast<ITEM*>(&val)];
            this->focalQueue.push(internal::focal::FocalHeapNode<internal::open::OpenHeapNode<ITEM, OPEN_COMPARATOR>, FOCAL_COMPARATOR>{handle});
        }

        /**
         * @brief let the item to be put in open **and** in focal
         * 
         * @param val the item to be put. We will put the **pointer**, not the item itself
         */
        virtual void pushInOpenAndInFocal(ITEM& val) {
            auto handle = this->openQueue.push(internal::open::OpenHeapNode<ITEM, OPEN_COMPARATOR>{&val});
            (*handle).priority = handle;
            this->stateToOpenQueue.insert(std::make_pair<>(&val, handle));
            this->focalQueue.push(internal::focal::FocalHeapNode<internal::open::OpenHeapNode<ITEM, OPEN_COMPARATOR>, FOCAL_COMPARATOR>{handle});
        }

        virtual COST_TYPE updateFocalWithOpenChanges(COST_TYPE bestOpenListScore) {
            COST_TYPE oldBestOpenListScore = bestOpenListScore;
            COST_TYPE newBestOpenListScore = GET_COST()(*this->openQueue.top().data);

            if (newBestOpenListScore > oldBestOpenListScore) {
                //do something only if the best value in open has increased. Otherwise the focal list is unaltered for sure

                for (auto iter = this->openQueue.ordered_begin(); iter != this->openQueue.ordered_end(); ++iter) {
                    COST_TYPE scoreOfStateInOpen = GET_COST()(*(iter->data));

                    /*
                     * (scoreOfStateInOpen > this->w * oldBestOpenListScore) means that the state was not in focal a tthe last iteration
                     * (scoreOfStateInOpen <= this->w * newBestOpenListScore) means that the state should now be considered in focal at the last iteration
                     */
                    if ((scoreOfStateInOpen > this->w * oldBestOpenListScore) && (scoreOfStateInOpen <= this->w * newBestOpenListScore)) {
                        this->focalQueue.push(internal::focal::FocalHeapNode<internal::open::OpenHeapNode<ITEM, OPEN_COMPARATOR>, FOCAL_COMPARATOR>{iter->priority});
                    }
                    //states are ordered in openQueue. as soon as the scoreOfStateInOpen goes beyond the scope of focal, we stop
                    //since we are sure that no state can be stored in focal
                    if (scoreOfStateInOpen > this->w * newBestOpenListScore) {
                        break;
                    }
                }
            }

            return newBestOpenListScore;
        }

        virtual ITEM& peekFromFocal() {
            auto focalNode = this->focalQueue.top();
            return *(*(focalNode.openHandle)).data;
        }

        /**
         * @brief remove from the queue the "best" element
         * 
         * The item will be removed from open **and** from focal
         * 
         * @return ITEM&  the "best" element so far
         */
        virtual ITEM& popFromFocal() {
            if (this->focalQueue.empty()) {
                log_error("focal list is empty!");
                throw cpp_utils::exceptions::EmptyObjectException<FocalList>{*this};
            }
            auto focalNode = this->focalQueue.top();
            auto result = ((*focalNode.openHandle).data);

            this->stateToOpenQueue.erase((*focalNode.openHandle).data);
            this->openQueue.erase(focalNode.openHandle);

            this->focalQueue.pop();
            return *result;
        }

        virtual bool containsInOpen(const ITEM& n) const {
            // critical("states are ", this->stateToOpenQueue.size());
            // for (auto it=this->stateToOpenQueue.begin(); it!= this->stateToOpenQueue.end(); ++it) {
            //     critical("it:", *(it->first), " (pointer=", it->first, ")");
            // }
            // critical("n is", n, " (pointer=", &n, ")");
            return this->stateToOpenQueue.find(const_cast<ITEM*>(&n)) != this->stateToOpenQueue.end();
        }

        virtual size_t getOpenListSize() const {
            return this->openQueue.size();
        }

        virtual size_t getFocalListSize() const {
            return this->focalQueue.size();
        }
    };

}

#endif 