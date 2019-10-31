#ifndef _PATHFINDINGUTILS_FOCAL_LIST_HEADER__
#define _PATHFINDINGUTILS_FOCAL_LIST_HEADER__

namespace pathfinding::search {

    template <typename ITEM>
    class FocalList: public IQueue<ITEM>, public ICleanable {
        typedef FocalList<ITEM> This;
        typedef IQueue<ITEM> Super;
    private:
        StaticPriorityQueue<ITEM> openList;
        StaticPriorityQueue<ITEM> focalList;
    public:
        FocalList(bool minQueue, size_t openCapacity, size_t focalCapacity): openList{minQueue, openCapacity}, focalList{minqueue, focalCapacity} {

        }
        virtual ~FocalList() {

        }
        FocalList(const This& o): openList{o.openList}, focalList{o.focalList} {

        }
        FocalList(This&& o):, openList{::std::move(o.openList)}, focalList{::std::move(o.focalList)} {

        }
        This& operator=(const This& o) {
            this->openList = o.openList;
            this->focalList = o.focalList;
            return *this;
        }
        This& operator=(This&& o) {
            this->openList = ::std::move(o.openList);
            this->focalList = ::std::move(o.focalList);
            return *this;
        }
    public:
        bool isEmpty() const {
            return this->openList.isEmpty();
        }
        size_t size() const {
            return this->openList.size();
        }
        ITEM& peek() const {
            this->focalList.peek();
        }
    public:
        void cleanup() {
            this->openList.cleanup();
            this->focalList.cleanup();
        }
    public:
        bool isOpenEmpty() const {
            return this->openList.isEmpty();
        }
        bool isFocalEmpty() const {
            return this->focalList.isEmpty();
        }
    }


}

#endif