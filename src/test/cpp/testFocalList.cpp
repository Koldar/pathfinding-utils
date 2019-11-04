
#include "catch.hpp"
#include "types.hpp"
#include "FocalList.hpp"

using namespace pathfinding;
using namespace pathfinding::data_structures;

class Foo {
    private:
        char state;
        cost_t cost;
        int valueOpen;
        int valueFocal;
    public:
        Foo(cost_t openCost, int valueOpen, int valueFocal, char state): cost{openCost}, valueOpen{valueOpen}, valueFocal{valueFocal}, state{state} {

        }
        virtual ~Foo() {

        }
    public:
        void setOpenCost(cost_t c) {
            this->cost = c;
        }
        cost_t getOpenCost() const {
            return this->cost;
        }
        void setOpenValue(int v) {
            this->valueOpen = v;
        }
        void setFocalValue(int v) {
            this->valueFocal = v;
        }
        int getOpenValue() const {
            return this->valueOpen;
        }
        int getFocalValue() const {
            return this->valueFocal;
        }
    public:
        friend std::ostream& operator <<(std::ostream& ss, const Foo& f) {
            ss << "{ cost=" << f.cost << " open value=" << f.valueOpen << " focal value=" << f.valueFocal << " state=" << f.state << "}";
            return ss;
        }
};

struct FooBaseOrdered {
    bool operator() (const Foo& a, const Foo& b) {
        return a.getOpenValue() < b.getOpenValue();
    }
};

struct FooFocalOrdered {
    bool operator() (const Foo& a, const Foo& b) {
        return a.getFocalValue() < b.getFocalValue();
    }
};

struct FooGetCost {
    cost_t operator() (const Foo& a) {
        return a.getOpenCost();
    }
};

SCENARIO("test FocalList") {

    FocalList<Foo, FooBaseOrdered, FooFocalOrdered, FooGetCost> focalList{2};

    auto fooa = Foo{0, 0, 0, 'a'};
    auto foob = Foo{10, 10, 10, 'b'};
    auto fooc = Foo{20, 20, 5, 'c'};
    auto food = Foo{30, 30, 0, 'd'};
    auto fooe = Foo{40, 40, 0, 'e'};

    GIVEN("pushing") {

        WHEN("pushing to open list") {
            REQUIRE(focalList.isOpenEmpty() == true);
            REQUIRE(focalList.isFocalEmpty() == true);

            focalList.pushInOpen(fooa);

            REQUIRE(focalList.isOpenEmpty() == false);
            REQUIRE(focalList.isFocalEmpty() == true);

            critical("checking if fooa is in open list");
            REQUIRE(focalList.containsInOpen(fooa));
            REQUIRE(focalList.containsInOpen(foob) == false);
        }

        WHEN("pushing to focal list as well") {
            REQUIRE(focalList.isOpenEmpty() == true);
            REQUIRE(focalList.isFocalEmpty() == true);

            focalList.pushInOpenAndInFocal(fooa);

            REQUIRE(focalList.isOpenEmpty() == false);
            REQUIRE(focalList.isFocalEmpty() == false);

            critical("checking if fooa is in open list");
            REQUIRE(focalList.containsInOpen(fooa));
            REQUIRE(focalList.containsInOpen(foob) == false);

            Foo& top = focalList.peekFromFocal();
            REQUIRE(&top == &fooa);

            top = focalList.popFromFocal();
            REQUIRE(&top == &fooa);
            REQUIRE(focalList.getOpenListSize() == 0);
            REQUIRE(focalList.getFocalListSize() == 0);
        }

        WHEN("pushing one on open and on on focal list") {
            focalList.pushInOpen(foob);
            focalList.pushInOpenAndInFocal(fooa);

            REQUIRE(focalList.getOpenListSize() == 2);
            REQUIRE(focalList.getFocalListSize() == 1);

            critical("a_ptr=", &fooa, "b_ptr=", &foob);
            REQUIRE(&focalList.popFromFocal() == &fooa);
            
            REQUIRE(focalList.getOpenListSize() == 1);
            REQUIRE(focalList.getFocalListSize() == 0);
        }

        WHEN("pushing both to focal list. focal comparator yield outcome aligned with open") {
            focalList.pushInOpenAndInFocal(foob);
            focalList.pushInOpenAndInFocal(fooa);

            REQUIRE(focalList.getOpenListSize() == 2);
            REQUIRE(focalList.getFocalListSize() == 2);

            critical("a_ptr=", &fooa, "b_ptr=", &foob);
            REQUIRE(&focalList.popFromFocal() == &fooa);
            REQUIRE(&focalList.popFromFocal() == &foob);
        }

        WHEN("pushing both to focal list. focal comparator yield outcome NOT aligned with open") {
            focalList.pushInOpenAndInFocal(foob);
            focalList.pushInOpenAndInFocal(fooc);

            REQUIRE(focalList.getOpenListSize() == 2);
            REQUIRE(focalList.getFocalListSize() == 2);

            critical("a_ptr=", &foob, "b_ptr=", &fooc);
            REQUIRE(&focalList.popFromFocal() == &fooc);
            REQUIRE(&focalList.popFromFocal() == &foob);
        }

    }

}