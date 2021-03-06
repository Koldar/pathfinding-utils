/*
 * @file
 *
 *  Created on: Oct 8, 2018
 *      Author: koldar
 */

#ifndef XYLOC_H_
#define XYLOC_H_

#include <boost/functional/hash.hpp>
#include <cstdint>
#include <iostream>
#include "types.hpp"

namespace pathfinding {

	/**
	 * A direction of a movement within the grid
	 */
	enum class Direction {
		NORTH,
		SOUTH,
		EAST,
		WEST,
		NORTHWEST,
		NORTHEAST,
		SOUTHWEST,
		SOUTHEAST
	};

	namespace DirectionMethods {

		/**
		 * @brief 
		 * 
		 * @param dir  the direction involved
		 * @return true if dir is either north, south, east or west
		 * @return false otherwise
		 */
		bool isStraight(Direction dir);

		/**
		 * @brief 
		 * 
		 * @param dir the direction involved
		 * @return true if dir is not straight
		 * @return false otherwise
		 */
		bool isDiagonal(Direction dir);


		/**
		 * @param[in] dir a direction
		 * @return the label associated
		 * ted to a direction
		 */
		const char* getLabel(const Direction& dir);

	}

	std::ostream& operator <<(std::ostream& ss, const Direction& dir);

	/**
	 * represent a position in the grid.
	 *
	 * Inside the grid, "x" is the column while "y" is the row.
	 * 
	 */
	struct xyLoc {
		///x coordinate
		ucood_t x;
		///y coordinate
		ucood_t y;

		xyLoc(ucood_t x, ucood_t y) : x{x}, y{y} {

		}

		xyLoc(ucood_t t): x{t}, y{t} {

		}

		xyLoc(): x{0}, y{0} {

		}

		xyLoc(const xyLoc& other): x{other.x}, y{other.y} {

		}

		xyLoc(xyLoc&& other): x{other.x}, y{other.y} {

		}

		xyLoc& operator =(const xyLoc& other) {
			this->x = other.x;
			this->y = other.y;
			return *this;
		};
		xyLoc& operator =(xyLoc&& other) {
			this->x = other.x;
			this->y = other.y;
			return *this;
		};
		xyLoc& operator +=(const xyLoc& other) = delete;
		xyLoc& operator -=(const xyLoc& other) = delete;
		xyLoc& operator *=(const xyLoc& other) = delete;
		xyLoc& operator /=(const xyLoc& other) = delete;

		/**
		 * The direction a cell is relative to another one
		 *
		 * for example if the current location is <tt>5,5</tt> and @c to is <tt>5,4</tt> the direction will be @c west because
		 * the second cell is on the west of the first one.
		 *
		 * @param[in] to the cell (other than this one) where we need to look at
		 * @return the direction between this cell and @c
		 */
		Direction getDirectionTo(const xyLoc& to) const;

		/**
		 * like ::xyLoc::getDirectionTo but we can specific 2 location, not just one
		 *
		 * @param[in] from the first cell to consider
		 * @param[in] to the second cell to consider
		 * @return the direction @c to is relative to @c from
		 */
		static Direction getDirection(const xyLoc& from, const xyLoc& to);

		/**
		 *
		 * 2 locations are immediately adjacent one to the other iff they shares a side or a point.
		 *
		 * for example \f$(4,5)\f$ is adjacent with \f$(4,6)\f$ or \f$(3,4)\f$ but is not adjacent with \f$(100, 400)\f$.
		 *
		 * If @c US represents this xyLoc location, "OK" stands for location immediately adjacent while "KO" stands for location not immediately adjacent.
		 *
		 *
		 * |--|--|--|--|
		 * |OK|OK|OK|KO|
		 * |OK|US|OK|KO|
		 * |OK|OK|OK|KO|
		 *
		 * @note
		 * This definition does not take into consideration if the underlying map is traversable or not in the involved locations
		 *
		 *
		 * @param[in] other the location to test against
		 * @return
		 *  @li true if this location is immediately adjacent to @c other;
		 *  @li false otherwise
		 */
		bool isAdjacentTo(const xyLoc& other) const;

		/**
		 * @brief check if, starting from this, we can go in a particular direction
		 * 
		 * @code
		 * 	{5,3}.isThereLocationInDirectionOf(LEFT, {5,5}); //yes
		 *  {5,3}.isThereLocationInDirectionOf(RIGHT, {5,5}); //no
		 * @endcode
		 * 
		 * @param dir the direction we should follow to check if there is another xyLoc.
		 * @param maxPoint coordinates representing the bottom right corner of an invisible rectangle we cannot escape from. You can have a location set to this point
		 * @param minPoint coordinates representing the top left corner of an invisible rectangle we cannot escape from. You can have a location set to this point
		 * @return true if there is a clocation adjacent to self by following @c dir
		 * @return false 
		 */
		bool isThereLocationInDirectionOf(Direction dir, xyLoc maxPoint, xyLoc minPoint = {0, 0}) const;

		/**
		 * @brief Get the Nearby Diagonale Cells object
		 * 
		 * If `loc1` and `loc2` are marked as `1` and `2` the out locations will be "a" and "b"
		 * @code
		 * 1|a
		 * -|-
		 * b|2
		 * @endcode
		 * 
		 * @pre
		 *  @li loc1 adjacent to loc2
		 * 
		 * @param loc1 first location
		 * @param loc2 second location
		 * @return a pair representing the adjacent cells
		 */
		static std::pair<xyLoc, xyLoc> getNearbyDiagonalCells(const xyLoc& loc1, const xyLoc& loc2);

		/**
		 * The coordiante system is in the topLeft corner (0,0) while the infinity is in the bottomRight
		 *
		 * locationns within the border of the rectangle are considered as well
		 *
		 * @param[in] topLeft the point representing a point of the rectangle
		 * @param[in] bottomRight the point representing a point of the rectangle
		 * @return
		 *  @li true if the point is inside the rectangle generated by @c topLeft and @c bottomRight
		 */
		bool isInside(const xyLoc& topLeft, const xyLoc& bottomRight) const {
			return (topLeft.x <= this->x)&&(this->x <= bottomRight.x) &&
					(topLeft.y <= this->y)&&(this->y <= bottomRight.y);
		}

		/**
		 * @brief return the distance between 2 points, ignoring sign
		 * 
		 * For example here:
		 * 
		 * ```
		 * | |B| | |
		 * | | | | |
		 * | | | | |
		 * | | | |A|
		 * ```
		 * 
		 * it would be `<-2, -3>` but with this method it will return `<2, 3>`
		 * 
		 * @param other the other point
		 * @return xyLoc 
		 */
		xyLoc getDistance(const xyLoc& other) const {
			cood_t ax = static_cast<cood_t>(this->x);
			cood_t ay = static_cast<cood_t>(this->y);
			cood_t bx = static_cast<cood_t>(other.x);
			cood_t by = static_cast<cood_t>(other.y);

			return xyLoc{
				static_cast<ucood_t>(std::abs(ax - bx)), 
				static_cast<ucood_t>(std::abs(ay - by))
			};
		}

		/**
		 * @brief get the adjacent cell of self by following a direction
		 * 
		 * @note
		 * UB if you don't check with ::isThereLocationInDirectionOf if the direction might generate a result
		 * 
		 * @param dir ther direction to follow to obtain a new location
		 * @return xyLoc the adjacent location by following a particular direction
		 */
		xyLoc getAdjacent(Direction dir) const;

		/**
		 * @brief Get the Min Coordinate object
		 * 
		 * @return ucood_t the coordinate which has the least value between x and y
		 */
		ucood_t getMinCoordinate() const {
			return this->x < this->y ? this->x : this->y;
		}

		/**
		 * @brief Get the Max Coordinate object
		 * 
		 * @return ucood_t the coordinate which has the greatest value between x and y
		 */
		ucood_t getMaxCoordinate() const {
			return this->x > this->y ? this->x : this->y;
		}

		friend std::size_t hash_value(const xyLoc& p) {
			std::size_t seed = 0;
			boost::hash_combine(seed, p.x);
			boost::hash_combine(seed, p.y);

			return seed;
		}
	};

	std::ostream& operator<<(std::ostream& str, const xyLoc& v);
	bool operator==(const xyLoc& a, const xyLoc& b);
	bool operator!=(const xyLoc& a, const xyLoc& b);
	xyLoc operator +(const xyLoc& a, const xyLoc& b);
	xyLoc operator +(const xyLoc& a, int value);
	xyLoc operator -(const xyLoc& a, const xyLoc& b);
	xyLoc operator -(const xyLoc& a, int value);

	/**
	 * @brief anew xyLoc containing the minimum of the coordinates of 2 xyLoc
	 * 
	 * @code
	 * 	min.x = min(a.x, b.x);
	 *  min.y = min(a.y, b.y);
	 * @endcode
	 * 
	 * @param a first location
	 * @param b second location
	 * @return xyLoc minimum of coordinates
	 */
	xyLoc min(const xyLoc& a, const xyLoc& b);
	/**
	 * @brief a new xyLoc containing the maximum of the coordinates of 2 xyLoc
	 * 
	 * @code
	 * 	max.x = max(a.x, b.x);
	 *  max.y = max(a.y, b.y);
	 * @endcode
	 * 
	 * @param a first location
	 * @param b second location
	 * @return xyLoc maximum of coordinates
	 */
	xyLoc max(const xyLoc& a, const xyLoc& b);

}

namespace std {

	template <>
	struct hash<pathfinding::xyLoc> {
	public:
		size_t operator()(const pathfinding::xyLoc& l) const {
			size_t seed = 0;
			boost::hash_combine(seed, l.x);
			boost::hash_combine(seed, l.y);
			return seed;
		}
	};

}

#endif /* XYLOC_H_ */
