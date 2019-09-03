/*
 * xyLoc.cpp
 *
 *  Created on: Oct 8, 2018
 *      Author: koldar
 */


#include "xyLoc.hpp"
#include <cmath>
#include <cpp-utils/exceptions.hpp>
#include <algorithm>

namespace pathfinding {

const char* getLabel(const Direction& dir) {
	switch (dir) {
	case Direction::NORTHWEST: { return "NW"; }
	case Direction::NORTH: { return "N"; }
	case Direction::NORTHEAST: { return "NE";}
	case Direction::WEST: { return "W"; }
	case Direction::EAST: { return "E"; }
	case Direction::SOUTHWEST: { return "SW"; }
	case Direction::SOUTH: { return "S"; }
	case Direction::SOUTHEAST: { return "SE"; }
	default:
		throw cpp_utils::exceptions::InvalidScenarioException<Direction>{dir};
	}
}

Direction xyLoc::getDirectionTo(const xyLoc& to) const {
	return xyLoc::getDirection(*this, to);
}

Direction xyLoc::getDirection(const xyLoc& from, const xyLoc& to) {
	if ((from.x < to.x) && (from.y < to.y)) {
		return Direction::SOUTHEAST;
	} else if ((from.x == to.x) && (from.y < to.y)) {
		return Direction::SOUTH;
	} else if ((from.x > to.x) && (from.y < to.y)) {
		return Direction::SOUTHWEST;
	} else if ((from.x < to.x) && (from.y == to.y)) {
		return Direction::EAST;
	} else if ((from.x > to.x) && (from.y == to.y)) {
		return Direction::WEST;
	} else if ((from.x < to.x) && (from.y > to.y)) {
		return Direction::NORTHEAST;
	} else if ((from.x == to.x) && (from.y > to.y)) {
		return Direction::NORTH;
	} else if ((from.x > to.x) && (from.y > to.y)) {
		return Direction::NORTHWEST;
	}
	throw cpp_utils::exceptions::InvalidPairScenarioException<xyLoc, xyLoc>{from, to};
}

bool xyLoc::isAdjacentTo(const xyLoc& other) const {
	//we check both because xyLoc has unsigned integer coordinates, hence if pos1=(4,6) and pos2=(5,6) if we do pos1.x-pos2.x we obtain -1
	//but being unsigned we obtain UINT_MAX-1 (a very big number)

	return ((std::abs(static_cast<int>(this->x - other.x)) <= 1) || (std::abs(static_cast<int>(other.x - this->x)) <= 1)) &&
			((std::abs(static_cast<int>(this->y - other.y)) <= 1) || (std::abs(static_cast<int>(other.y - this->y)) <= 1));
}

bool xyLoc::isThereLocationInDirectionOf(Direction dir, xyLoc maxPoint, xyLoc minPoint) const {
	switch (dir) {
		case Direction::NORTH: {
			return this->y > minPoint.y;
		}
		case Direction::SOUTH: {
			return this->y < maxPoint.y;
		}
		case Direction::EAST: {
			return this->x < maxPoint.x;
		}
		case Direction::WEST: {
			return this->x > minPoint.x;
		}
		case Direction::NORTHWEST: {
			return this->y > minPoint.y && this->x > minPoint.x;
		}
		case Direction::NORTHEAST: {
			return this->y > minPoint.y && this->x < maxPoint.x;
		}
		case Direction::SOUTHWEST: {
			return this->y < maxPoint.y && this->x > minPoint.x;
		}
		case Direction::SOUTHEAST: {
			return this->y < maxPoint.y && this->x < maxPoint.x;
		}
		default: {
			throw cpp_utils::exceptions::InvalidScenarioException<Direction>{dir};
		}
	}
}

xyLoc operator +(const xyLoc& a, const xyLoc& b) {
	return xyLoc{a.x + b.x, a.y + b.y};
}

xyLoc operator +(const xyLoc& a, int value) {
	return xyLoc{a.x + value, a.y + value};
}

xyLoc operator -(const xyLoc& a, const xyLoc& b) {
	return xyLoc{a.x - b.x, a.y - b.y};
}

xyLoc operator -(const xyLoc& a, int value) {
	return xyLoc{a.x - value, a.y - value};
}

xyLoc min(const xyLoc& a, const xyLoc& b) {
	return xyLoc{std::min(a.x, b.x), std::min(a.y, b.y)};
}

xyLoc max(const xyLoc& a, const xyLoc& b) {
	return xyLoc{std::max(a.x, b.x), std::max(a.y, b.y)};
}

}