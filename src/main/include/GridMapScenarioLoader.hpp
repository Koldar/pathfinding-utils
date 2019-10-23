/*
 * @file
 * 
 * 
 * @note
 * file modified from "cenarioLoader", Renee Jansen on 5/2/2006. Author kept.
 *
 * Created by Renee Jansen on 5/2/2006
 *
 */ 

#ifndef _PATHFINDING_UTILS_SCENARIO_LOADER_HEADER__
#define _PATHFINDING_UTILS_SCENARIO_LOADER_HEADER__

#include <cpp-utils/vectorplus.hpp>
#include <cstring>
#include <string>
#include "xyLoc.hpp"

namespace pathfinding::maps {

	using namespace cpp_utils;

	/** 
	 * Experiments stored by the GridMapScenarioLoader class. 
	 */
	class GridMapScenarioLoader;

	/**
	 * An experiment is a path finding query that needs to be answered
	 *
	 * Normally experiments query should be time-profiled
	 */
	class GridMapScenarioExperiment {
	public:
		/**
		 * creates a new experiment
		 *
		 * @param[in] sx the x of the start location of the agent
		 * @param[in] xy the y of the start location of the agent
		 * @param[in] gx the x of the location the agent needs to reach
		 * @param[in] gy yhe y of the location the agent needs to reach
		 * @param[in] d the euclidean distance between point "s" and point "g"
		 * @param[in] m the filename of the map involved
		 */
		GridMapScenarioExperiment(ucood_t sx, ucood_t sy, ucood_t gx, ucood_t gy, int b, double d, const std::string& m)
		:startx{sx}, starty{sy}, goalx{gx}, goaly{gy}, scaleX{-1}, scaleY{-1}, bucket{b}, distance{d}, map{m} {

		}
		/**
		 * creates a new experiment
		 *
		 * @param[in] sx the x of the start location of the agent
		 * @param[in] xy the y of the start location of the agent
		 * @param[in] gx the x of the location the agent needs to reach
		 * @param[in] gy yhe y of the location the agent needs to reach
		 * @param[in] sizeX the width of the map
		 * @param[in] sizeY the height of the map
		 * @param[in] b the bucket of the experiment
		 * @param[in] d the euclidean distance between point "s" and point "g"
		 * @param[in] m the filename of the map involved
		 */
		GridMapScenarioExperiment(ucood_t sx,ucood_t sy, ucood_t gx, ucood_t gy, int sizeX, int sizeY,int b, double d, const std::string& m)
		:startx{sx}, starty{sy}, goalx{gx}, goaly{gy}, scaleX{sizeX}, scaleY{sizeY}, bucket{b}, distance{d}, map{m} {
		}
		// /**
		//  * @return the x of the starting point
		//  */
		// int GetStartX() const {return startx;}
		// /**
		//  * @return the y of the starting point
		//  */
		// int GetStartY() const {return starty;}

		/**
		 * @return the start of the agent
		 */
		xyLoc getStart() const  {
			return xyLoc{startx, starty};
		}

		/**
		 * @return the goal of the agent
		 */
		xyLoc getGoal() const {
			return xyLoc{goalx, goaly};
		}

		// /**
		//  * @return the x of the goal to reach
		//  */
		// int GetGoalX() const {return goalx;}
		// /**
		//  * @return the y of the goal to reach
		//  */
		// int GetGoalY() const {return goaly;}
		/**
		 * @return the bucket of the experiment
		 */
		int GetBucket() const {return bucket;}
		/**
		 * @return the distance betwen the starting point and the goal
		 */
		double GetDistance() const {return distance;}
		// /**
		//  * generate the name of the map involved
		//  *
		//  * @param[inout] mymap an initialized buffer where to store the map name
		//  */
		// void GetMapName(char* mymap) const {strcpy(mymap,map.c_str());}
		/**
		 * get the actual name of the map
		 *
		 * @return the actual pointer of the filename of the map
		 */
		const std::string& GetMapName() const { return this->map; }
		/**
		 * @return
		 *  @li the width of the map;
		 *  @li -1 if the scenario file didn't advertise the sizeof the map
		 */
		int GetXScale() const {return scaleX;}
		/**
		 * @return
		 *  @li the height of the map;
		 *  @li -1 if the scenario file didn't advertise the sizeof the map
		 */
		int GetYScale() const {return scaleY;}
	public:
		friend std::ostream& operator<<(std::ostream& ss, const GridMapScenarioExperiment& g) {
			ss << "{ map=" << g.map << " bucket=" << g.bucket << " start=" << g.getStart() << " goal=" << g.getGoal() << " distance=" << g.distance << " }";
			return ss;
		}
	private:
		friend class GridMapScenarioLoader;
		ucood_t startx, starty, goalx, goaly;
		int scaleX;
		int scaleY;
		int bucket;
		double distance;
		std::string map;
	};

	/** A class which loads and stores scenarios from files.  
	 * Versions currently handled: 0.0 and 1.0 (includes scale). 
	 */
	class GridMapScenarioLoader {
	private:
		/**
		 * name of the scenario
		 */
		const boost::filesystem::path filename;
		/**
		 * list of experiment inside the scenario
		 */
		vectorplus<GridMapScenarioExperiment> experiments;
	public:
		/**
		 * cerate a GridMapScenarioLoader which automatically read the data in the scenario file given
		 *
		 * @param[in] the filename specifying the scenario file to load
		 */
		GridMapScenarioLoader(const boost::filesystem::path& scenarioFilename);
		/**
		 * save all the experiments in the given GridMapScenarioLoader in a file
		 *
		 * @note
		 * the file will be formatted according to the scenario file layout version 1.0
		 *
		 * @param[in] the filename of the file to dump the experiments on
		 */
		// void Save(const char *) const;

		/**
		 * @brief the experiments in this scenario file
		 * 
		 * @return const vectorplus<GridMapScenarioExperiment> 
		 */
		const vectorplus<GridMapScenarioExperiment>& getExperiments() const;

		// /**
		//  * @return the number of experiments in this GridMapScenarioLoader
		//  */
		// int GetNumExperiments() const{return experiments.size();}
		// /**
		//  * @return a name representing this set of experiments
		//  */
		// const char *GetScenarioName() const { return scenName; }
		// /**
		//  * @param[in] which the id of the experiment to retrieve
		//  * @return the GridMapScenarioExperiment you want to fetch
		//  */
		// GridMapScenarioExperiment GetNthExperiment(int which) const
		// {return experiments[which];}
		/**
		 * Adds a new experiment in the scenario
		 *
		 * @note
		 * the experiment will be added at the end of all other experiments
		 *
		 * @param[in] which experiment to add
		 */
		// void AddExperiment(GridMapScenarioExperiment which);

	};
}

#endif
