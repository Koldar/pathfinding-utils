/* 
 * scenarioLoader.cpp
 * hog
 * 
 * Created by Renee Jansen on 5/2/2006
 *
 */

#include <fstream>
#include "GridMapScenarioLoader.hpp"

namespace pathfinding::maps {

    /** 
     * Loads the experiments from the scenario file. 
     */
    GridMapScenarioLoader::GridMapScenarioLoader(const boost::filesystem::path& scenarioFilename): filename{scenarioFilename}, experiments{} {
        std::ifstream sfile(filename.string(), std::ios::in);
        
        float version;
        std::string first;
        sfile >> first;

        // Check if a version number is given
        if(first != "version"){
            version = 0.0;
            sfile.seekg(0, std::ios::beg);
        }
        else{
            sfile >> version;
        }

        int sizeX = 0;
        int sizeY = 0; 
        int bucket;
        std::string map;  
        ucood_t xs, ys, xg, yg;
        double dist;

        // Read in & store experiments
        if (version == 0.0) {
            while(sfile >> bucket >> map >> xs >> ys >> xg >> yg >> dist) {
                experiments.push_back(GridMapScenarioExperiment{xs,ys,xg,yg,bucket,dist,map});
            }
        } else if (version == 1.0) {
            while(sfile >> bucket >> map >> sizeX >> sizeY >> xs >> ys >> xg >> yg >> dist){
                experiments.push_back(GridMapScenarioExperiment{xs,ys,xg,yg,sizeX,sizeY,bucket,dist,map});
            }
        } else {
            throw cpp_utils::exceptions::InvalidScenarioException<double>{version};
        }
    }

    const vectorplus<GridMapScenarioExperiment>& GridMapScenarioLoader::getExperiments() const {
        return this->experiments;
    }

    // void GridMapScenarioLoader::Save(const char *fname) const
    // {
    // //	strncpy(scenName, fname, 1024);
    //     ofstream ofile(fname);
        
    //     float ver = 1.0;
    //     ofile<<"version "<<ver<<std::endl;
        
        
    //     for (unsigned int x = 0; x < experiments.size(); x++)
    //     {
    //         ofile<<experiments[x].bucket<<"\t"<<experiments[x].map<<"\t"<<experiments[x].scaleX<<"\t";
    //         ofile<<experiments[x].scaleY<<"\t"<<experiments[x].startx<<"\t"<<experiments[x].starty<<"\t";
    //         ofile<<experiments[x].goalx<<"\t"<<experiments[x].goaly<<"\t"<<experiments[x].distance<<std::endl;
    //     }
    // }

    // void GridMapScenarioLoader::AddExperiment(GridMapScenarioExperiment which)
    // {
    //     experiments.push_back(which);
    // }

}