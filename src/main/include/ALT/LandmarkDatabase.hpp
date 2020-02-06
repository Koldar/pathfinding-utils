#ifndef _PATHFINDING_UTILS_LANDMARK_DATABASE_HEADER__
#define _PATHFINDING_UTILS_LANDMARK_DATABASE_HEADER__

#include <boost/filesystem.hpp>

#include <cpp-utils/igraph.hpp>
#include <cpp-utils/profiling.hpp>
#include <cpp-utils/imemory.hpp>
#include <cpp-utils/operators.hpp>

#include "AbstractLandmarkPlacingStrategy.hpp"

#include <cstdio>

namespace pathfinding::search {
    template <typename G, typename V>
    class LandmarkDatabase;
}

namespace cpp_utils::serializers {

    using namespace cpp_utils::graphs;

    template <typename G, typename V>
    void saveToFile(std::FILE* f, const pathfinding::search::LandmarkDatabase<G, V>& db) {
        saveToFile(f, db.landmarks);
        for (int i=0; i<db.landmarks.size(); ++i) {
            nodeid_t landmark = db.landmarks[i];
            //we don't save the dpf::cost_t by itself, but we save only the first 4 bytes of it
            saveToFile(f, db.landmarksDistances.at(landmark));
        }
    }

    template <typename G, typename V>
    pathfinding::search::LandmarkDatabase<G, V>& loadFromFile(FILE* f, pathfinding::search::LandmarkDatabase<G, V>& result) {
        loadFromFile(f, result.landmarks);
        for (int i=0; i<result.landmarks.size(); ++i) {
            nodeid_t landmark = result.landmarks[i];
            std::vector<uint32_t> loaded{};
            loadFromFile(f, loaded);
            result.landmarksDistances[landmark] = loaded;
        }
        return result;
    }

}

namespace pathfinding::search {

    

    

    /**
     * @brief Manages a chunk of memory which represent sthe Landmakr database
     * 
     * @note
     * The implementation assumes that the underlying graph is, albeit directed, rerepsenting
     * an undirected graph!
     * 
     * @tparam G 
     * @tparam V 
     */
    template <typename G, typename V>
    class LandmarkDatabase: public IMemorable {
        using This = LandmarkDatabase<G, V>;
        friend This& ::cpp_utils::serializers::loadFromFile<G, V>(FILE* f, This& result);
        friend void ::cpp_utils::serializers::saveToFile<G, V>(FILE* f, const This& result);
    private:
        std::unordered_map<nodeid_t, std::vector<distance_t>> landmarksDistances;
        std::vector<nodeid_t> landmarks;
    private:
        /**
         * @brief An empty database which still needs to be populated
         * 
         * Internal usage only
         * 
         */
        LandmarkDatabase(): landmarksDistances{}, landmarks{} {

        }
    public:
        virtual ~LandmarkDatabase() {

        }
        LandmarkDatabase(const This& o): landmarksDistances{o.landmarksDistances}, landmarks{o.landmarks} {

        }
        LandmarkDatabase(This&& o): landmarksDistances{::std::move(o.landmarksDistances)}, landmarks{::std::move(o.landmarks)} {

        }
        This& operator=(const This& o) {
            this->landmarksDistances = o.landmarksDistances;
            this->landmarks = o.landmarks;
            return *this;
        }
        This& operator=(const This&& o) {
            this->landmarksDistances = ::std::move(o.landmarksDistances);
            this->landmarks = ::std::move(o.landmarks);
            return *this;
        }
    private:
        void populateDatabase(const IImmutableGraph<G, V, cost_t>& graph, AbstractLandmarkPlacingStrategy<G,V>& policy) {
            timing_t timeElapsed;
            PROFILE_TIME(timeElapsed) {
                this->landmarks = policy.getLandmarks(graph, this->landmarksDistances);
            }
            critical("in order to generate the landmarks of the graph we took ", timeElapsed.toMicros(), "!");

            if (this->landmarksDistances.size() == 0) {
                throw cpp_utils::exceptions::ImpossibleException{"landmarks distances is empty! policy is %p", &policy};
            }

            DO_ON_DEBUG {
                int expectedSize = this->landmarksDistances[this->landmarks[0]].size();
                for (auto it=this->landmarksDistances.begin(); it!=this->landmarksDistances.end(); ++it) {
                    if (it->second.size() != expectedSize) {
                        log_error("sizes don't match!");
                        assert(false);
                    }
                }
            }
        }
    public:
        /**
         * @brief Return the number of vertices in the graph from the landmark database itself
         * 
         * @return size_t 
         */
        size_t getGraphSize() const {
            return this->landmarksDistances.at(this->landmarks.at(0)).size();
        }

        /**
         * @brief Fetch a list of landmarks from the database
         * 
         * @return const std::vector<nodeid_t> the list of landmarks.
         */
        const std::vector<nodeid_t> getLandmarks() const {
            return this->landmarks;
        }
        /**
         * @brief Get the distance between this landmakr and a particular node
         * 
         * @param landmark 
         * @param n 
         * @return dpf::cost_t 
         */
        cost_t getDistanceFromLandmarkToNode(nodeid_t landmark, nodeid_t n) const {
            return this->landmarksDistances.at(landmark)[n];
        }
    public:
        static This fetchOrCompute(const IImmutableGraph<G, V, cost_t>& graph, AbstractLandmarkPlacingStrategy<G, V>& policy, const boost::filesystem::path& databaseFilename) {
            info("checking if ", databaseFilename.string(), " exists..");
            if (boost::filesystem::exists(databaseFilename)) {
                debug("it exists!");
                FILE* f = fopen(databaseFilename.string().c_str(), "rb");
                if (f == nullptr) {
                    throw cpp_utils::exceptions::FileOpeningException{databaseFilename};
                }
                This result{};
                cpp_utils::serializers::loadFromFile(f, result);
                fclose(f);
                return result;
            } else {
                debug("nope!");
                This result{};
                result.populateDatabase(graph, policy);

                FILE* f = fopen(databaseFilename.native().c_str(), "wb");
                if (f == NULL) {
                    throw cpp_utils::exceptions::FileOpeningException{databaseFilename};
                }
                cpp_utils::serializers::saveToFile(f, result);
                fclose(f);
                return result;
            }
        }
    public:
        virtual MemoryConsumption getByteMemoryOccupied() const {
            MemoryConsumption result{};

            for (auto it=this->landmarksDistances.begin(); it != this->landmarksDistances.end(); ++it) {
                result += it->second.capacity() * sizeof(it->second.front());
            }

            result += sizeof(this->landmarks.front()) * this->landmarks.capacity();
            result += sizeof(*this);

            return result;
        }
    };

}

#endif