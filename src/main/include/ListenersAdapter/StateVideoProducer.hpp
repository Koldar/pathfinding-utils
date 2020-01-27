#ifndef _CPPUTILS_STATEVIDEOPRODUCER_HEADER__
#define _CPPUTILS_STATEVIDEOPRODUCER_HEADER__

#include <cpp-utils/VideoBuilder.hpp>

#include "utils.hpp"
#include "IPathFindingMap.hpp"

#include "StateImageProducer.hpp"

namespace pathfinding::search::listeners {

    using namespace pathfinding;
    using namespace pathfinding::maps;

    /**
     * @brief a class which generate an image every time a node is expanded
     * 
     */
    //TODO this should have information about the perturbations!
    template <typename G, typename V, typename EORIGINAL, typename EPERTURBATED, typename MAPTYPE, typename IMAGETYPE>
    class StateVideoProducer: public StateImageProducer<G, V, EORIGINAL, EPERTURBATED, MAPTYPE, IMAGETYPE> {
    public:
        using This = StateVideoProducer<G, V, EORIGINAL, EPERTURBATED, MAPTYPE, IMAGETYPE>;
        using Super = StateImageProducer<G, V, EORIGINAL, EPERTURBATED, MAPTYPE, IMAGETYPE>;
    protected:
        VideoBuilder videoBuilder;
    public:
        StateVideoProducer(const IImmutableGraph<G,V,EORIGINAL>& originalGraph, const IImmutableGraph<G, V, EPERTURBATED>& perturbatedGraph, const MAPTYPE& map, const function_t<EPERTURBATED,cost_t>& costFunction): Super{originalGraph,  perturbatedGraph, map, costFunction}, videoBuilder{} {
            this->videoBuilder
                .setDuration(1)
                .setInputFramerate(25)
                .setOutputFramerate(25)
            ;
        }
        virtual ~StateVideoProducer() {

        }
        StateVideoProducer(const This& o) = default;
        StateVideoProducer(This&& o) = default;
        This& operator=(const This& o) = default;
        This& operator=(This&& o) = default;
    public:
        /**
         * @brief adds a frame in the video
         * 
         * @param image the frame to add
         */
        void addImage(const boost::filesystem::path& image) {
            this->videoBuilder.addImage(image);
        }
        /**
         * @brief build the video
         * 
         * @param outputName output name of the vide
         */
        void buildVideo(const boost::filesystem::path& outputName) {
            this->videoBuilder.buildVideo(outputName);
        }
        /**
         * @brief build the video
         * 
         * @param outputName output name fo the video
         * @param audio audio to add to the video
         */
        void buildVideo(const boost::filesystem::path& outputName, const boost::filesystem::path& audio) {
            this->videoBuilder
                .setAudio(audio)
                .buildVideo(outputName);
        }
    };

}

#endif