//
// Created by lacie on 27/08/2023.
//

#ifndef SIMPLESFM_MAPBUILDER_H
#define SIMPLESFM_MAPBUILDER_H

#include <string>

#include "Utils/Types.h"
#include "Utils/Timer.h"
#include "Utils/Visualization.h"

#include "SfM/SceneGraph.h"
#include "SfM/RegisterGraph.h"
#include "SfM/Initializer.h"
#include "SfM/Registrant.h"
#include "SfM/Triangulator.h"
#include "SfM/Map.h"

#include "Optimizer/BundleData.h"
#include "Optimizer/CeresBundleOptimizer.h"

namespace SimpleSfM
{

    class MapBuilder
    {
    public:
        struct Parameters
        {
            // camera reference
            double fx;
            double fy;
            double cx;
            double cy;

            // Distortion parameter, the default parameter is invalid
            double k1 = 0.0;
            double k2 = 0.0;
            double p1 = 0.0;
            double p2 = 0.0;

            Initializer::Parameters init_params;        // When initializing, the parameters that need to be used
            Registrant::Parameters regis_params;        // The parameters needed to calculate the camera position and pose according to the 2D-3D point correspondence
            Triangulator::Parameters tri_params;        // Register the next picture, the parameters needed for triangulation
            CeresBundleOptimizer::Parameters ba_params; // Parameters needed for BA optimization

            size_t min_num_matches = 10; // Only image pairs with matching numbers greater than this threshold in the database will be loaded into the scene graph
            size_t max_num_init_trials = 100;

            double complete_max_reproj_error = 4.0; // When completing the track, the maximum reprojection error
            double merge_max_reproj_error = 4.0;    // When merging tracks, the maximum reprojection error
            double filtered_max_reproj_error = 4.0; // When filtering track, the maximum reprojection error
            double filtered_min_tri_angle = 1.5;    // When filtering track, the minimum angle to be satisfied

            double global_ba_ratio = 1.07; // When the image increases the ratio, the global BA will only be performed

            bool is_visualization = true; // Whether to enable the visualization of point cloud and camera during reconstruction
        };
        struct Statistics
        {
            // TOOD
        };

    public:
        MapBuilder(const std::string &database_path, const MapBuilder::Parameters &params);

        ////////////////////////////////////////////////////////////////////////////////
        // When rebuilding, the function that needs to be called
        // Call SetUp() to set the data that needs to be loaded when rebuilding
        // Call DoBuild() to rebuild
        // Call Summary() to output the statistical information of the reconstruction result
        ////////////////////////////////////////////////////////////////////////////////
        void SetUp();
        void DoBuild();
        void Summary();

        //
        ////////////////////////////////////////////////////////////////////////////////
        // Write the reconstruction results (camera parameters, image parameters, 3D points) to the file
        ////////////////////////////////////////////////////////////////////////////////
        void WriteCOLMAP(const std::string &directory);
        void WriteOpenMVS(const std::string &directory);
        void WritePLY(const std::string &path);
        void WritePLYBinary(const std::string &path);
        void Write(const std::string &path);
        void WriteCamera(const std::string &path);
        void WriteImages(const std::string &path);
        void WritePoints3D(const std::string &path);

    private:
        ////////////////////////////////////////////////////////////////////////////////
        // Find image pairs for initialization
        ////////////////////////////////////////////////////////////////////////////////
        std::vector<image_t> FindFirstInitialImage() const;
        std::vector<image_t> FindSecondInitialImage(image_t image_id) const;

        ////////////////////////////////////////////////////////////////////////////////
        // Try to initialize until it succeeds, or gets a limited number of initializations
        ////////////////////////////////////////////////////////////////////////////////
        bool TryInitialize();

        ////////////////////////////////////////////////////////////////////////////////
        // Try to register the next image
        ////////////////////////////////////////////////////////////////////////////////
        bool TryRegisterNextImage(const image_t &image_id);
        size_t Triangulate(const std::vector<std::vector<Map::CorrData>> &points2D_corr_datas,
                           double &ave_residual);

        ////////////////////////////////////////////////////////////////////////////////
        // What to do if Local BA is performed
        ////////////////////////////////////////////////////////////////////////////////
        void LocalBA();
        void MergeTracks();
        void CompleteTracks();
        void FilterTracks();

        ////////////////////////////////////////////////////////////////////////////////
        // If doing Global BA, what needs to be done
        ////////////////////////////////////////////////////////////////////////////////
        void GlobalBA();
        void FilterAllTracks();
        // TODO : Reconstructive triangulation of 2D points in an image (or pair of images) without 3D points
        void Retriangulate();

        std::string database_path_;
        Parameters params_;

        cv::Ptr<Initializer> initailizer_;
        cv::Ptr<Registrant> registrant_;
        cv::Ptr<Triangulator> triangulator_;

        cv::Ptr<RegisterGraph> register_graph_;
        cv::Ptr<SceneGraph> scene_graph_;
        cv::Ptr<Map> map_;
        cv::Ptr<CeresBundleOptimizer> bundle_optimizer_;
        cv::Ptr<AsyncVisualization> async_visualization_;

        int width_;
        int height_;
        cv::Mat K_;
        cv::Mat dist_coef_;

        Timer timer_for_initialize_;
        Timer timer_for_register_;
        Timer timer_for_triangulate_;
        Timer timer_for_merge_;
        Timer timer_for_complete_;
        Timer timer_for_local_filter_;
        Timer timer_for_local_ba_;
        Timer timer_for_global_filter_;
        Timer timer_for_global_ba_;

        Timer timer_for_visualization_;

        Timer timer_for_debug_;

        Timer timer_for_total_;
    };

}

#endif //SIMPLESFM_MAPBUILDER_H
