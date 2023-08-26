//
// Created by lacie on 23/08/2023.
//

#ifndef SIMPLESFM_REGISTERGRAPH_H
#define SIMPLESFM_REGISTERGRAPH_H

#include <vector>
#include <cassert>
#include <cstddef>
#include <algorithm>

#include "Utils/Types.h"

namespace SimpleSfM
{
    class RegisterGraph
    {
    public:
        RegisterGraph(const size_t &total_node);

        void AddEdge(const image_t &image_id1, const image_t &image_id2);

        bool IsRegistered(const image_t &image_id);
        void SetRegistered(const image_t &image_id);

        void AddNumTrial(const image_t &image_id);
        size_t GetNumTrial(const image_t &image_id);

        std::vector<size_t> GetAllImagesNumTrial();
        double GetMeanNumTrial();

        size_t NumRegisteredImage() const;

        // Get the picture to be registered next time
        // Sort by priority from high to low
        std::vector<image_t> GetNextImageIds();

    private:
        std::vector<std::vector<image_t>> nodes_;

        std::vector<bool> registered_;
        std::vector<size_t> num_registered_trials_;
        std::vector<size_t> num_registered_neighbor_;
        std::vector<size_t> registered_images_;
        size_t total_node_;
    };
}

#endif //SIMPLESFM_REGISTERGRAPH_H
