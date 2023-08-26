//
// Created by lacie on 27/08/2023.
//

#ifndef SIMPLESFM_CERESBUNDLEOPTIMIZER_H
#define SIMPLESFM_CERESBUNDLEOPTIMIZER_H

#include "Optimizer/BundleData.h"

void initLogging();

namespace SimpleSfM
{

    class CeresBundleOptimizer
    {
    public:
        struct Parameters
        {
            int min_observation_per_image = 10;
            bool refine_focal_length = false;
            double loss_function_scale = 1.0;
        };

        struct Statistics
        {
            bool is_succeed = false;
        };

        CeresBundleOptimizer(const Parameters &params);
        bool Optimize(BundleData &bundle_data);

    private:
        Parameters params_;
    };

}

#endif //SIMPLESFM_CERESBUNDLEOPTIMIZER_H
