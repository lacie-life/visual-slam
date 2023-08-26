//
// Created by lacie on 22/08/2023.
//

#ifndef SIMPLESFM_TRACK_H
#define SIMPLESFM_TRACK_H

#include <cstddef>
#include <cassert>
#include <vector>
#include <algorithm>

#include "Utils/Types.h"

namespace SimpleSfM {

    struct TrackElement
    {
        TrackElement() {}
        TrackElement(image_t image_id, point2D_t point2D_idx)
                : image_id(image_id), point2D_idx(point2D_idx) {}

        image_t image_id;
        point2D_t point2D_idx;
    };

    class Track
    {
    public:
        ////////////////////////////////////////////////////////////////////////////////
        // Get the length of the track
        ////////////////////////////////////////////////////////////////////////////////
        size_t Length() const;

        ////////////////////////////////////////////////////////////////////////////////
        // Get the whole track
        ////////////////////////////////////////////////////////////////////////////////
        const std::vector<TrackElement> &Elements() const;
        std::vector<TrackElement> &Elements();

        ////////////////////////////////////////////////////////////////////////////////
        // Get the elements in the track
        ////////////////////////////////////////////////////////////////////////////////
        const TrackElement &Element(const size_t idx) const;
        TrackElement &Element(const size_t idx);

        ////////////////////////////////////////////////////////////////////////////////
        // Add elements to track
        ////////////////////////////////////////////////////////////////////////////////
        void AddElement(const TrackElement &element);
        void AddElement(const image_t image_id, const point2D_t point2D_idx);
        void AddElements(const std::vector<TrackElement> &elements);

        ////////////////////////////////////////////////////////////////////////////////
        // remove element from track
        ////////////////////////////////////////////////////////////////////////////////
        void DeleteElement(const size_t idx);
        void DeleteElement(const TrackElement &element);
        void DeleteElement(const image_t image_id, const point2D_t point2D_idx);

    private:
        std::vector<TrackElement> elements_;
    };
}



#endif //SIMPLESFM_TRACK_H
