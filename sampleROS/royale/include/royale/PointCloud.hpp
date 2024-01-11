/****************************************************************************\
 * Copyright (C) 2015 pmdtechnologies ag
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#pragma once

#include <cstdint>
#include <memory>
#include <royale/Definitions.hpp>
#include <royale/StreamId.hpp>
#include <royale/Vector.hpp>

namespace royale {
/**
 *  The point cloud gives XYZ and confidence for every valid
 *  point.
 *  It is given as an array of packed coordinate quadruplets (x,y,z,c)
 *  as floating point values. The x, y and z coordinates are in meters.
 *  The confidence (c) has a floating point value in [0.0, 1.0], where 1
 *  corresponds to full confidence.
 */
struct PointCloud {
    int64_t timestamp; //!< timestamp for the frame
    StreamId streamId; //!< stream which produced the data
    uint16_t width;    //!< width of depth image
    uint16_t height;   //!< height of depth image
    float *xyzcPoints; //!< XYZ and confidence for every valid point

    PointCloud() : xyzcPoints(nullptr),
                   isCopy(false) {
    }

    PointCloud &operator=(const PointCloud &dd) {
        if (this != &dd) {
            this->timestamp = dd.timestamp;
            this->streamId = dd.streamId;
            this->width = dd.width;
            this->height = dd.height;

            const auto numPixels = this->width * this->height;

            this->pcCopy.resize(numPixels);
            memcpy(&this->pcCopy[0], &dd.xyzcPoints[0], numPixels * sizeof(uint16_t));
            this->xyzcPoints = &this->pcCopy[0];

            isCopy = true;
        }
        return *this;
    }

    size_t getNumPoints() const {
        return this->width * this->height;
    }

    bool getIsCopy() const {
        return isCopy;
    }

  private:
    royale::Vector<float> pcCopy;
    bool isCopy;
};
} // namespace royale