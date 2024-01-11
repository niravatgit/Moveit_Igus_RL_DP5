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
 *  The depth image represents the depth and confidence for every pixel.
 *  The least significant 13 bits are the depth (z value along the optical axis) in
 *  millimeters. 0 stands for invalid measurement / no data.
 *
 *  The most significant 3 bits correspond to a confidence value. 0 is the highest confidence, 7
 *  the second highest, and 1 the lowest.
 *
 *  Exception Note: For use-cases with UR larger than 8.191 m all 16 bits are used and no
 *  confidence information is available.
 */
struct DepthImage {
    int64_t timestamp;  //!< timestamp for the frame
    StreamId streamId;  //!< stream which produced the data
    uint16_t width;     //!< width of depth image
    uint16_t height;    //!< height of depth image
    uint16_t *data;     //!< depth and confidence for the pixel
    bool hasConfidence; //!< to check presence of confidence information

    DepthImage() : data(nullptr),
                   hasConfidence(false),
                   isCopy(false) {
    }

    DepthImage &operator=(const DepthImage &dd) {
        if (this != &dd) {
            this->timestamp = dd.timestamp;
            this->streamId = dd.streamId;
            this->width = dd.width;
            this->height = dd.height;
            this->hasConfidence = dd.hasConfidence;
            const auto numPixels = this->width * this->height;

            this->depthCopy.resize(numPixels);
            memcpy(&this->depthCopy[0], &dd.data[0], numPixels * sizeof(uint16_t));
            this->data = &this->depthCopy[0];

            isCopy = true;
        }
        return *this;
    }

    uint16_t getDepth(size_t idx) const {
        if (this->data) {
            return this->data[idx];
        } else {
            return 0u;
        }
    }

    size_t getNumPoints() const {
        return this->width * this->height;
    }

    bool getIsCopy() const {
        return isCopy;
    }

  private:
    royale::Vector<uint16_t> depthCopy;
    bool isCopy;
};
} // namespace royale
