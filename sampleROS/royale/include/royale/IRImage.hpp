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
 *  Infrared image with 8Bit mono information for every pixel
 */
struct IRImage {
    int64_t timestamp; //!< timestamp for the frame
    StreamId streamId; //!< stream which produced the data
    uint16_t width;    //!< width of depth image
    uint16_t height;   //!< height of depth image
    uint8_t *data;     //!< 8Bit mono IR image

    IRImage() : data(nullptr),
                isCopy(false) {
    }

    IRImage &operator=(const IRImage &dd) {
        if (this != &dd) {
            this->timestamp = dd.timestamp;
            this->streamId = dd.streamId;
            this->width = dd.width;
            this->height = dd.height;

            const auto numPixels = this->width * this->height;

            this->irCopy.resize(numPixels);
            memcpy(&this->irCopy[0], &dd.data[0], numPixels * sizeof(uint8_t));
            this->data = &this->irCopy[0];

            isCopy = true;
        }
        return *this;
    }

    uint8_t getIR(size_t idx) const {
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
    royale::Vector<uint8_t> irCopy;
    bool isCopy;
};

} // namespace royale