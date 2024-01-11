/****************************************************************************\
 * Copyright (C) 2015 Infineon Technologies
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 \****************************************************************************/

#pragma once

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <royale/Definitions.hpp>
#include <royale/StreamId.hpp>
#include <royale/Vector.hpp>

namespace royale {
/**
 *  Encapsulates a 3D point in object space, with coordinates in meters.  In addition to the
 *  X/Y/Z coordinate each point also includes a gray value, a noise standard deviation, and a
 *  depth confidence value.
 */
struct DepthPoint {
    float x;                 //!< X coordinate [meters]
    float y;                 //!< Y coordinate [meters]
    float z;                 //!< Z coordinate [meters]
    float noise;             //!< noise value [meters]
    uint16_t grayValue;      //!< 16-bit gray value
    uint8_t depthConfidence; //!< value from 0 (invalid) to 255 (full confidence)
};

/**
 *  This structure defines the depth data which is delivered through the callback.
 *  This data comprises a dense 3D point cloud with the size of the depth image (width, height).
 *  The coordinates pointer gives you an array (row-based) with the size of (width x height x 4).
 *  For each pixel it will give you x, y and z (in meters) plus the confidence (in the range 0 (bad) to 1 (good)).
 *  Based on this confidence the user can decide to use the 3D point or not.
 *  The point cloud uses a right handed coordinate system (x -> right, y -> down, z -> in viewing direction).
 *
 */
struct DepthData {
    std::chrono::microseconds timeStamp;    //!< timestamp in microseconds precision (time since epoch 1970)
    StreamId streamId;                      //!< stream which produced the data
    uint16_t width;                         //!< width of depth image
    uint16_t height;                        //!< height of depth image
    royale::Vector<uint32_t> exposureTimes; //!< exposureTimes retrieved from CapturedUseCase
    bool hasDepth;                          //!< to check presence of depth information
    float illuminationTemperature;          //!< temperature of illumination

    float *coordinates;   //!< coordinates array with x, y, z and confidence for every pixel
    bool hasAmplitudes;   //!< to check presence of amplitude information
    uint16_t *amplitudes; //!< amplitude value for each pixel

    DepthData() : hasDepth(false),
                  coordinates(nullptr),
                  hasAmplitudes(false),
                  amplitudes(nullptr),
                  isCopy(false) {
    }

    DepthData &operator=(const DepthData &dd) {
        if (this != &dd) {
            this->timeStamp = dd.timeStamp;
            this->streamId = dd.streamId;
            this->width = dd.width;
            this->height = dd.height;
            this->hasDepth = dd.hasDepth;
            this->illuminationTemperature = dd.illuminationTemperature;
            this->hasAmplitudes = dd.hasAmplitudes;

            this->exposureTimes.resize(dd.exposureTimes.size());
            memcpy(&this->exposureTimes[0], &dd.exposureTimes[0], dd.exposureTimes.size() * sizeof(uint32_t));

            const auto numPixels = this->width * this->height;

            if (dd.hasDepth) {
                this->coordinatesCopy.resize(4 * numPixels);
                memcpy(&this->coordinatesCopy[0], &dd.coordinates[0], 4 * numPixels * sizeof(float));
                this->coordinates = &this->coordinatesCopy[0];
            }

            if (dd.hasAmplitudes) {
                this->amplitudesCopy.resize(numPixels);
                memcpy(&this->amplitudesCopy[0], &dd.amplitudes[0], numPixels * sizeof(uint16_t));
                this->amplitudes = &this->amplitudesCopy[0];
            }

            isCopy = true;
        }
        return *this;
    }

    /**
     *  Returns the maximal height supported by the camera device.
     */
    royale::DepthPoint getLegacyPoint(size_t idx) const {
        royale::DepthPoint point;

        if (hasDepth) {
            point.x = this->coordinates[idx * 4 + 0];
            point.y = this->coordinates[idx * 4 + 1];
            point.z = this->coordinates[idx * 4 + 2];
            point.depthConfidence = this->coordinates[idx * 4 + 3] < 1.0f ? static_cast<uint8_t>(this->coordinates[idx * 4 + 3] * 255.0f) : 255;
        } else {
            point.x = 0.0f;
            point.y = 0.0f;
            point.z = 0.0f;
            point.depthConfidence = 0;
        }

        if (hasAmplitudes) {
            point.grayValue = this->amplitudes[idx];
        } else {
            point.grayValue = 0u;
        }
        point.noise = 0.0f; // not supported anymore

        return point;
    }

    royale::Vector<royale::DepthPoint> getLegacyPoints() const {
        royale::Vector<royale::DepthPoint> points;

        uint32_t numPixels = this->height * this->width;
        points.resize(numPixels);

        DepthPoint *targetPoint = &points[0];
        for (auto i = 0u; i < numPixels; ++i, ++targetPoint) {
            *targetPoint = getLegacyPoint(i);
        }
        return points;
    }

    float getX(size_t idx) const {
        return this->coordinates[idx * 4 + 0];
    }

    float getY(size_t idx) const {
        return this->coordinates[idx * 4 + 1];
    }

    float getZ(size_t idx) const {
        return this->coordinates[idx * 4 + 2];
    }

    uint16_t getGrayValue(size_t idx) const {
        return this->amplitudes[idx];
    }

    float getDepthConfidence(size_t idx) const {
        return this->coordinates[idx * 4 + 3];
    }

    size_t getNumPoints() const {
        return this->width * this->height;
    }

    bool getIsCopy() const {
        return isCopy;
    }

  private:
    royale::Vector<float> coordinatesCopy;
    royale::Vector<uint16_t> amplitudesCopy;

    bool isCopy;
};
} // namespace royale
