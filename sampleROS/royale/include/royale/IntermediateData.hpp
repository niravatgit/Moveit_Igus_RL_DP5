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

#include <royale/Definitions.hpp>
#include <royale/DepthData.hpp>
#include <royale/ProcessingFlag.hpp>
#include <royale/StreamId.hpp>
#include <royale/Vector.hpp>

#include <chrono>
#include <cstdint>
#include <memory>

namespace royale {
/**
 *  DEPRECATED : In addition to the standard depth point, the intermediate point also stores
 *  information which is calculated as temporaries in the processing pipeline.
 *  Distance : Radial distance for each point (in meter)
 *  Amplitude : Grayscale image that also provides a hint on the amount of reflected light.
 *              The values are positive, but the range depends on the camera that is used.
 *  Intensity : Intensity image (values can be negative in some cases)
 *  Flags : Flag image that shows invalid pixels. For a description of the flags please refer
 *          to the documentation you receive after getting level 2 access from pmd.
 */
struct IntermediatePoint {
    float distance;  //!< radial distance of the current pixel
    float amplitude; //!< amplitude value of the current pixel
    float intensity; //!< intensity value of the current pixel
    uint32_t flags;  //!< flag value of the current pixel
};

/**
 *  This structure defines the Intermediate depth data which is delivered through the callback
 *  if the user has access level 2 for the CameraDevice.
 */
struct IntermediateData {
    std::chrono::microseconds timeStamp;            //!< timestamp in microseconds precision (time since epoch 1970)
    StreamId streamId;                              //!< stream which produced the data
    uint16_t width;                                 //!< width of distance image
    uint16_t height;                                //!< height of distance image
    royale::Vector<uint32_t> modulationFrequencies; //!< modulation frequencies for each sequence
    royale::Vector<uint32_t> exposureTimes;         //!< integration times for each sequence
    uint32_t numFrequencies;                        //!< number of processed frequencies
    royale::Vector<size_t> rawFrameCount;           //!< raw frame count of each exposure group
    ProcessingParameterMap processingParameters;    //!< processing Parameters used

    bool hasFlags;
    uint32_t *flags;
    bool hasIntensities;
    float *intensities;
    bool hasDistances;
    float *distances;
    bool hasAmplitudes;
    float *amplitudes;
    bool hasNoise;
    float *noise;

    IntermediateData() : hasFlags(false),
                         flags(nullptr),
                         hasIntensities(false),
                         intensities(nullptr),
                         hasDistances(false),
                         distances(nullptr),
                         hasAmplitudes(false),
                         amplitudes(nullptr),
                         hasNoise(false),
                         noise(nullptr),
                         isCopy(false) {
    }

    IntermediateData &operator=(const IntermediateData &dd) {
        if (this != &dd) {
            this->timeStamp = dd.timeStamp;
            this->streamId = dd.streamId;
            this->width = dd.width;
            this->height = dd.height;

            this->modulationFrequencies.resize(dd.modulationFrequencies.size());
            memcpy(&this->modulationFrequencies[0], &dd.modulationFrequencies[0], dd.modulationFrequencies.size() * sizeof(uint32_t));

            this->exposureTimes.resize(dd.exposureTimes.size());
            memcpy(&this->exposureTimes[0], &dd.exposureTimes[0], dd.exposureTimes.size() * sizeof(uint32_t));

            this->numFrequencies = dd.numFrequencies;

            const auto numPixels = this->width * this->height;

            this->hasFlags = dd.hasFlags;
            if (dd.hasFlags) {
                this->flagsCopy.resize(numPixels);
                memcpy(&this->flagsCopy[0], &dd.flags[0], numPixels * sizeof(uint32_t));
                this->flags = &this->flagsCopy[0];
            }

            this->hasIntensities = dd.hasIntensities;
            if (dd.hasIntensities) {
                this->intensitiesCopy.resize(numPixels);
                memcpy(&this->intensitiesCopy[0], &dd.intensities[0], numPixels * sizeof(float));
                this->intensities = &this->intensitiesCopy[0];
            }

            this->hasDistances = dd.hasDistances;
            if (dd.hasDistances) {
                this->distancesCopy.resize(numPixels);
                memcpy(&this->distancesCopy[0], &dd.distances[0], numPixels * sizeof(float));
                this->distances = &this->distancesCopy[0];
            }

            this->hasAmplitudes = dd.hasAmplitudes;
            if (dd.hasAmplitudes) {
                this->amplitudesCopy.resize(numPixels);
                memcpy(&this->amplitudesCopy[0], &dd.amplitudes[0], numPixels * sizeof(float));
                this->amplitudes = &this->amplitudesCopy[0];
            }

            this->hasNoise = dd.hasNoise;
            if (dd.hasNoise) {
                this->noiseCopy.resize(numPixels);
                memcpy(&this->noiseCopy[0], &dd.noise[0], numPixels * sizeof(float));
                this->noise = &this->noiseCopy[0];
            }

            isCopy = true;
        }
        return *this;
    }

    royale::IntermediatePoint getLegacyPoint(size_t idx) const {
        royale::IntermediatePoint point;

        if (hasIntensities) {
            point.intensity = this->intensities[idx];
        } else {
            point.intensity = 0.0f;
        }

        if (hasDistances) {
            point.distance = this->distances[idx];
        } else {
            point.distance = 0.0f;
        }

        if (hasAmplitudes) {
            point.amplitude = this->amplitudes[idx];
        } else {
            point.amplitude = 0.0f;
        }

        if (hasFlags) {
            point.flags = this->flags[idx];
        } else {
            point.flags = 0u;
        }

        return point;
    }

    royale::Vector<royale::IntermediatePoint> getLegacyPoints() const {
        royale::Vector<royale::IntermediatePoint> points;

        uint32_t numPixels = this->height * this->width;

        points.resize(numPixels);

        IntermediatePoint *targetPoint = &points[0];

        for (auto i = 0u; i < numPixels; ++i, ++targetPoint) {
            *targetPoint = getLegacyPoint(i);
        }

        return points;
    }

    float getDistance(size_t idx) const {
        if (hasDistances) {
            return this->distances[idx];
        } else {
            return 0.0f;
        }
    }

    float getAmplitude(size_t idx) const {
        if (hasAmplitudes) {
            return this->amplitudes[idx];
        } else {
            return 0.0f;
        }
    }

    float getIntensity(size_t idx) const {
        if (hasIntensities) {
            return this->intensities[idx];
        } else {
            return 0.0f;
        }
    }

    float getNoise(size_t idx) const {
        if (hasNoise) {
            return this->noise[idx];
        } else {
            return 0.0f;
        }
    }

    uint32_t getFlags(size_t idx) const {
        if (hasFlags) {
            return this->flags[idx];
        } else {
            return 0u;
        }
    }

    size_t getNumPoints() const {
        return this->width * this->height;
    }

    // Retrieve the number of raw frames
    const size_t getNumberOfRawFrames() const {
        size_t num = 0u;
        for (auto curGroup : rawFrameCount) {
            num += curGroup;
        }
        return num;
    }

    bool getIsCopy() const {
        return isCopy;
    }

  private:
    royale::Vector<uint32_t> flagsCopy;
    royale::Vector<float> intensitiesCopy;
    royale::Vector<float> distancesCopy;
    royale::Vector<float> amplitudesCopy;
    royale::Vector<float> noiseCopy;

    bool isCopy;
};
} // namespace royale
