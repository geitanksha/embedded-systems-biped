/**
 *  @file   neopixel.h
 *  @author Simon Yu
 *  @date   04/01/2022
 *  @brief  NeoPixel class header.
 *
 *  This file defines the NeoPixel class.
 */

/*
 *  Include guard.
 */
#ifndef PLATFORM_NEOPIXEL_H_
#define PLATFORM_NEOPIXEL_H_

/*
 *  External headers.
 */
#include <Adafruit_NeoPixel.h>
#include <ArduinoEigenDense.h>
#include <memory>
#include <vector>

/*
 *  Biped namespace.
 */
namespace biped
{
/**
 *  @brief  NeoPixel class.
 *
 *  This class provides functions for
 *  controlling the NeoPixel LED array.
 */
class NeoPixel
{
public:

    using Frame = std::vector<Eigen::Vector3i>;

    /**
     *  @brief  NeoPixel class constructor.
     *
     *  This constructor initializes all class member variables.
     *  Additionally, the constructor initializes the NeoPixel
     *  driver and clears the NeoPixel LED array.
     */
    NeoPixel();

    /**
     *  @param  brightness LED brightness.
     *  @brief  Set the brightness of the NeoPixel LED array.
     *
     *  This function sets the brightness of the NeoPixel
     *  LED array. The brightness value is clamped between
     *  the minimum and maximum LED brightness values defined
     *  in the NeoPixel parameter namespace.
     */
    void
    setBrightness(const int& brightness);

    /**
     *  @param  frame NeoPixel frame.
     *  @brief  Set the NeoPixel frame.
     *
     *  This function sets the NeoPixel frame.
     */
    void
    setFrame(const std::shared_ptr<Frame> frame);

    /**
     *  @brief  Clear the NeoPixel LED array.
     *
     *  This function clears the NeoPixel LED array.
     */
    void
    clear();

    /**
     *  @brief  Show the NeoPixel frames to the NeoPixel LED array.
     *
     *  This function shows class member NeoPixel frames to the
     *  NeoPixel LED array. The frame showed depends on the current
     *  worst log level. This function is expected to be called
     *  periodically.
     */
    void
    show();

private:

    std::shared_ptr<Frame> frame_;  //!< NeoPixel frame shared pointer.
    std::shared_ptr<Frame> frame_error_;    //!< NeoPixel error frame shared pointer.
    Adafruit_NeoPixel neopixel_;    //!< Adafruit NeoPixel driver object.
};
}   // namespace biped

#endif  // PLATFORM_NEOPIXEL_H_
