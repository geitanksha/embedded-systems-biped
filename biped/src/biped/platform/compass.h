/**
 *  @file   compass.h
 *  @author Simon Yu
 *  @date   03/26/2023
 *  @brief  Compass class header.
 *
 *  This file defines the compass class.
 */

/*
 *  Include guard.
 */
#ifndef PLATFORM_COMPASS_H_
#define PLATFORM_COMPASS_H_

/*
 *  External headers.
 */
#include <Kalman.h>

/*
 *  Project headers.
 */
#include "common/type.h"

/*
 *  Biped namespace.
 */
namespace biped
{
/**
 *  @brief  Compass class.
 *
 *  This class provides functions for calibrating the compass,
 *  as well as for calculating attitude from the compass values.
 */
class Compass
{
public:

    /**
     *  @brief	Calibration data struct.
     *
     *  This struct contains calibration data entries.
     */
    struct Calibration
    {
        double offset_x;
        double offset_y;
        double offset_z;
        double scaler_x;
        double scaler_y;
        double scaler_z;
        double sign_x;
        double sign_y;
        double sign_z;

        /**
         *  @brief  Calibration data struct constructor.
         *
         *  This constructor initializes all calibration data struct entries.
         */
        Calibration() : offset_x(0), offset_y(0), offset_z(0), scaler_x(1), scaler_y(1),
                scaler_z(1), sign_x(1), sign_y(1), sign_z(1)
        {
        }
    };

    /**
     *  @brief  Compass class constructor.
     *
     *  This constructor initializes all class member variables.
     */
    Compass();

    /**
     *  @return Calibration data struct.
     *  @brief  Get the class member calibration data struct.
     *
     *  This function returns the class member calibration data struct.
     */
    Calibration
    getCalibration() const;

    /**
     *  @param  imu_data IMU data struct.
     *  @brief  Initialize compass.
     *
     *  This function initializes compass attitude calculation
     *  and Kalman filter.
     */
    void
    initialize(IMUData& imu_data);

    /**
     *  @brief  Start compass calibration.
     *
     *  This function starts the compass calibration process.
     */
    void IRAM_ATTR
    startCalibration();

    /**
     *  @param  imu_data IMU data struct.
     *  @brief  Execute compass calibration.
     *
     *  This function executes the compass calibration process.
     *  The function commands the controller for calibration
     *  and records the range of the compass data in the given
     *  IMU data struct. This function is expected to be called
     *  periodically.
     */
    void
    calibrate(IMUData& imu_data);

    /**
     *  @param  imu_data IMU data struct.
     *  @brief  Attitude calculation function.
     *
     *  This function calculates attitude data from the compass data
     *  in the given IMU data struct and populates the corresponding
     *  attitude entries in the given IMU data struct. The function
     *  also filters certain data using the Kalman filter.
     */
    void
    calculateAttitude(IMUData& imu_data);

private:

    /**
     *  @brief  Calculate compass calibration data.
     *
     *  This function calculate the compass calibration data after
     *  each compass calibration process and populates the
     *  corresponding entries in the member calibration data struct.
     */
    void
    calculateCalibration();

    /**
     *  @brief  End compass calibration process.
     *
     *  This function resets the controller and class member
     *  variables preparing for the next calibration.
     */
    void
    endCalibration();

    Calibration calibration_;   //!< Calibration object.
    bool calibration_started_;  //!< Calibration start flag.
    unsigned long calibration_time_point_start_; //!< Calibration start time point, in milliseconds.
    IMUData imu_data_upper_bound_;  //!< Upper bound IMU data struct.
    IMUData imu_data_lower_bound_;  //!< Lower bound IMU data struct.
    Kalman kalman_filter_attitude_z_;   //!< Z attitude Kalman filter object.
};
}   // namespace biped

#endif  // PLATFORM_COMPASS_H_
