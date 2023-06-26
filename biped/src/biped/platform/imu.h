/**
 *  @file   imu.h
 *  @author Simon Yu
 *  @date   01/05/2023
 *  @brief  IMU class header.
 *
 *  This file defines the IMU class.
 */

/*
 *  Include guard.
 */
#ifndef PLATFORM_IMU_H_
#define PLATFORM_IMU_H_

/*
 *  External headers.
 */
#include <Adafruit_MPU6050.h>
#include <DFRobot_BMX160.h>
#include <Kalman.h>

/*
 *  Project headers.
 */
#include "platform/compass.h"
#include "common/type.h"

/*
 *  Biped namespace.
 */
namespace biped
{
/**
 *  @brief  IMU class.
 *
 *  This class provides functions for reading from the
 *  inertial measurement units (IMUs.) The class also
 *  provides functions for calculating attitude, as well
 *  as callback functions for handling interrupts from
 *  push buttons.
 */
class IMU
{
public:

    /**
     *  @brief  IMU class constructor.
     *
     *  This constructor initializes all class member variables.
     *  Additionally, the constructor initializes the IMU drivers
     *  and configures the IMUs.
     */
    IMU();

    /**
     *  @return BMX160 compass calibration data struct.
     *  @brief  Get the BMX160 compass calibration data struct.
     *
     *  This function returns the BMX160 compass calibration data struct
     *  from the class member BMX160 compass object.
     */
    Compass::Calibration
    getCompassCalibrationBMX160() const;

    /**
     *  @return IMU data struct.
     *  @brief  Get the class member BMX160 IMU data struct.
     *
     *  This function returns the class member BMX160 IMU data struct.
     */
    IMUData
    getDataBMX160() const;

    /**
     *  @return IMU data struct.
     *  @brief  Get the class member MPU6050 IMU data struct.
     *
     *  This function returns the class member MPU6050 IMU data struct.
     */
    IMUData
    getDataMPU6050() const;

    /**
     *  @brief  BMX160 reading function.
     *
     *  This function reads data from the BMX160 IMU, populates the
     *  corresponding entries in the member BMX160 IMU data struct,
     *  executes the BMX160 compass calibration, and calculates
     *  attitude.
     */
    void
    readBMX160();

    /**
     *  @brief  MPU6050 reading function.
     *
     *  This function reads data from the MPU6050 IMU, populates the
     *  corresponding entries in the member MPU6050 IMU data struct,
     *  and calculates attitude.
     */
    void
    readMPU6050();

    /**
     *  @brief  Push button B callback function.
     *
     *  This function processes the interrupt from push button B.
     */
    void IRAM_ATTR
    onPushButtonB();

private:

    /**
     *  @brief  Initialize BMX160 IMU.
     *
     *  This constructor initializes the BMX160 IMU driver, configures
     *  the BMX160 IMU, performs an initial read from the IMU, initializes
     *  the BMX160 compass, performs initial attitude calculation, and
     *  configures the Kalman filters.
     */
    void
    initializeBMX160();

    /**
     *  @brief  Initialize MPU6050 IMU.
     *
     *  This constructor initializes the MPU6050 IMU driver, configures
     *  the MPU6050 IMU, performs an initial read from the IMU and an
     *  initial attitude calculation, and configures the Kalman filters.
     */
    void
    initializeMPU6050();

    /**
     *  @brief  BMX160 attitude calculation function.
     *
     *  This function calculates attitude data from acceleration
     *  data and populates the corresponding entries in the member
     *  BMX160 IMU data struct. The function also filters certain data
     *  using the Kalman filter.
     */
    void
    calculateAttitudeBMX160();

    /**
     *  @brief  MPU6050 attitude calculation function.
     *
     *  This function calculates attitude data from acceleration
     *  data and populates the corresponding entries in the member
     *  MPU6050 IMU data struct. The function also filters certain
     *  data using the Kalman filter.
     */
    void
    calculateAttitudeMPU6050();

    DFRobot_BMX160 bmx160_; //!< DFRobot BMX160 IMU driver object.
    Compass bmx160_compass_;    //!< BMX160 compass object.
    IMUData bmx160_data_;    //!< BMX160 IMU data struct.
    Kalman bmx160_kalman_filter_attitude_y_;  //!< BMX160 Y attitude (pitch) Kalman filter object.

    Adafruit_MPU6050 mpu6050_;  //!< Adafruit MPU6050 IMU driver object.
    IMUData mpu6050_data_;    //!< MPU6050 IMU data struct.
    Kalman mpu6050_kalman_filter_attitude_y_;  //!< MPU6050 Y attitude (pitch) Kalman filter object.
};
}   // namespace biped

#endif  // PLATFORM_IMU_H_
