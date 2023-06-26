/**
 *  @file   sensor.h
 *  @author Simon Yu
 *  @date   01/12/2022
 *  @brief  Sensor class header.
 *
 *  This file defines the sensor class.
 */

/*
 *  Include guard.
 */
#ifndef SENSOR_SENSOR_H_
#define SENSOR_SENSOR_H_

/*
 *  Project headers.
 */
#include "platform/encoder.h"
#include "platform/imu.h"
#include "platform/time_of_flight.h"

/*
 *  Biped namespace.
 */
namespace biped
{
/*
 *  Forward declaration.
 */
class Actuator;

/**
 *  @brief  Sensor class.
 *
 *  This class provides functions for retrieving data
 *  from various sensors, such as the inertial measurement
 *  units (IMUs,) the motor encoders, and the time-of-flight
 *  sensors, and for populating data into the sensor data
 *  struct. The class also provides callback functions
 *  for interrupt-based sensors, such as the encoders.
 */
class Sensor
{
public:

    /**
     *  @brief  Sensor class constructor.
     *
     *  This constructor initializes all class member variables.
     *  Additionally, the constructor sets the related I/O pin
     *  modes and initializes the time-of-flight sensors.
     */
    Sensor();

    /**
     *  @return BMX160 compass calibration data struct.
     *  @brief  Get the BMX160 compass calibration data struct.
     *
     *  This function returns the BMX160 compass calibration data struct
     *  from the class member IMU object.
     */
    Compass::Calibration
    getCompassCalibrationBMX160() const;

    /**
     *  @return Encoder data struct.
     *  @brief  Get the encoder data struct.
     *
     *  This function returns the encoder data struct from the class
     *  member encoder object.
     */
    EncoderData
    getEncoderData() const;

    /**
     *  @return IMU data struct.
     *  @brief  Get the IMU data struct.
     *
     *  This function returns the IMU data struct from the class
     *  member IMU object.
     */
    IMUData
    getIMUDataBMX160() const;

    /**
     *  @return IMU data struct.
     *  @brief  Get the IMU data struct.
     *
     *  This function returns the IMU data struct from the class
     *  member IMU object.
     */
    IMUData
    getIMUDataMPU6050() const;

    /**
     *  @return Time-of-flight data struct.
     *  @brief  Get the class member time-of-flight data struct.
     *
     *  This function returns the class member time-of-flight data struct.
     */
    TimeOfFlightData
    getTimeOfFlightData() const;

    /**
     *  @param  fast_domain Whether to perform fast domain sensing.
     *  @brief  Sensor data acquisition function.
     *
     *  This function performs acquisition of all sensor data, for
     *  fast or slow domain. This function is expected to be called
     *  periodically.
     */
    void
    sense(const bool& fast_domain);

    /**
     *  @brief  Left encoder A callback function.
     *
     *  This function processes the interrupt from the left motor encoder A.
     */
    void IRAM_ATTR
    onEncoderLeftA();

    /**
     *  @brief  Left encoder B callback function.
     *
     *  This function processes the interrupt from the left motor encoder B.
     */
    void IRAM_ATTR
    onEncoderLeftB();

    /**
     *  @brief  Right encoder A callback function.
     *
     *  This function processes the interrupt from the right motor encoder A.
     */
    void IRAM_ATTR
    onEncoderRightA();

    /**
     *  @brief  Right encoder B callback function.
     *
     *  This function processes the interrupt from the right motor encoder B.
     */
    void IRAM_ATTR
    onEncoderRightB();

    /**
     *  @brief  Push button B callback function.
     *
     *  This function processes the interrupt from the push button B.
     */
    void IRAM_ATTR
    onPushButtonB();

private:

    Encoder encoder_;   //!< Encoder object.
    IMU imu_;   //!< IMU object.
    TimeOfFlightData time_of_flight_data_;  //!< Time-of-flight data struct.
    std::unique_ptr<TimeOfFlight> time_of_flight_left_; //!< Left Time-of-flight object unique pointer.
    std::unique_ptr<TimeOfFlight> time_of_flight_middle_; //!< Middle Time-of-flight object unique pointer.
    std::unique_ptr<TimeOfFlight> time_of_flight_right_; //!< Right Time-of-flight object unique pointer.
};
}   // namespace biped

#endif  // SENSOR_SENSOR_H_
