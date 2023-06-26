/**
 *  @file   sensor.cpp
 *  @author Simon Yu
 *  @date   01/12/2022
 *  @brief  Sensor class source.
 *
 *  This file implements the sensor class.
 */

 /*
  *  Project headers.
  */
#include "common/global.h"
#include "platform/io_expander.h"
#include "common/parameter.h"
#include "common/pin.h"
#include "sensor/sensor.h"

  /*
   *  Biped namespace.
   */
namespace biped
{
    Sensor::Sensor()
    {
        /*
         *  Set pin mode for the time-of-flight shutdown pins using
         *  the I/O expander pin mode functions.
         *  See the parameter header for details.
         */
        if (io_expander_a_) {
            io_expander_a_->pinModePortA(IOExpanderAPortAPin::time_of_flight_left_shutdown, OUTPUT);
            io_expander_a_->pinModePortB(IOExpanderAPortBPin::time_of_flight_middle_shutdown, OUTPUT);
            io_expander_a_->pinModePortB(IOExpanderAPortBPin::time_of_flight_right_shutdown, OUTPUT);
        }

        /*
         *  Instantiate all time-of-flight objects and store
         *  their unique pointers.
         *  See the parameter header for details.
         */
        time_of_flight_left_ = std::unique_ptr<TimeOfFlight>(
            new TimeOfFlight(AddressParameter::time_of_flight_left,
                IOExpanderAPortAPin::time_of_flight_left_shutdown, io_expander_a_));
        time_of_flight_middle_ = std::unique_ptr<TimeOfFlight>(
            new TimeOfFlight(AddressParameter::time_of_flight_middle,
                IOExpanderAPortBPin::time_of_flight_middle_shutdown
                + IOExpanderParameter::num_port_pins, io_expander_a_));
        time_of_flight_right_ = std::unique_ptr<TimeOfFlight>(
            new TimeOfFlight(AddressParameter::time_of_flight_right,
                IOExpanderAPortBPin::time_of_flight_right_shutdown
                + IOExpanderParameter::num_port_pins, io_expander_a_));
    }

    Compass::Calibration
        Sensor::getCompassCalibrationBMX160() const
    {
        /*
         *  Get BMX160 compass calibration struct from the
         *  member IMU object and return the struct.
         */

        return this->imu_.getCompassCalibrationBMX160();
    }

    EncoderData
        Sensor::getEncoderData() const
    {
        /*
         *  Get encoder data struct from the member encoder
         *  object and return the struct.
         */
        return this->encoder_.getData();
    }

    IMUData
        Sensor::getIMUDataBMX160() const
    {
        /*
         *  Get BMX160 IMU data struct from the member IMU
         *  object and return the struct.
         */
        return this->imu_.getDataBMX160();
    }

    IMUData
        Sensor::getIMUDataMPU6050() const
    {
        /*
         *  Get MPU6050 IMU data struct from the member IMU
         *  object and return the struct.
         */
        return this->imu_.getDataMPU6050();
    }

    TimeOfFlightData
        Sensor::getTimeOfFlightData() const
    {
        /*
         *  Return the class member time-of-flight data struct.
         */
        return this->time_of_flight_data_;
    }

    void
        Sensor::sense(const bool& fast_domain)
    {
        /*
         *  The fast domain tasks are the ones that requires
         *  faster execution frequency, such as the reading of
         *  the IMU, the attitude calculation, and etc. If such
         *  tasks were performed too infrequently, the delay might
         *  cause inaccurate or delayed data sampling. For example, if
         *  the attitude was calculated too infrequently, then it
         *  is possible that the plant has already moved to another
         *  pose during the gap between the calculations, which could
         *  cause, for instance, Biped failing to perfectly
         *  balance itself due to the delayed sample.
         *
         *  The slow domain tasks, on the other hand, are the ones
         *  that are better executed at lower frequency due to sensor noise
         *  or performance issues. For example, the X velocity is
         *  calculated as the number of encoder steps per unit time.
         *  The smaller the period, the noisier the velocity.
         *  Additionally, it is unnecessary to read non-essential sensors
         *  such as the time-of-flight sensors at higher frequency. Thus, it's
         *  more performance-friendly to process such tasks in slow domain.
         */
        if (fast_domain)
        {
            /*
             *  Read encoders.
             */
            this->encoder_.read();

            /*
             *  Read BMX160 IMU.
             */
            this->imu_.readBMX160();

            /*
             *  Read MPU6050 IMU.
             */
            this->imu_.readMPU6050();
        }
        else
        {
            /*
             *  Calculate velocity.
             */
            this->encoder_.calculateVelocity();

            /*
             *  Read time-of-flight sensors.
             */
            time_of_flight_data_.range_left = this->time_of_flight_left_->read();
            time_of_flight_data_.range_middle = this->time_of_flight_middle_->read();
            time_of_flight_data_.range_right = this->time_of_flight_right_->read();
        }
    }

    void IRAM_ATTR
        Sensor::onEncoderLeftA()
    {
        /*
         *  Call left encoder A callback function.
         *  See the encoder class for details.
         */
        encoder_.onLeftA();
    }

    void IRAM_ATTR
        Sensor::onEncoderLeftB()
    {
        /*
         *  Call left encoder B callback function.
         *  See the encoder class for details.
         */
        encoder_.onLeftB();
    }

    void IRAM_ATTR
        Sensor::onEncoderRightA()
    {
        /*
         *  Call right encoder A callback function.
         *  See the encoder class for details.
         */
        encoder_.onRightA();
    }

    void IRAM_ATTR
        Sensor::onEncoderRightB()
    {
        /*
         *  Call right encoder B callback function.
         *  See the encoder class for details.
         */
        encoder_.onRightB();
    }

    void IRAM_ATTR
        Sensor::onPushButtonB()
    {
        /*
         *  Call push button B callback function.
         *  See the IMU class for details.
         */
        imu_.onPushButtonB();
    }
}   // namespace biped
