/**
 *  @file   compass.cpp
 *  @author Simon Yu
 *  @date   03/26/2023
 *  @brief  Compass class source.
 *
 *  This file implements the compass class.
 */

 /*
  *  External headers.
  */
#include <cmath>
#include <esp32-hal.h>
#include <limits>

  /*
   *  Project headers.
   */
#include "platform/compass.h"
#include "controller/controller.h"
#include "common/global.h"
#include "common/parameter.h"

   /*
    *  Biped namespace.
    */
namespace biped
{
    Compass::Compass() : calibration_started_(false), calibration_time_point_start_(0)
    {
        /*
         *  Initialize calibration data struct with parameters
         *  in compass parameter name space. To persist a compass
         *  calibration result, print the calibration data struct
         *  in the best-effort task, record the values, and update
         *  the parameters in the compass parameter name space.
         *  The calibration would then persist across reboots.
         */
        calibration_.offset_x = CompassParameter::calibration_offset_x;
        calibration_.offset_y = CompassParameter::calibration_offset_y;
        calibration_.offset_z = CompassParameter::calibration_offset_z;
        calibration_.scaler_x = CompassParameter::calibration_scaler_x;
        calibration_.scaler_y = CompassParameter::calibration_scaler_y;
        calibration_.scaler_z = CompassParameter::calibration_scaler_z;
        calibration_.sign_x = CompassParameter::calibration_sign_x;
        calibration_.sign_y = CompassParameter::calibration_sign_y;
        calibration_.sign_z = CompassParameter::calibration_sign_z;

        /*
         *  Initialize compass entries in upper bound IMU data
         *  struct to minimum double values.
         */
        imu_data_upper_bound_.compass_x = std::numeric_limits<double>::lowest();
        imu_data_upper_bound_.compass_y = std::numeric_limits<double>::lowest();
        imu_data_upper_bound_.compass_z = std::numeric_limits<double>::lowest();

        /*
         *  Initialize compass entries in lower bound IMU data
         *  struct to maximum double values.
         */
        imu_data_lower_bound_.compass_x = std::numeric_limits<double>::max();
        imu_data_lower_bound_.compass_y = std::numeric_limits<double>::max();
        imu_data_lower_bound_.compass_z = std::numeric_limits<double>::max();
    }

    Compass::Calibration
        Compass::getCalibration() const
    {
        /*
         *  Return class member calibration data struct.
         */
        return this->calibration_;
    }

    void
        Compass::initialize(IMUData& imu_data)
    {
        /*
         *  Perform initial attitude calculation.
         *  Do not call the calculateAttitude function here.
         *  See the comments in the calculateAttitude function
         *  for further instructions.
         */
        imu_data.attitude_z = atan2(imu_data.compass_x, imu_data.compass_y);
        //check if needs to be negated

        /*
         *  Configure Z attitude Kalman filter.
         */
        kalman_filter_attitude_z_.setAngle(radiansToDegrees(imu_data.attitude_z));
        kalman_filter_attitude_z_.setQangle(KalmanFilterParameter::q_angle);
        kalman_filter_attitude_z_.setQbias(KalmanFilterParameter::q_bias);
        kalman_filter_attitude_z_.setRmeasure(KalmanFilterParameter::r_measure);
    }

    void IRAM_ATTR
        Compass::startCalibration()
    {
        /*
         *  Set calibration start flag to true if not.
         */
        this->calibration_started_ = true;
    }

    void
        Compass::calibrate(IMUData& imu_data)
    {
        /*
         *  Declare static calibration end flag and set to true.
         */
        static bool calibration_ended = true;

        if (calibration_started_)
        {
            /*
             *  Update calibration start time point, set Z attitude controller
             *  reference to the calibration controller reference in compass
             *  parameter name space, enable Z attitude open-loop control, and
             *  update calibration end flag.
             */
            if (calibration_ended)
            {
                calibration_time_point_start_ = millis();

                if (controller_)
                {
                    ControllerReference controller_reference = controller_->getControllerReference();
                    controller_reference.attitude_z =
                        CompassParameter::calibration_controller_reference;
                    controller_->setControllerReference(controller_reference);
                    controller_->enableOpenLoopAttitudeZControl(true);
                }

                calibration_ended = false;
            }

            /*
             *  Record X compass data upper bound.
             */
            if (imu_data.compass_x > imu_data_upper_bound_.compass_x)
            {
                imu_data_upper_bound_.compass_x = imu_data.compass_x;
            }

            /*
             *  Record Y compass data upper bound.
             */
            if (imu_data.compass_y > imu_data_upper_bound_.compass_y)
            {
                imu_data_upper_bound_.compass_y = imu_data.compass_y;
            }

            /*
             *  Record Z compass data upper bound.
             */
            if (imu_data.compass_z > imu_data_upper_bound_.compass_z)
            {
                imu_data_upper_bound_.compass_z = imu_data.compass_z;
            }

            /*
             *  Record X compass data lower bound.
             */
            if (imu_data.compass_x < imu_data_lower_bound_.compass_x)
            {
                imu_data_lower_bound_.compass_x = imu_data.compass_x;
            }

            /*
             *  Record Y compass data lower bound.
             */
            if (imu_data.compass_y < imu_data_lower_bound_.compass_y)
            {
                imu_data_lower_bound_.compass_y = imu_data.compass_y;
            }

            /*
             *  Record Z compass data lower bound.
             */
            if (imu_data.compass_z < imu_data_lower_bound_.compass_z)
            {
                imu_data_lower_bound_.compass_z = imu_data.compass_z;
            }

            /*
             *  Stop calibration if the time passed is greater than
             *  the calibration time in the compass parameter name space.
             *  Otherwise, if the time passed is greater than half of the
             *  calibration time in the compass parameter name space,
             *  set Z attitude controller reference to negative of the
             *  calibration controller reference in compass parameter name
             *  space.
             */
            if (millisecondsToSeconds(millis() - calibration_time_point_start_)
                >= CompassParameter::calibration_time)
            {
                calibration_started_ = false;
            }
            else if (millisecondsToSeconds(millis() - calibration_time_point_start_)
                >= CompassParameter::calibration_time / 2.0)
            {
                if (controller_)
                {
                    ControllerReference controller_reference = controller_->getControllerReference();
                    controller_reference.attitude_z = -1
                        * CompassParameter::calibration_controller_reference;
                    controller_->setControllerReference(controller_reference);
                }
            }
        }
        else
        {
            /*
             *  Calculate calibration, end calibration, and
             *  update calibration end flag.
             */
            if (!calibration_ended)
            {
                calculateCalibration();
                endCalibration();

                calibration_ended = true;
            }

            /*
             *  Apply calibration data to the compass data.
             */
            imu_data.compass_x *= calibration_.scaler_x;
            imu_data.compass_y *= calibration_.scaler_y;
            imu_data.compass_z *= calibration_.scaler_z;
            imu_data.compass_x *= calibration_.sign_x;
            imu_data.compass_y *= calibration_.sign_y;
            imu_data.compass_z *= calibration_.sign_z;
            imu_data.compass_x += calibration_.offset_x;
            imu_data.compass_y += calibration_.offset_y;
            imu_data.compass_z += calibration_.offset_z;
        }
    }

    void
        Compass::calculateAttitude(IMUData& imu_data)
    {
        /*
         *  Calculate the raw Z attitude (yaw) data using
         *  the populated compass vectors in the member
         *  given IMU data struct. Refer to the following
         *  materials to correctly convert the calculated
         *  data into the standard body reference frame.
         *
         *  Note that the Z attitude (yaw) is the angle between
         *  a pair of compass vectors. Use atan2 function instead
         *  of atan for correct signedness.
         *
         *  Remember to perform the same calculation in the
         *  initialize function for initialization but populate the
         *  given IMU data struct using the raw data (unfiltered)
         *  directly, since the Kalman filter had not been initialized
         *  at that point in the initialize function.
         *
         *  Standard body reference frame:
         *  https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-refframes
         *
         *  Rotational right-hand rule:
         *  https://en.wikipedia.org/wiki/Right-hand_rule#Rotations
         */
        const double attitude_z_raw = atan2(imu_data.compass_x, imu_data.compass_y);
        // check if needs to be negated

        /*
         *  Filter the raw Z attitude data using the Kalman filter.
         */
        const double attitude_z_kalman_filter = kalman_filter_attitude_z_.getAngle(
            radiansToDegrees(attitude_z_raw), radiansToDegrees(imu_data.angular_velocity_z),
            PeriodParameter::fast);

        /*
         *  Convert the filtered Z attitude data back to radians
         *  using the degreesToRadians function, and populate
         *  the corresponding entry in the given IMU data struct.
         */
        imu_data.attitude_z = degreesToRadians(attitude_z_kalman_filter);
        // check if needs to be negated
    }

    void
        Compass::calculateCalibration()
    {
        /*
         *  Calculate compass vector ranges based on the recorded
         *  upper and lower bounds
         */
        const double compass_x_range = fabs(
            imu_data_upper_bound_.compass_x - imu_data_lower_bound_.compass_x);
        const double compass_y_range = fabs(
            imu_data_upper_bound_.compass_y - imu_data_lower_bound_.compass_y);
        const double compass_z_range = fabs(
            imu_data_upper_bound_.compass_z - imu_data_lower_bound_.compass_z);

        /*
         *  Calculate calibration scalers based on the calculated ranges and
         *  the calibration range in the compass parameter name space.
         */
        calibration_.scaler_x = CompassParameter::calibration_range / compass_x_range;
        calibration_.scaler_y = CompassParameter::calibration_range / compass_y_range;
        calibration_.scaler_z = CompassParameter::calibration_range / compass_z_range;

        /*
         *  Scale the compass data using the calculated calibration scalers.
         */
        imu_data_upper_bound_.compass_x *= calibration_.scaler_x;
        imu_data_upper_bound_.compass_y *= calibration_.scaler_y;
        imu_data_upper_bound_.compass_z *= calibration_.scaler_z;

        /*
         *  Calculate calibration offsets based on the recorded bounds and
         *  the calibration offset in the compass parameter name space.
         */
        calibration_.offset_x = (CompassParameter::calibration_range / 2.0)
            - imu_data_upper_bound_.compass_x;
        calibration_.offset_y = (CompassParameter::calibration_range / 2.0)
            - imu_data_upper_bound_.compass_y;
        calibration_.offset_z = (CompassParameter::calibration_range / 2.0)
            - imu_data_upper_bound_.compass_z;
    }

    void
        Compass::endCalibration()
    {
        /*
         *  Reset controller reference and disable Z attitude
         *  open-loop control.
         */
        if (controller_)
        {
            ControllerReference controller_reference = controller_->getControllerReference();
            controller_reference.attitude_z = 0;
            controller_->setControllerReference(controller_reference);
            controller_->enableOpenLoopAttitudeZControl(false);
        }

        /*
         *  Reset upper bound IMU data struct.
         */
        imu_data_upper_bound_.compass_x = std::numeric_limits<double>::lowest();
        imu_data_upper_bound_.compass_y = std::numeric_limits<double>::lowest();
        imu_data_upper_bound_.compass_z = std::numeric_limits<double>::lowest();

        /*
         *  Reset lower bound IMU data struct.
         */
        imu_data_lower_bound_.compass_x = std::numeric_limits<double>::max();
        imu_data_lower_bound_.compass_y = std::numeric_limits<double>::max();
        imu_data_lower_bound_.compass_z = std::numeric_limits<double>::max();

        /*
         *  Reset calibration start time point.
         */
        calibration_time_point_start_ = 0;
    }
}   // namespace biped
