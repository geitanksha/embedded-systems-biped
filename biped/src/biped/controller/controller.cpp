/**
 *  @file   controller.cpp
 *  @author Simon Yu
 *  @date   01/13/2022
 *  @brief  Controller class source.
 *
 *  This file implements the controller class.
 */

 /*
  *  Project headers.
  */
#include "controller/controller.h"
#include "platform/display.h"
#include "common/global.h"
#include "utility/math.h"
#include "common/parameter.h"
#include "sensor/sensor.h"

  /*
   *  Biped namespace.
   */
namespace biped
{
    Controller::Controller() : active_(false), open_loop_attitude_z_control_enabled_(false),
        output_position_x_(0), output_attitude_y_(0), output_attitude_z_(0)
    {
        /*
         *  Set Z attitude (yaw) open loop controller gain.
         *
         *  Tune this controller only after successfully tuning both the Y
         *  attitude (pitch) controller and the X position (forward/backward)
         *  controller. Also, you should have successfully tuned the input
         *  saturation bounds below. Before that, set the gain of this
         *  controller to 0.
         *
         *  The attitude (yaw) open loop controller is used for calibrating
         *  the compass by providing a simple open-loop yaw control to spin
         *  Biped around the Z axis. During the spin, the compass object
         *  performs the calibration for the compass, and data from calibrated
         *  compass could subsequently be used for closed-loop PID yaw control.
         *
         *  For this controller, simply set the magnitude of the gain to be 1.
         *  For the signedness, however, change the sign of the gain such that
         *  a positive reference would result in a positive spin around the Z
         *  axis.
         */
        open_loop_controller_gain_attitude_z_ = ControllerParameter::attitude_z_gain_open_loop;

        /*
         *  Populate X position (forward/backward) PID controller gain struct.
         *
         *  Tune this controller only after successfully tuning the Y attitude
         *  (pitch) controller. Before that, set all gains of this controller to 0.
         *
         *  For this controller, all gains should be positive. If your system
         *  diverges with positive gains, make sure you have all sensor data
         *  in the standard body reference frame, and you have subtracted state
         *  with reference (Y - R), not the other way around, in the PID
         *  controller control function.
         *
         *  While tuning, start increasing the differential gain first. After the
         *  controller is able resist some velocity change (damping),
         *  then start increasing the proportional gain until Biped is able to
         *  track the X position reference when pushed along the X axis
         *  (X position control).
         *
         *  The integral gain tuning is optional.
         *
         *  See Lecture 13 for the PID controller tuning heuristics.
         */
        pid_controller_gain_position_x_.proportional =
            ControllerParameter::position_x_gain_proportional;
        pid_controller_gain_position_x_.differential =
            ControllerParameter::position_x_gain_differential;
        pid_controller_gain_position_x_.integral = ControllerParameter::position_x_gain_integral;
        pid_controller_gain_position_x_.integral_max =
            ControllerParameter::position_x_gain_integral_max;

        /*
         *  Populate Y attitude (pitch) PID controller gain struct.
         *
         *  Tune this controller first. Before start, set all the gains of
         *  all controllers to 0, including this one.
         *
         *  For this controller, all gains should be negative (except for
         *  max integral). If your system diverges with positive gains, make
         *  sure you have all sensor data in the standard body reference frame,
         *  and you have subtracted state with reference (Y - R), not the other
         *  way around, in the PID controller control function.
         *
         *  While tuning, start with the proportional gain first. Increase the
         *  magnitude of the proportional gain until Biped starts mildly
         *  oscillating around the Y axis (marginally stable). Then, start
         *  increasing the magnitude of the differential gain until any
         *  oscillations disappears (damping).
         *
         *  The integral gain tuning is optional.
         *
         *  See Lecture 13 for the PID controller tuning heuristics.
         */
        pid_controller_gain_attitude_y_.proportional =
            ControllerParameter::attitude_y_gain_proportional;
        pid_controller_gain_attitude_y_.differential =
            ControllerParameter::attitude_y_gain_differential;
        pid_controller_gain_attitude_y_.integral = ControllerParameter::attitude_y_gain_integral;
        pid_controller_gain_attitude_y_.integral_max =
            ControllerParameter::attitude_y_gain_integral_max;

        /*
         *  Populate Z attitude (yaw) PID controller gain struct.
         *
         *  Tune this controller last. By now, you should have successfully
         *  tuned all other controllers. Also, you should have successfully
         *  calibrated the compass using the open-loop controller and have
         *  tuned the input saturation bounds below. Before that, set all gains
         *  of this controller to 0.
         *
         *  For this controller, all gains should be positive. If your system
         *  diverges with positive gains, make sure you have all sensor data
         *  in the standard body reference frame, and you have subtracted
         *  state with reference (Y - R), not the other way around, in the PID
         *  controller control function.
         *
         *  While tuning, start with the proportional gain first. Increase the
         *  magnitude of the proportional gain until Biped starts mildly
         *  oscillating around the Z axis (marginally stable). Then, start
         *  increasing the magnitude of the differential gain until any
         *  oscillations disappears (damping).
         *
         *  The integral gain tuning is optional.
         *
         *  See Lecture 13 for the PID controller tuning heuristics.
         */
        pid_controller_gain_attitude_z_.proportional =
            ControllerParameter::attitude_z_gain_proportional;
        pid_controller_gain_attitude_z_.differential =
            ControllerParameter::attitude_z_gain_differential;
        pid_controller_gain_attitude_z_.integral = ControllerParameter::attitude_z_gain_integral;
        pid_controller_gain_attitude_z_.integral_max =
            ControllerParameter::attitude_z_gain_integral_max;

        /*
         *  Set the controller gains to their respective controllers.
         */
        open_loop_controller_attitude_z_.setGain(open_loop_controller_gain_attitude_z_);
        pid_controller_attitude_y_.setGain(pid_controller_gain_attitude_y_);
        pid_controller_attitude_z_.setGain(pid_controller_gain_attitude_z_);
        pid_controller_position_x_.setGain(pid_controller_gain_position_x_);

        /*
         *  Adjust the input saturation bounds such that your Biped
         *  is able to go forward and backward with moderate speed
         *  under the command of the waypoint planner example plan.
         *
         *  The lower bound controls the forward saturation and should
         *  be a negative value. The upper bound controls the backward
         *  saturation and should be a positive value. The bounds in
         *  turn also control the velocity of Biped (too fast Biped
         *  would fail to balance, too slow Biped would fail to overcome
         *  disturbance (friction.) The bounds are also affected by the
         *  center of gravity of Biped, and each Biped might have slightly
         *  different center of gravity. Therefore, it is important to
         *  stick with the same Biped throughout the remaining labs.
         *
         *  Why does the controller input need to be saturated? Let's
         *  say your Biped is current at 0 meter X position, and your
         *  reference is 1 meter. The magnitude of the error (e) would
         *  simply be 1. However, if your reference is instead 1000 meters,
         *  then the magnitude of the error (e) would be 1000, which would be
         *  around four times the maximum motor PWM values. Therefore, the
         *  controller input needs to be saturated within a limited range
         *  to prevent excessive control output.
         */
        pid_controller_saturation_position_x_.input_lower =
            ControllerParameter::position_x_saturation_input_lower;
        pid_controller_saturation_position_x_.input_upper =
            ControllerParameter::position_x_saturation_input_upper;

        /*
         *  Set the controller saturations to their respective controllers.
         */
        pid_controller_position_x_.setSaturation(pid_controller_saturation_position_x_);
        // pid_controller_saturation_attitude_y_.input_lower = 
        // pid_controller_saturation_attitude_y_.input_upper = 
        // pid_controller_saturation_attitude_z.input_lower = 
        // pid_controller_saturation_attitude_z.input_upper = 

        /*
         *  Set controller reference to be the current member controller
         *  reference struct.
         */
        open_loop_controller_attitude_z_.setReference(controller_reference_.attitude_z);
        pid_controller_attitude_y_.setReference(controller_reference_.attitude_y);
        pid_controller_attitude_z_.setReference(controller_reference_.attitude_z);
        pid_controller_position_x_.setReference(controller_reference_.position_x);

        /*
         *  Initialize NeoPixel frame for controller active status.
         */
        neopixel_frame_active_ = std::make_shared<NeoPixel::Frame>();
        neopixel_frame_active_->push_back(Eigen::Vector3i(0, 255, 0));
        neopixel_frame_active_->push_back(Eigen::Vector3i(0, 255, 0));
        neopixel_frame_active_->push_back(Eigen::Vector3i(0, 255, 0));
        neopixel_frame_active_->push_back(Eigen::Vector3i(0, 255, 0));

        /*
         *  Initialize NeoPixel frame for controller inactive status.
         */
        neopixel_frame_inactive_ = std::make_shared<NeoPixel::Frame>();
        neopixel_frame_inactive_->push_back(Eigen::Vector3i(0, 0, 255));
        neopixel_frame_inactive_->push_back(Eigen::Vector3i(0, 0, 255));
        neopixel_frame_inactive_->push_back(Eigen::Vector3i(0, 0, 255));
        neopixel_frame_inactive_->push_back(Eigen::Vector3i(0, 0, 255));
    }

    ActuationCommand
        Controller::getActuationCommand() const
    {
        /*
         *  Return the class member actuation command struct.
         */
        return this->actuation_command_;
    }

    bool
        Controller::getActiveStatus() const
    {
        /*
         *  Return the controller active flag.
         */
        return this->active_;
    }

    ControllerReference
        Controller::getControllerReference() const
    {
        /*
         *  Return the class member controller reference struct.
         */
        return this->controller_reference_;
    }

    void
        Controller::setControllerReference(const ControllerReference& controller_reference)
    {
        /*
         *  Store the given controller reference struct in the
         *  function parameter to the member controller
         *  reference struct.
         */
        this->controller_reference_ = controller_reference;
        /*
         *  Set the entries in the member controller reference
         *  struct to their respective controllers.
         */
        this->open_loop_controller_attitude_z_.setReference(controller_reference_.attitude_z);
        this->pid_controller_attitude_y_.setReference(controller_reference_.attitude_y);
        this->pid_controller_attitude_z_.setReference(controller_reference_.attitude_z);
        this->pid_controller_position_x_.setReference(controller_reference_.position_x);
    }

    void
        Controller::setPeriod(const double& period, const bool& fast_domain)
    {
        /*
         *  The reason behind the fast and slow time domain here is
         *  similar to the explanation in the sensor class.
         *  Controlling Y attitude (pitch) requires little to no
         *  delay and thus requires fast domain. Controlling X position
         *  (forward/backward) and Z attitude (yaw), however, the slow
         *  domain is more suitable as their sensor data is noisier.
         */
        if (fast_domain)
        {
            /*
             *  Set the Y attitude (pitch) PID controller period.
             */
            this->pid_controller_attitude_y_.setPeriod(period);
        }
        else
        {
            /*
             *  Set the X position (forward/backward) PID controller period.
             */
            this->pid_controller_position_x_.setPeriod(period);

            /*
             *  Set the Z attitude (yaw) PID controller period.
             */
            this->pid_controller_attitude_z_.setPeriod(period);
        }
    }

    void
        Controller::enableOpenLoopAttitudeZControl(const bool& enable)
    {
        /*
         *  Reset the Z attitude (yaw) PID controller integral error and
         *  set the Z attitude open loop control enable flag.
         */
        pid_controller_attitude_z_.resetErrorIntegral();
        open_loop_attitude_z_control_enabled_ = enable;
    }

    void
        Controller::control(const bool& fast_domain)
    {
        /*
         *  Get the encoder data, BMX160 IMU data, and MPU6050 IMU data
         *  structs from the sensor object.
         */
        IMUData mpu_data = sensor_->getIMUDataMPU6050();
        IMUData bmx_data = sensor_->getIMUDataBMX160();
        EncoderData encoder_data = sensor_->getEncoderData();

        /*
         *  Update the controller active status using the MPU6050 IMU
         *  data struct.
         */
        updateActiveStatus(mpu_data);

        /*
         *  The reason behind the fast and slow time domain here is
         *  similar to the explanation in the sensor class.
         *  Controlling Y attitude (pitch) requires little to no
         *  delay and thus requires fast domain. Controlling X position
         *  (forward/backward) and Z attitude (yaw), however, the slow
         *  domain is more suitable as their sensor data is noisier.
         */
        if (fast_domain)
        {
            /*
             *  Set the plant state input (Y) of the Y attitude (pitch)
             *  PID controller to be the Y attitude (pitch) in the MPU6050
             *  IMU data struct.
             */
            pid_controller_attitude_y_.setState(mpu_data.attitude_y);

            /*
             *  Set the error derivative input (delta e) of the Y attitude
             *  (pitch) PID controller to be the Y angular velocity in the
             *  MPU6050 IMU data struct.
             */
            pid_controller_attitude_y_.setErrorDifferential(mpu_data.angular_velocity_y);

            /*
             *  Execute the Y attitude (pitch) PID controller and store
             *  the output into the member Y attitude (pitch)
             *  controller output variable.
             */
            output_attitude_y_ = pid_controller_attitude_y_.control();
        }
        else
        {
            /*
             *  Set the plant state input (Y) of the X position
             *  (forward/backward) PID controller to be the X
             *  position in the encoder data struct.
             */
            pid_controller_position_x_.setState(encoder_data.position_x);

            /*
             *  Set the error derivative input (delta e) of the X position
             *  (forward/backward) PID controller to be the X linear velocity
             *  in the encoder data struct.
             */
            pid_controller_position_x_.setErrorDifferential(encoder_data.velocity_x);

            /*
             *  Set the plant state input (Y) of the Z attitude
             *  (yaw) PID controller to be the Z attitude in the
             *  BMX160 IMU data struct.
             */
            pid_controller_attitude_z_.setState(bmx_data.attitude_z);

            /*
             *  Set the error derivative input (delta e) of the Z attitude (yaw)
             *  PID controller to be the Z angular velocity in the MPU6050 IMU
             *  data struct.
             */
            pid_controller_attitude_z_.setErrorDifferential(mpu_data.angular_velocity_z);

            /*
             *  Execute the X position (forward/backward) PID controller
             *  and store the output into the member X position
             *  (forward/backward) controller output variable.
             */
            output_position_x_ = pid_controller_position_x_.control();

            /*
             *  if the Z attitude open loop control is enabled, execute
             *  the Z attitude (yaw) open loop controller; otherwise, execute
             *  the Z attitude (yaw) PID controller. Store the output
             *  into the member Z attitude (yaw) controller output variable.
             */
            if (open_loop_attitude_z_control_enabled_) {
                output_attitude_z_ = open_loop_controller_attitude_z_.control();
            }
            else {
                output_attitude_z_ = pid_controller_attitude_z_.control();
            }
        }

        /*
         *  Produce the left motor output by adding the
         *  X position controller output with the
         *  Y attitude controller output and then subtract
         *  the Z attitude controller output.
         */
        double left_motor_output = output_position_x_ + output_attitude_y_ - output_attitude_z_;
        /*
         *  Produce the right motor output by adding all three
         *  controller outputs.
         */
        double right_motor_output = output_position_x_ + output_attitude_y_ + output_attitude_z_;

        /*
         *  If the controller is inactive, stop the motors
         *  by setting both the motor outputs to 0.
         */
        if (!active_) {
            left_motor_output = 0;
            right_motor_output = 0;
        }

        /*
         *  Set the motor enable in the member actuation
         *  command struct to true, enabling the motors.
         */
        actuation_command_.motor_enable = true;

        /*
         *  Set the motor directions in the member actuation
         *  command struct to be the sign of the motor output
         *  values. For example, the left motor forward flag
         *  should be true if the left motor output value is
         *  greater than or equal to the minimum PWM value in
         *  the motor parameter name space.
         */
        if ((left_motor_output >= MotorParameter::pwm_min) && (left_motor_output >= 0)) {
            actuation_command_.motor_left_forward = true;
        }

        if ((left_motor_output <= ((-1) * MotorParameter::pwm_min)) && (left_motor_output < 0)) {
            actuation_command_.motor_left_forward = false;
        }


        if ((right_motor_output >= MotorParameter::pwm_min) && (right_motor_output >= 0)) {
            actuation_command_.motor_right_forward = true;
        }

        if ((right_motor_output <= ((-1) * MotorParameter::pwm_min)) && (right_motor_output < 0)) {
            actuation_command_.motor_right_forward = false;
        }

        /*
         *  Clamp the magnitude of the motor output values to be
         *  within the minimum and the maximum motor PWM values using
         *  the clamp function in the math header. See the motor
         *  parameter name space. Remember to static_cast the motor
         *  parameter entries to double before passing them to the
         *  clamp function.
         */
        if (actuation_command_.motor_left_forward) {
            actuation_command_.motor_left_pwm = clamp(left_motor_output, static_cast<double>(MotorParameter::pwm_min), static_cast<double>(MotorParameter::pwm_max));
        }
        else {
            actuation_command_.motor_left_pwm = clamp((-1) * left_motor_output, static_cast<double>(MotorParameter::pwm_min), static_cast<double>(MotorParameter::pwm_max));
        }

        if (actuation_command_.motor_right_forward) {
            actuation_command_.motor_right_pwm = clamp(right_motor_output, static_cast<double>(MotorParameter::pwm_min), static_cast<double>(MotorParameter::pwm_max));
        }
        else {
            actuation_command_.motor_right_pwm = clamp((-1) * right_motor_output, static_cast<double>(MotorParameter::pwm_min), static_cast<double>(MotorParameter::pwm_max));
        }

    }

    void
        Controller::updateActiveStatus(const IMUData& imu_data)
    {
        /*
         *  Check Y attitude (pitch)
         */
        if (fabs(radiansToDegrees(imu_data.attitude_y)) > ControllerParameter::attitude_y_active)
        {
            /*
             *  The controller becomes inactive if the magnitude of the
             *  Y attitude (pitch) is greater than the Y attitude threshold
             *  in the controller parameter name space.
             */
            active_ = false;

            /*
             *  Set NeoPixel frame to be the frame for controller
             *  inactive status.
             */
            if (neopixel_)
            {
                neopixel_->setFrame(neopixel_frame_inactive_);
            }
        }
        else
        {
            /*
             *  If the controller was inactive and is now becoming active,
             *  reset all integrated errors.
             */
            if (!active_)
            {
                pid_controller_position_x_.resetErrorIntegral();
                pid_controller_attitude_y_.resetErrorIntegral();
                pid_controller_attitude_z_.resetErrorIntegral();
            }

            /*
             *  The controller is active if the magnitude of the
             *  Y attitude (pitch) is within the threshold.
             */
            active_ = true;

            /*
             *  Set NeoPixel frame to be the frame for controller
             *  active status.
             */
            if (neopixel_)
            {
                neopixel_->setFrame(neopixel_frame_active_);
            }
        }
    }
}   // namespace biped
