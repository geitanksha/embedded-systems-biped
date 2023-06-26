/**
 *  @file   controller.h
 *  @author Simon Yu
 *  @date   01/13/2022
 *  @brief  Controller class header.
 *
 *  This file defines the controller class.
 */

/*
 *  Include guard.
 */
#ifndef CONTROLLER_CONTROLLER_H_
#define CONTROLLER_CONTROLLER_H_

/*
 *  Project headers.
 */
#include "platform/neopixel.h"
#include "controller/open_loop_controller.h"
#include "controller/pid_controller.h"
#include "common/type.h"

/*
 *  Biped namespace.
 */
namespace biped
{
/**
 *  @brief  Controller class.
 *
 *  This class provides functions for creating a controller,
 *  which computes actuation command from a series of sensor data.
 */
class Controller
{
public:

    /**
     *  @brief  Controller class constructor.
     *
     *  This constructor initializes all class member variables.
     *  Additionally, the constructor initializes and configures
     *  all controllers.
     */
    Controller();

    /**
     *  @return Actuation command struct.
     *  @brief  Get the class member actuation command struct.
     *
     *  This function returns the class member actuation command struct.
     */
    ActuationCommand
    getActuationCommand() const;

    /**
     *  @return Controller active flag.
     *  @brief  Get the controller active status.
     *
     *  This function returns the controller active status. If inactive,
     *  the controller generates an actuation command that stops the motors.
     */
    bool
    getActiveStatus() const;

    /**
     *  @return Controller reference struct.
     *  @brief  Get the class member controller reference struct.
     *
     *  This function returns the class member controller reference struct.
     */
    ControllerReference
    getControllerReference() const;

    /**
     *  @param  controller_reference Controller reference struct.
     *  @brief  Set the controller reference.
     *
     *  This function sets the controller reference.
     */
    void
    setControllerReference(const ControllerReference& controller_reference);

    /**
     *  @param  period Controller period, in seconds.
     *  @param  fast_domain Whether the given controller period is for fast domain.
     *  @brief  Set the controller period.
     *
     *  This function sets the controller period, for fast or slow
     *  domain, for the controller.
     */
    void
    setPeriod(const double& period, const bool& fast_domain);

    /**
     *  @param  enable Whether to enable open-loop control for Z attitude.
     *  @brief  Enable or disable open-loop control for Z attitude.
     *
     *  This function enables or disables open-loop control for Z attitude
     *  depending on the parameter.
     */
    void
    enableOpenLoopAttitudeZControl(const bool& enable);

    /**
     *  @param  fast_domain Whether to perform fast domain control.
     *  @brief  Execute the controller.
     *
     *  This function executes the controller, for fast or slow
     *  domain, and populates the member actuation command struct.
     *  This function is expected to be called periodically.
     */
    void
    control(const bool& fast_domain);

private:

    /**
     *  @param  imu_data IMU data.
     *  @brief  Update the controller active status.
     *
     *  This function updates the controller active status. If inactive,
     *  the controller generates an actuation command that stops the motors.
     */
    void
    updateActiveStatus(const IMUData& imu_data);

    bool active_;   //!< Controller active flag.
    bool open_loop_attitude_z_control_enabled_; //!< Z attitude open loop control enable flag.
    ActuationCommand actuation_command_;    //!< Actuation command struct.
    ControllerReference controller_reference_;    //!< Controller reference struct.
    std::shared_ptr<NeoPixel::Frame> neopixel_frame_active_; //!< NeoPixel frame for controller active status.
    std::shared_ptr<NeoPixel::Frame> neopixel_frame_inactive_; //!< NeoPixel frame for controller inactive status.
    double output_position_x_;   //!< X position (forward/backward) controller output.
    double output_attitude_y_;   //!< Y attitude (pitch) controller output.
    double output_attitude_z_;   //!< Z attitude (yaw) controller output.
    OpenLoopController open_loop_controller_attitude_z_; //!< Z attitude (yaw) open loop controller object.
    PIDController pid_controller_attitude_y_;   //!< Y attitude (pitch) PID controller object.
    PIDController pid_controller_attitude_z_;   //!< Z attitude (yaw) PID controller object.
    PIDController pid_controller_position_x_; //!< X position (forward/backward) PID controller object.
    double open_loop_controller_gain_attitude_z_;   //!< Z attitude (yaw) open loop controller gain.
    PIDController::Gain pid_controller_gain_attitude_y_; //!< Y attitude (pitch) PID controller gain struct.
    PIDController::Gain pid_controller_gain_attitude_z_; //!< Z attitude (yaw) PID controller gain struct.
    PIDController::Gain pid_controller_gain_position_x_; //!< X position (forward/backward) PID controller gain struct.
    ControllerSaturation open_loop_controller_saturation_attitude_z_; //!< Z attitude (yaw) open loop controller saturation struct.
    ControllerSaturation pid_controller_saturation_attitude_y_; //!< Y attitude (pitch) PID controller saturation struct.
    ControllerSaturation pid_controller_saturation_attitude_z_; //!< Z attitude (yaw) PID controller saturation struct.
    ControllerSaturation pid_controller_saturation_position_x_; //!< X position (forward/backward) PID controller saturation struct.
};
}   // namespace biped

#endif  // CONTROLLER_CONTROLLER_H_
