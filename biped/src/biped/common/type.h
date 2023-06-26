/**
 *  @file   type.h
 *  @author Simon Yu
 *  @date   01/05/2023
 *  @brief  Type class header.
 *
 *  This file defines the type classes.
 */

/*
 *  Include guard.
 */
#ifndef COMMON_TYPE_H_
#define COMMON_TYPE_H_

/*
 *  External headers.
 */
#include <limits>
#include <memory>

/*
 *  Project headers.
 */
#include "utility/math.h"

/*
 *  Biped namespace.
 */
namespace biped
{
/**
 *  @brief  Actuation command struct.
 *
 *  This struct contains actuation command entries,
 *  such as the motor enable, left and right motor directions
 *  and pulse width modulation (PWM) values.
 */
struct ActuationCommand
{
    bool motor_enable;  //!< Motor enable.
    bool motor_left_forward;    //!< Left motor direction.
    bool motor_right_forward;   //!< Right motor direction.
    double motor_left_pwm;   //!< Left motor PWM value.
    double motor_right_pwm;  //!< Right motor PWM value.

    /**
     *  @brief  Actuation command struct constructor.
     *
     *  This constructor initializes all actuation command struct entries.
     */
    ActuationCommand() : motor_enable(false), motor_left_forward(true), motor_right_forward(false),
            motor_left_pwm(0), motor_right_pwm(0)
    {
    }
};

/**
 *  @brief  Controller reference struct.
 *
 *  This struct contains controller reference entries,
 *  such as X position (forward/backward), Y attitude (pitch), and
 *  Z attitude (yaw) references.
 */
struct ControllerReference
{
    double attitude_y;   //!< Y attitude (pitch) controller reference, in radians.
    double attitude_z;   //!< Z attitude (yaw) controller reference, in radians.
    double position_x;   //!< X position controller reference, in meters.

    /**
     *  @brief  Controller reference struct constructor.
     *
     *  This constructor initializes all controller reference struct entries to 0.
     */
    ControllerReference() : attitude_y(degreesToRadians(0)), attitude_z(degreesToRadians(0)),
            position_x(0)
    {
    }
};

/**
 *  @brief  Controller saturation struct.
 *
 *  This struct contains controller saturation entries,
 *  such as the input and output saturations.
 */
struct ControllerSaturation
{
    double input_upper;   //!< Input saturation upper bound.
    double input_lower;   //!< Input saturation lower bound.
    double output_upper;  //!< Output saturation upper bound.
    double output_lower;  //!< Output saturation lower bound.

    /**
     *  @brief  Controller saturation struct constructor.
     *
     *  This constructor initializes all controller saturation
     *  entries to their respective extremes.
     */
    ControllerSaturation() : input_upper(std::numeric_limits<double>::max()),
            input_lower(std::numeric_limits<double>::lowest()),
            output_upper(std::numeric_limits<double>::max()),
            output_lower(std::numeric_limits<double>::lowest())
    {
    }
};

/**
 *  @brief  Encoder data struct.
 *
 *  This struct contains encoder data entries.
 *
 *  All spatial data is in the standard body reference frame.
 *
 *  Standard body reference frame:
 *  https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-refframes
 */
struct EncoderData
{
    double position_x;   //!< X position, in meters.
    double velocity_x;   //!< X velocity, in meters per second.
    double steps;   //!< Encoder steps.

    /**
     *  @brief  Encoder data struct constructor.
     *
     *  This constructor initializes all encoder data struct entries to 0.
     */
    EncoderData() : position_x(0), velocity_x(0), steps(0)
    {
    }
};

/**
 *  @brief  IMU data struct.
 *
 *  This struct contains inertial measurement unit (IMU) data entries,
 *  such as attitude, linear velocity, angular velocity,
 *  linear acceleration, and etc.
 *
 *  All spatial data is in the standard body reference frame.
 *
 *  Standard body reference frame:
 *  https://www.vectornav.com/resources/inertial-navigation-primer/math-fundamentals/math-refframes
 *
 *  Rotational right-hand rule:
 *  https://en.wikipedia.org/wiki/Right-hand_rule#Rotations
 */
struct IMUData
{
    double acceleration_x;   //!< X acceleration, in meters per second squared.
    double acceleration_y;   //!< Y acceleration, in meters per second squared.
    double acceleration_z;   //!< Z acceleration, in meters per second squared.
    double attitude_x;   //!< X attitude, or roll angle, in radians.
    double attitude_y;   //!< Y attitude, or pitch angle, in radians.
    double attitude_z;   //!< Z attitude, or yaw angle, in radians.
    double angular_velocity_x;   //!< X angular velocity, or roll rate, in radians per second.
    double angular_velocity_y;   //!< Y angular velocity, or pitch rate, in radians per second.
    double angular_velocity_z;   //!< Z angular velocity, or yaw rate, in radians per second.
    double compass_x;   //!< X compass field strength, in micro Tesla.
    double compass_y;   ///!< Y compass field strength, In micro Tesla.
    double compass_z;   //!< Z compass field strength, In micro Tesla.
    double temperature;  //!< Temperature, in Celsius.

    /**
     *  @brief  IMU data struct constructor.
     *
     *  This constructor initializes all IMU data struct entries to 0.
     */
    IMUData() : acceleration_x(0), acceleration_y(0), acceleration_z(0), attitude_x(0),
            attitude_y(0), attitude_z(0), angular_velocity_x(0), angular_velocity_y(0),
            angular_velocity_z(0), compass_x(0), compass_y(0), compass_z(0), temperature(0)
    {
    }
};

/**
 *  @brief  Log level enum class.
 *
 *  This enum class defines log levels.
 */
enum class LogLevel
{
    fatal = 0,  //!< Fatal log level.
    error,  //!< Error log level.
    warn,   //!< Warning log level.
    info,   //!< Information log level.
    debug,  //!< Debugging log level.
    trace   //!< Tracing log level.
};

/**
 *  @brief  Planner maneuver struct
 *
 *  This struct contains planner maneuver entries,
 *  such as maneuver transition type, maneuver transition value,
 *  maneuver type, and the shared pointer to the next maneuver.
 *  The maneuvers can be chained up in a linked list fashion.
 *  The struct also defines the planner maneuver type, and
 *  maneuver transition type enum classes.
 */
struct Maneuver
{
    /**
     *  @brief  Planner maneuver type enum class
     *
     *  This enum class defines maneuver types.
     */
    enum class Type
    {
        drive = 0,  //!< Drive forward until the end of this maneuver.
        drive_left, //!< Drive to the left until the end of this maneuver.
        drive_right,    //!< Drive to the right until the end of this maneuver.
        park,   //!< Park or stop until the end of this maneuver.
        reverse,    //!< Reverse until the end of this maneuver.
        reverse_left,   //!< Reverse to the left until the end of this maneuver.
        reverse_right,  //!< Reverse to the right until the end of this maneuver.
    };

    /**
     *  @brief  Planner maneuver transition type enum class
     *
     *  This enum class defines maneuver transition types.
     */
    enum class TransitionType
    {
        attitude_z_above = 0, //!< Transition if the Z attitude is above a certain value, in radians.
        attitude_z_below,   //!< Transition if the Z attitude is below a certain value, in radians.
        duration, //!< Transition after a certain time duration, in seconds.
        position_x_above, //!< Transition if the X position is above a certain value, in meters.
        position_x_below, //!< Transition if the X position is below a certain value, in meters.
        range_left_above, //!< Transition if the left time-of-flight range is above a certain value, in meters.
        range_left_below, //!< Transition if the left time-of-flight range is below a certain value, in meters.
        range_middle_above, //!< Transition if the middle time-of-flight range is above a certain value, in meters.
        range_middle_below, //!< Transition if the middle time-of-flight range is below a certain value, in meters.
        range_right_above, //!< Transition if the right time-of-flight range is above a certain value, in meters.
        range_right_below //!< Transition if the right time-of-flight range is below a certain value, in meters.
    };

    TransitionType transition_type; //!< Maneuver transition type.
    double transition_value; //!< Maneuver transition value, the meaning of which depends on the maneuver transition type.
    Type type;  //!< Maneuver type.
    std::shared_ptr<Maneuver> next; //!< Pointer to the next maneuver.

    /**
     *  @brief  Planner maneuver struct constructor
     *
     *  This constructor initializes all planner maneuver struct entries.
     */
    Maneuver() : transition_type(TransitionType::duration), transition_value(0), type(Type::park),
            next(nullptr)
    {
    }
};

/**
 *  @brief  Stream manipulator enum class.
 *
 *  This enum class defines stream manipulators.
 */
enum class StreamManipulator
{
    carriage_return,    //!< Carriage return stream manipulator.
    clear_line, //!< Clear-line stream manipulator.
    endl    //!< New-line stream manipulator.
};

/**
 *  @brief  Time-of-flight data struct.
 *
 *  This struct contains time-of-flight data entries.
 */
struct TimeOfFlightData
{
    double range_left;   //!< Left time-of-flight range, in meters.
    double range_middle; //!< Middle time-of-flight range, in meters.
    double range_right;   //!< Right time-of-flight range, in meters.
};

/**
 *  @brief  Planner waypoint struct
 *
 *  This struct contains planner waypoint entries,
 *  such as controller reference, waypoint duration,
 *  and the shared pointer to the next waypoint. The
 *  waypoints can be chained up in a linked list fashion.
 */
struct Waypoint
{
    ControllerReference controller_reference; //!< Controller reference.
    double duration; //!< Transition to the next waypoint after this amount of time duration, in seconds.
    std::shared_ptr<Waypoint> next; //!< Pointer to the next waypoint.

    /**
     *  @brief  Planner waypoint struct constructor
     *
     *  This constructor initializes all planner waypoint struct entries.
     */
    Waypoint() : controller_reference(), duration(0), next(nullptr)
    {
    }
};
}   // namespace biped

#endif  // COMMON_TYPE_H_
