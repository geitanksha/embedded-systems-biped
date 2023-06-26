/**
 *  @file   pid_controller.cpp
 *  @author Simon Yu
 *  @date   01/13/2022
 *  @brief  PID controller class source.
 *
 *  This file implements the PID controller class.
 */

 /*
  *  Project headers.
  */
#include "utility/math.h"
#include "common/parameter.h"
#include "controller/pid_controller.h"
#include "platform/serial.h"

  /*
   *  Biped namespace.
   */
namespace biped
{
    PIDController::PIDController() : state_(0), reference_(0), period_(0), error_differential_(0),
        error_integral_(0)
    {
    }

    double
        PIDController::getReference() const
    {
        /*
         *  Return the PID controller reference (R).
         */

        return this->reference_;
    }

    void
        PIDController::setGain(const Gain& gain)
    {
        /*
         *  Set the PID controller gain.
         */

        this->gain_ = gain;

        /*
         *  The integrated error would become meaningless
         *  with new gains. Reset the integrated error
         *  (integral of e).
         */

         // this->error_integral = resetErrorIntegral(); //unsure
        resetErrorIntegral(); // unsure

    }

    void
        PIDController::setSaturation(const ControllerSaturation& saturation)
    {
        /*
         *  Set the PID controller saturation.
         */

        this->saturation_ = saturation;
    }

    void
        PIDController::setState(const double& state)
    {
        /*
         *  Set the plant state input (Y).
         */
        this->state_ = state;
    }

    void
        PIDController::setReference(const double& reference)
    {
        /*
         *  Set the PID controller reference (R).
         */
        this->reference_ = reference;

        /*
         *  The integrated error would become meaningless
         *  with a new reference. Reset the integrated error
         *  (integral of e).
         */
         // this->error_integral = resetErrorIntegral(); //unsure
        resetErrorIntegral(); // unsure
    }

    void
        PIDController::setPeriod(const double& period)
    {
        /*
         *  Set the PID controller period.
         */
        this->period_ = period;
        /*
         *  The integrated error would become meaningless
         *  with a new control period. Reset the integrated
         *  error (integral of e).
         */
         // this->error_integral = resetErrorIntegral(); //unsure
        resetErrorIntegral(); // unsure

    }

    void
        PIDController::setErrorDifferential(const double& error_differential)
    {
        /*
         *  Set the error derivative input (delta e).
         */
        this->error_differential_ = error_differential;

    }

    void
        PIDController::resetErrorIntegral()
    {
        /*
         *  Reset the integrated error (integral of e) to 0.
         */
        this->error_integral_ = 0;
    }

    double
        PIDController::control()
    {
        /*
         *  Validate PID controller period.
         */
        if (period_ <= 0)
        {
            Serial(LogLevel::error) << "Invalid period.";
            return 0;
        }

        /*
         *  Calculate the error (e) between the plant input state (Y)
         *  and PID controller reference (R), and clamp the
         *  calculated error between the input saturation upper
         *  and lower bounds.
         *
         *  For the "correct" signedness, calculate the error (e) by
         *  subtracting state with reference (Y - R). Otherwise, the signs
         *  of your PID controller gains may be flipped.
         *
         *  See Lecture 13 for the definition of
         *  the PID controller.
         */

        double cur_error = clamp((this->state_ - this->reference_), this->saturation_.input_lower, this->saturation_.input_upper); // do we set the error for this current instance? struct elements suggest no



        /*
         *  Calculate the new discrete integral of error (integral of e),
         *  and clamp the calculated integral of error (integral of e)
         *  between the negative and positive maximum integrated error
         *  from the PID controller gain struct.
         *
         *  The new discrete integrated error is the current integrated
         *  error plus the product between the PID controller period
         *  and the current error (e).
         *
         *  The above is essentially the Riemann sum:
         *  https://en.wikipedia.org/wiki/Riemann_sum
         *
         *  See Lecture 13 for the definition of
         *  the PID controller.
         */

        this->error_integral_ = clamp((this->error_integral_ + (this->period_ * cur_error)), (-1 * (this->gain_.integral_max)), (this->gain_.integral_max)); // do we set the error for this current instance? struct elements suggest no

        /*
         *  Calculate the proportional output.
         *
         *  See Lecture 13 for the definition of
         *  the PID controller.
         */

         // F_p = -Kp*x
        double p_output = (this->gain_.proportional) * (cur_error);


        /*
         *  Calculate the integral output.
         *
         *  See Lecture 13 for the definition of
         *  the PID controller.
         */
         // F_i = -Ki*integrate(x)
        double i_output = (this->gain_.integral) * (this->error_integral_);


        /*
         *  Calculate the differential output.
         *
         *  In theory, the differential output is proportional
         *  to the time derivative of the error (e). Instead of
         *  calculating the derivative of the error (e), we
         *  measure the derivative for better performance. For
         *  example, for position control, the differential
         *  output would be proportional to the derivative of
         *  the error (e), which, in this example, is simply
         *  velocity (assuming the target is unchanged over
         *  time.) Instead of calculating the velocity, we
         *  measure it using the sensors (encoders). Here,
         *  the measured error derivative is stored in the
         *  error differential member variable.
         *
         *  See Lecture 13 for the definition of
         *  the PID controller.
         */

         // F_i = -Kd*differentiate(x)
         // F_i = -Kd*encodervelocity

        double d_output = (this->gain_.differential) * (this->error_differential_);



        /*
         *  Sum up all the above outputs.
         */

        double sum_ouput = p_output + i_output + d_output;

        /*
         *  Return the sum of the outputs clamped between the output
         *  saturation upper and lower bounds as the final output
         *  of the PID controller.
         */
        return clamp(sum_ouput, this->saturation_.output_lower, this->saturation_.output_upper);
    }
}   // namespace biped
