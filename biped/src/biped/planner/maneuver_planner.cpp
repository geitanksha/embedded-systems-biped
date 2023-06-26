/**
 *  @file   maneuver_planner.cpp
 *  @author Simon Yu
 *  @date   04/01/2022
 *  @brief  Maneuver planner class source.
 *
 *  This file implements the maneuver planner class.
 */

 /*
  *  Project headers.
  */
#include "controller/controller.h"
#include "common/global.h"
#include "planner/maneuver_planner.h"
#include "sensor/sensor.h"
#include "platform/serial.h"
#include "common/type.h"

  /*
   *  Biped namespace.
   */
namespace biped
{
    ManeuverPlanner::ManeuverPlanner() : maneuver_counter_(1), maneuver_timer_(0), plan_started_(false),
        maneuver_started_(false), plan_completed_(true)
    {
        /*
         *  Create a set of maneuvers for the example plan.
         *  During their configurations, the maneuvers should
         *  be chained up in a linked list fashion.
         */
         // std::shared_ptr<Maneuver> maneuver_1 = std::make_shared<Maneuver>();
         // std::shared_ptr<Maneuver> maneuver_2 = std::make_shared<Maneuver>();
         // std::shared_ptr<Maneuver> maneuver_3 = std::make_shared<Maneuver>();

         /*
          *  Set the start and current maneuvers.
          */
          // maneuver_start_ = maneuver_1;
          // maneuver_ = maneuver_start_;

          /*
           *  Example plan maneuver 1 configuration:
           *      - Park for 2 seconds.
           *      - Then, start maneuver 2.
           */
           // maneuver_1->transition_type = Maneuver::TransitionType::duration;
           // maneuver_1->transition_value = 2;
           // maneuver_1->type = Maneuver::Type::park;
           // maneuver_1->next = maneuver_2;

           /*
            *  Example plan maneuver 2 configuration:
            *      - Drive forward until the X position goes above 1 meter.
            *      - Then, start maneuver 3.
            */
            // maneuver_2->transition_type = Maneuver::TransitionType::position_x_above;
            // maneuver_2->transition_value = 1;
            // maneuver_2->type = Maneuver::Type::drive;
            // maneuver_2->next = maneuver_3;

            /*
             *  Example plan maneuver 3 configuration:
             *      - Park for 2 seconds.
             *      - Then, plan completed.
             */
             // maneuver_3->transition_type = Maneuver::TransitionType::duration;
             // maneuver_3->transition_value = 2;
             // maneuver_3->type = Maneuver::Type::park;
             // maneuver_3->next = nullptr;

             /*
              *  Using the example plan above, create your own maneuver-based plan.
              *  Feel free to add or remove maneuvers. Also feel free to remove or
              *  comment out the example plan. Just remember to set the current
              *  maneuver to the first maneuver in your own maneuver-based plan.
              *  See the maneuver type and the maneuver transition type enum classes
              *  for all the available maneuver types and maneuver transition types.
              *
              *  Be aware of the singularity in the Z attitude data, i.e., the Z
              *  attitude data goes from 90 degrees to -270 degrees due to the atan2
              *  function. Going across the singularity might cause the Kalman filters
              *  and the controllers to become unstable. Therefore, it is recommended to
              *  try and avoid the Z attitude singularity in your maneuvers.
              */
        std::shared_ptr<Maneuver> maneuver_1 = std::make_shared<Maneuver>();
        std::shared_ptr<Maneuver> maneuver_2 = std::make_shared<Maneuver>();
        // std::shared_ptr<Maneuver> maneuver_3 = std::make_shared<Maneuver>();

        maneuver_start_ = maneuver_1;
        maneuver_ = maneuver_start_;

        maneuver_1->transition_type = Maneuver::TransitionType::range_middle_below;
        maneuver_1->transition_value = 0.1;
        maneuver_1->type = Maneuver::Type::drive;
        maneuver_1->next = maneuver_2;

        maneuver_2->transition_type = Maneuver::TransitionType::range_middle_above;
        maneuver_2->transition_value = 1.5;
        maneuver_2->type = Maneuver::Type::reverse;
        maneuver_2->next = maneuver_1;



    }

    void IRAM_ATTR
        ManeuverPlanner::start()
    {
        /*
         *  If the plan is completed, reset the maneuver pointer to the
         *  start maneuver pointer, reset the maneuver counter to 1,
         *  mark maneuver as not started, mark the plan as not started
         *  and not completed.
         */

        if (plan_completed_)
        {
            maneuver_ = maneuver_start_;
            maneuver_counter_ = 1;
            maneuver_started_ = false;
            plan_started_ = false;
            plan_completed_ = false;
        }

    }

    int
        ManeuverPlanner::plan()
    {
        /*
         *  Validate sensor object pointer.
         */
        if (!sensor_)
        {
            Serial(LogLevel::error) << "Sensor missing.";
            return -1;
        }

        /*
         *  Validate controller object pointer.
         */
        if (!controller_)
        {
            Serial(LogLevel::error) << "Controller missing.";
            return -1;
        }

        /*
         *  Return -1 if the plan is completed, or if controller
         *  is not active (pause the plan during safety disengage.)
         */

        if (plan_completed_ || (controller_->getActiveStatus() == false))
        {
            return -1;
        }

        /*
         *  Mark the plan as not started and completed, and return
         *  -1 if the plan has started, has not completed, but the
         *  current maneuver is null.
         */

        if (!maneuver_ && plan_started_ && !plan_completed_)
        {
            plan_started_ = false;
            plan_completed_ = true;
            return -1;
        }

        /*
         *  If the plan has not started.
         */
        if (!plan_started_)
        {
            /*
             *  The plan is empty if the plan has not started
             *  but the current maneuver is already null.
             */
            if (!maneuver_)
            {
                Serial(LogLevel::error) << "Empty maneuver-based plan.";
                return -1;
            }

            /*
             *  Mark the plan as started if it has not started.
             */
            Serial(LogLevel::info) << "Started maneuver-based plan.";
            plan_started_ = true;
        }

        if (!maneuver_started_)
        {
            /*
             *  Execute the current maneuver if it has not started.
             *  Generate controller reference from the current maneuver,
             *  Set the generated controller reference to the controller,
             *  update the maneuver timer to the current time, and mark
             *  the current maneuver as started.
             */

             //plan(); // to check!
            controller_->setControllerReference(generateControllerReference());
            maneuver_timer_ = millis(); // to check
            maneuver_started_ = true;

        }
        else
        {
            /*
             *  Transition to the next maneuver based on the
             *  current maneuver transition type and value.
             */
            switch (maneuver_->transition_type)
            {
            case Maneuver::TransitionType::attitude_z_above:
            {
                /*
                 *  If the current Z attitude goes above the transition value from
                 *  the current maneuver, transition to the next maneuver, increment
                 *  the maneuver counter, and mark the current maneuver as not started.
                 */

                IMUData bmx_data = sensor_->getIMUDataBMX160();
                if (bmx_data.attitude_z > maneuver_->transition_value)
                {
                    maneuver_ = maneuver_->next;
                    maneuver_counter_++;
                    maneuver_started_ = false;
                }

                break;
            }
            case Maneuver::TransitionType::attitude_z_below:
            {
                /*
                 *  If the current Z attitude goes below the transition value from
                 *  the current maneuver, transition to the next maneuver, increment
                 *  the maneuver counter, and mark the current maneuver as not started.
                 */

                IMUData bmx_data = sensor_->getIMUDataBMX160();
                if (bmx_data.attitude_z < maneuver_->transition_value)
                {
                    maneuver_ = maneuver_->next;
                    maneuver_counter_++;
                    maneuver_started_ = false;
                }

                break;
            }
            case Maneuver::TransitionType::duration:
            {
                /*
                 *  If the elapsed duration goes above the transition value from
                 *  the current maneuver, transition to the next maneuver, increment
                 *  the maneuver counter, and mark the current maneuver as not started.
                 */

                if ((millis() - maneuver_timer_) > (maneuver_->transition_value * 1000))
                {
                    maneuver_ = maneuver_->next;
                    maneuver_counter_++;
                    maneuver_started_ = false;
                }

                break;
            }
            case Maneuver::TransitionType::position_x_above:
            {
                /*
                 *  If the current X position goes above the transition value from
                 *  the current maneuver, transition to the next maneuver, increment
                 *  the maneuver counter, and mark the current maneuver as not started.
                 */
                 // to check - use encoder 
                EncoderData enc_data = sensor_->getEncoderData();

                if (enc_data.position_x > maneuver_->transition_value)
                {
                    maneuver_ = maneuver_->next;
                    maneuver_counter_++;
                    maneuver_started_ = false;
                }

                break;
            }
            case Maneuver::TransitionType::position_x_below:
            {
                /*
                 *  If the current X position goes below the transition value from
                 *  the current maneuver, transition to the next maneuver, increment
                 *  the maneuver counter, and mark the current maneuver as not started.
                 */

                EncoderData enc_data = sensor_->getEncoderData();

                if (enc_data.position_x < maneuver_->transition_value)
                {
                    maneuver_ = maneuver_->next;
                    maneuver_counter_++;
                    maneuver_started_ = false;
                }

                break;
            }
            case Maneuver::TransitionType::range_left_above:
            {
                /*
                 *  If the left time-of-flight range goes above the transition value from
                 *  the current maneuver, transition to the next maneuver, increment
                 *  the maneuver counter, and mark the current maneuver as not started.
                 */

                TimeOfFlightData tof_data = sensor_->getTimeOfFlightData();
                if (tof_data.range_left > maneuver_->transition_value)
                {
                    maneuver_ = maneuver_->next;
                    maneuver_counter_++;
                    maneuver_started_ = false;
                }


                break;
            }
            case Maneuver::TransitionType::range_left_below:
            {
                /*
                 *  If the left time-of-flight range goes below the transition value from
                 *  the current maneuver, transition to the next maneuver, increment
                 *  the maneuver counter, and mark the current maneuver as not started.
                 */
                TimeOfFlightData tof_data = sensor_->getTimeOfFlightData();
                if (tof_data.range_left < maneuver_->transition_value)
                {
                    maneuver_ = maneuver_->next;
                    maneuver_counter_++;
                    maneuver_started_ = false;
                }

                break;
            }
            case Maneuver::TransitionType::range_middle_above:
            {
                /*
                 *  If the middle time-of-flight range goes above the transition value from
                 *  the current maneuver, transition to the next maneuver, increment
                 *  the maneuver counter, and mark the current maneuver as not started.
                 */

                TimeOfFlightData tof_data = sensor_->getTimeOfFlightData();
                if (tof_data.range_middle > maneuver_->transition_value)
                {
                    maneuver_ = maneuver_->next;
                    maneuver_counter_++;
                    maneuver_started_ = false;
                }

                break;
            }
            case Maneuver::TransitionType::range_middle_below:
            {
                /*
                 *  If the middle time-of-flight range goes below the transition value from
                 *  the current maneuver, transition to the next maneuver, increment
                 *  the maneuver counter, and mark the current maneuver as not started.
                 */
                TimeOfFlightData tof_data = sensor_->getTimeOfFlightData();
                if (tof_data.range_middle < maneuver_->transition_value)
                {
                    maneuver_ = maneuver_->next;
                    maneuver_counter_++;
                    maneuver_started_ = false;
                }

                break;
            }
            case Maneuver::TransitionType::range_right_above:
            {
                /*
                 *  If the right time-of-flight range goes above the transition value from
                 *  the current maneuver, transition to the next maneuver, increment
                 *  the maneuver counter, and mark the current maneuver as not started.
                 */
                TimeOfFlightData tof_data = sensor_->getTimeOfFlightData();
                if (tof_data.range_right > maneuver_->transition_value)
                {
                    maneuver_ = maneuver_->next;
                    maneuver_counter_++;
                    maneuver_started_ = false;
                }

                break;
            }
            case Maneuver::TransitionType::range_right_below:
            {
                /*
                 *  If the right time-of-flight range goes below the transition value from
                 *  the current maneuver, transition to the next maneuver, increment
                 *  the maneuver counter, and mark the current maneuver as not started.
                 */
                TimeOfFlightData tof_data = sensor_->getTimeOfFlightData();
                if (tof_data.range_right < maneuver_->transition_value)
                {
                    maneuver_ = maneuver_->next;
                    maneuver_counter_++;
                    maneuver_started_ = false;
                }

                break;
            }
            default:
            {
                /*
                 *  Unknown maneuver transition type. Log warning message.
                 */
                Serial(LogLevel::warn) << "Unknown maneuver transition type.";
                break;
            }
            }
        }

        /*
         *  Return the maneuver counter.
         */
        return maneuver_counter_;
    }

    ControllerReference
        ManeuverPlanner::generateControllerReference() const
    {
        /*
         *  Create a default controller reference.
         */
        ControllerReference controller_reference;

        /*
         *  Validate current maneuver pointer.
         */
        if (!maneuver_)
        {
            Serial(LogLevel::warn) << "Invalid maneuver.";
            return controller_reference;
        }

        /*
         *  Generate the controller reference based on the
         *  current maneuver type.
         */
        switch (maneuver_->type)
        {
        case Maneuver::Type::park:
        {
            /*
             *  Set the X position and Z attitude controller references to be
             *  the current X position and Z attitude, i.e., stay at the
             *  current X position and Z attitude.
             */

             /* Create the object */
            controller_reference = controller_->getControllerReference();
            /* Get data from sensors*/
            IMUData bmx_data = sensor_->getIMUDataBMX160();
            EncoderData enc_data = sensor_->getEncoderData();
            /* Change current values depending on sensor values received */
            controller_reference.position_x = enc_data.position_x;
            controller_reference.attitude_z = bmx_data.attitude_z;


            break;
        }
        case Maneuver::Type::reverse:
        {
            /*
             *  Set the X position controller reference to be way below the
             *  current X position, i.e., reverse until the next maneuver, and
             *  the Z attitude controller reference to be the current Z attitude,
             *  i.e., stay at the current Z attitude.
             */

             /* Create the object */
            controller_reference = controller_->getControllerReference();
            /* Get data from sensors*/
            IMUData bmx_data = sensor_->getIMUDataBMX160();
            EncoderData enc_data = sensor_->getEncoderData();
            /* Change current values depending on sensor values received */
            controller_reference.position_x = enc_data.position_x - 1000000;
            controller_reference.attitude_z = bmx_data.attitude_z;

            break;
        }
        case Maneuver::Type::reverse_left:
        {
            /*
             *  Set the X position controller reference to be way below the
             *  current X position, i.e., reverse until the next maneuver, and
             *  the Z attitude controller reference to be 90 degrees above the
             *  current Z attitude, i.e., reverse left 90 degrees.
             */

             /* Create the object */
            controller_reference = controller_->getControllerReference();
            /* Get data from sensors*/
            IMUData bmx_data = sensor_->getIMUDataBMX160();
            EncoderData enc_data = sensor_->getEncoderData();
            /* Change current values depending on sensor values received */
            controller_reference.position_x = enc_data.position_x - 1000000;
            controller_reference.attitude_z = bmx_data.attitude_z + degreesToRadians(90);



            // controller_reference = controller_->getControllerReference();
            // controller_reference.position_x -= 1000000;
            // controller_reference.attitude_z += degreesToRadians(90);


            break;
        }
        case Maneuver::Type::reverse_right:
        {
            /*
             *  Set the X position controller reference to be way below the
             *  current X position, i.e., reverse until the next maneuver, and
             *  the Z attitude controller reference to be 90 degrees below the
             *  current Z attitude, i.e., reverse right 90 degrees.
             */

             /* Create the object */
            controller_reference = controller_->getControllerReference();
            /* Get data from sensors*/
            IMUData bmx_data = sensor_->getIMUDataBMX160();
            EncoderData enc_data = sensor_->getEncoderData();
            /* Change current values depending on sensor values received */
            controller_reference.position_x = enc_data.position_x - 1000000;
            controller_reference.attitude_z = bmx_data.attitude_z - degreesToRadians(90);



            // controller_reference = controller_->getControllerReference();
            // controller_reference.position_x -= 1000000;
            // controller_reference.attitude_z -= degreesToRadians(90);


            break;
        }
        case Maneuver::Type::drive:
        {
            /*
             *  Set the X position controller reference to be way above the
             *  current X position, i.e., reverse until the next maneuver, and
             *  the Z attitude controller reference to be the current Z attitude,
             *  i.e., stay at the current Z attitude.
             */

             // controller_reference = controller_->getControllerReference();
             // controller_reference.position_x += 1000000;

             /* Create the object */
            controller_reference = controller_->getControllerReference();
            /* Get data from sensors*/
            IMUData bmx_data = sensor_->getIMUDataBMX160();
            EncoderData enc_data = sensor_->getEncoderData();
            /* Change current values depending on sensor values received */
            controller_reference.position_x = enc_data.position_x + 1000000;
            controller_reference.attitude_z = bmx_data.attitude_z;

            break;
        }
        case Maneuver::Type::drive_left:
        {
            /*
             *  Set the X position controller reference to be way above the
             *  current X position, i.e., reverse until the next maneuver, and
             *  the Z attitude controller reference to be 90 degrees below the
             *  current Z attitude, i.e., drive left 90 degrees.
             */
             // controller_reference = controller_->getControllerReference();
             // controller_reference.position_x += 1000000;
             // controller_reference.attitude_z -= degreesToRadians(90);

             /* Create the object */
            controller_reference = controller_->getControllerReference();
            /* Get data from sensors*/
            IMUData bmx_data = sensor_->getIMUDataBMX160();
            EncoderData enc_data = sensor_->getEncoderData();
            /* Change current values depending on sensor values received */
            controller_reference.position_x = enc_data.position_x + 1000000;
            controller_reference.attitude_z = bmx_data.attitude_z - degreesToRadians(90);


            break;
        }
        case Maneuver::Type::drive_right:
        {
            /*
             *  Set the X position controller reference to be way below the
             *  current X position, i.e., reverse until the next maneuver, and
             *  the Z attitude controller reference to be 90 degrees above the
             *  current Z attitude, i.e., drive right 90 degrees.
             */

             // controller_reference = controller_->getControllerReference();
             // controller_reference.position_x -= 1000000;
             // controller_reference.attitude_z += degreesToRadians(90);

             /* Create the object */
            controller_reference = controller_->getControllerReference();
            /* Get data from sensors*/
            IMUData bmx_data = sensor_->getIMUDataBMX160();
            EncoderData enc_data = sensor_->getEncoderData();
            /* Change current values depending on sensor values received */
            controller_reference.position_x = enc_data.position_x + 1000000;
            controller_reference.attitude_z = bmx_data.attitude_z + degreesToRadians(90);


            break;
        }
        default:
        {
            /*
             *  Unknown maneuver type. Log warning message.
             */
            Serial(LogLevel::warn) << "Unknown maneuver type.";
            break;
        }
        }

        /*
         *  Return the generated controller reference.
         */
        return controller_reference;
    }
}   // namespace biped
