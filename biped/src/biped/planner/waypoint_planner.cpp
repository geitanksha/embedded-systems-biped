/**
 *  @file   waypoint_planner.cpp
 *  @author Simon Yu
 *  @date   01/20/2022
 *  @brief  Waypoint planner class source.
 *
 *  This file implements the waypoint planner class.
 */

 /*
  *  Project headers.
  */
#include "controller/controller.h"
#include "common/global.h"
#include "utility/math.h"
#include "sensor/sensor.h"
#include "platform/serial.h"
#include "common/type.h"
#include "planner/waypoint_planner.h"

  /*
   *  Biped namespace.
   */
namespace biped
{
    WaypointPlanner::WaypointPlanner() : waypoint_counter_(1), waypoint_timer_(0), plan_started_(false),
        waypoint_started_(false), plan_completed_(true)
    {
        /*
         *  Create a set of waypoints for the example plan.
         *  During their configurations, the waypoints should
         *  be chained up in a linked list fashion.
         */

         // std::shared_ptr<Waypoint> waypoint_1 = std::make_shared<Waypoint>(); // forward
         // std::shared_ptr<Waypoint> waypoint_2 = std::make_shared<Waypoint>(); // reverse
         // std::shared_ptr<Waypoint> waypoint_3 = std::make_shared<Waypoint>();
         // std::shared_ptr<Waypoint> waypoint_4 = std::make_shared<Waypoint>();
         // std::shared_ptr<Waypoint> waypoint_5 = std::make_shared<Waypoint>();

         // /*
         //  *  Set the start and current waypoints.
         //  */
         // waypoint_start_ = waypoint_1;
         // waypoint_ = waypoint_start_;

         // /*
         //  *  Example plan waypoint 1 configuration:
         //  *      - Go to 0 meter X position (0 meter forward) without turning for 2 seconds.
         //  *      - Then, start waypoint 2.
         //  */
         // waypoint_1->controller_reference.attitude_z = degreesToRadians(0);
         // waypoint_1->controller_reference.position_x = 0;
         // waypoint_1->duration = 2;
         // waypoint_1->next = waypoint_2;

         // /*
         //  *  Example plan waypoint 2 configuration:
         //  *      - Go to 1 meter X position (1 meter forward) without turning for 10 seconds.
         //  *      - Then, start waypoint 3.
         //  */
         // waypoint_2->controller_reference.attitude_z = degreesToRadians(0);
         // waypoint_2->controller_reference.position_x = 1;
         // waypoint_2->duration = 10;
         // waypoint_2->next = waypoint_3;

         // /*
         //  *  Example plan waypoint 3 configuration:
         //  *      - Go to 2 meter X position (1 meter forward) while turning right 90 degrees for 10 seconds.
         //  *      - Then, start waypoint 4.
         //  */
         // waypoint_3->controller_reference.attitude_z = degreesToRadians(-90);
         // waypoint_3->controller_reference.position_x = 2;
         // waypoint_3->duration = 10;
         // waypoint_3->next = waypoint_4;

         // /*
         //  *  Example plan waypoint 4 configuration:
         //  *      - Go to 1 meter X position (1 meter backward) while turning right 90 degrees for 10 seconds.
         //  *      - Then, start waypoint 5.
         //  */
         // waypoint_4->controller_reference.attitude_z = degreesToRadians(0);
         // waypoint_4->controller_reference.position_x = 1;
         // waypoint_4->duration = 10;
         // waypoint_4->next = waypoint_5;

         // /*
         //  *  Example plan waypoint 5 configuration:
         //  *      - Go to 0 meter X position (1 meter backward) without turning for 10 seconds.
         //  *      - Then, plan completed.
         //  */
         // waypoint_5->controller_reference.attitude_z = degreesToRadians(0);
         // waypoint_5->controller_reference.position_x = 0;
         // waypoint_5->duration = 10;
         // waypoint_5->next = nullptr;

         /*
          *  Using the example plan above, create your own waypoint-based plan.
          *  Feel free to add or remove waypoints. Also feel free to remove or
          *  comment out the example plan.
          *
          *  Be aware of the singularity in the Z attitude data, i.e., the Z
          *  attitude data goes from 90 degrees to -270 degrees due to the atan2
          *  function. Going across the singularity might cause the Kalman filters
          *  and the controllers to become unstable. Therefore, it is recommended to
          *  try and avoid the Z attitude singularity in your waypoints.



         1. park
         2. forward
         3. forward right
         4. back right
         5. back
         6. forward left
         7. backward left
          */

          /*
           *  Create a set of waypoints for the example plan.
           *  During their configurations, the waypoints should
           *  be chained up in a linked list fashion.
           */

        std::shared_ptr<Waypoint> waypoint_1 = std::make_shared<Waypoint>(); // park
        std::shared_ptr<Waypoint> waypoint_2 = std::make_shared<Waypoint>(); // forward
        std::shared_ptr<Waypoint> waypoint_3 = std::make_shared<Waypoint>(); // forward right
        std::shared_ptr<Waypoint> waypoint_4 = std::make_shared<Waypoint>(); // back right
        std::shared_ptr<Waypoint> waypoint_5 = std::make_shared<Waypoint>(); // back
        std::shared_ptr<Waypoint> waypoint_6 = std::make_shared<Waypoint>(); // forward left
        std::shared_ptr<Waypoint> waypoint_7 = std::make_shared<Waypoint>(); // backward left

        /*
         *  Set the start and current waypoints.
         */
        waypoint_start_ = waypoint_1;
        waypoint_ = waypoint_start_;

        /*
         *  Waypoint 1 configuration:
         *      - Go to 0 meter X position (0 meter forward) without turning for 2 seconds.
         *      - Then, start waypoint 2.
         */
        waypoint_1->controller_reference.attitude_z = degreesToRadians(0);
        waypoint_1->controller_reference.position_x = 0;
        waypoint_1->duration = 2;
        waypoint_1->next = waypoint_2;

        /*
         *  Waypoint 2 configuration:
         *      - Go to 1 meter X position (1 meter forward) without turning for 7 seconds.
         *      - Then, start waypoint 3.
         */
        waypoint_2->controller_reference.attitude_z = degreesToRadians(0);
        waypoint_2->controller_reference.position_x = 1;
        waypoint_2->duration = 7;
        waypoint_2->next = waypoint_3;


        /*
         *  Waypoint 3 configuration:
         *      - Go to 2 meter X position (1 meter forward) while turning right 90 degrees for 7 seconds.
         *      - Then, start waypoint 4.
         */
        waypoint_3->controller_reference.attitude_z = degreesToRadians(-90);
        waypoint_3->controller_reference.position_x = 2;
        waypoint_3->duration = 7;
        waypoint_3->next = waypoint_4;

        /*
         *  Waypoint 4 configuration:
         *      - Go to 1 meter X position (1 meter backward) while turning right 90 degrees for 7 seconds.
         *      - Then, start waypoint 5.
         */
        waypoint_4->controller_reference.attitude_z = degreesToRadians(-90);
        waypoint_4->controller_reference.position_x = 1;
        waypoint_4->duration = 7;
        waypoint_4->next = waypoint_5;

        /*
         *  Waypoint 5 configuration:
         *      - Go to 0 meter X position (1 meter backward) without turning for 7 seconds.
         *      - Then, plan completed.
         */
        waypoint_5->controller_reference.attitude_z = degreesToRadians(0);
        waypoint_5->controller_reference.position_x = 0;
        waypoint_5->duration = 7;
        waypoint_5->next = waypoint_6;

        /*
         *  Waypoint 6 configuration:
         *      - Go to 1 meter X position (1 meter forward) while turning left 90 degrees for 7 seconds.
         *      - Then, plan completed.
         */
        waypoint_6->controller_reference.attitude_z = degreesToRadians(90);
        waypoint_6->controller_reference.position_x = 1;
        waypoint_6->duration = 7;
        waypoint_6->next = waypoint_7;

        /*
         *  Waypoint 7 configuration:
         *      - Go to 0 meter X position (1 meter backward) while turning left 90 degrees for 7 seconds.
         *      - Then, plan completed.
         */
        waypoint_7->controller_reference.attitude_z = degreesToRadians(90);
        waypoint_7->controller_reference.position_x = 0;
        waypoint_7->duration = 7;
        waypoint_7->next = nullptr;
    }

    void IRAM_ATTR
        WaypointPlanner::start()
    {
        /*
         *  If the plan is completed, reset the waypoint pointer to the
         *  start waypoint pointer, reset the waypoint counter to 1,
         *  mark waypoint as not started, mark the plan as not started
         *  and not completed.
         */

        if (plan_completed_)
        {
            waypoint_ = waypoint_start_;
            waypoint_counter_ = 1;
            waypoint_started_ = false;
            plan_started_ = false;
            plan_completed_ = false;
        }

    }

    int
        WaypointPlanner::plan()
    {
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

        if ((controller_->getActiveStatus() == false) || (plan_completed_ == true))
        {
            return -1;
        }


        /*
         *  Mark the plan as not started and completed, and return
         *  -1 if the plan has started, has not completed, but the
         *  current waypoint is null.
         */

        if (!waypoint_ && plan_started_ && !plan_completed_)
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
             *  but the current waypoint is already null.
             */
            if (!waypoint_)
            {
                Serial(LogLevel::error) << "Empty waypoint-based plan.";
                return -1;
            }

            /*
             *  Mark the plan as started if it has not started.
             */
            Serial(LogLevel::info) << "Started waypoint-based plan.";
            plan_started_ = true;
        }

        if (!waypoint_started_)
        {
            /*
             *  Execute the current waypoint if it has not started.
             *  Set the controller reference from the current waypoint
             *  to the controller, update the waypoint timer to the
             *  current time, and mark the current waypoint as started.
             */

             // plan();
            controller_->setControllerReference(waypoint_->controller_reference);
            waypoint_timer_ = millis();
            waypoint_started_ = true;

        }
        else
        {
            /*
             *  If the elapsed duration since the last waypoint timer
             *  update goes above the duration value from the current
             *  waypoint, transtion to the next waypoint, increment
             *  the waypoint counter, and mark the current waypoint
             *  as not started.
             */

            if ((millis() - waypoint_timer_) > (waypoint_->duration * 1000))
            {
                waypoint_ = waypoint_->next;
                waypoint_counter_++;
                waypoint_started_ = false;
            }
        }

        /*
         *  Return the waypoint counter.
         */
        return waypoint_counter_;
    }
}   // namespace biped
