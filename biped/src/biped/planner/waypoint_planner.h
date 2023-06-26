/**
 *  @file   waypoint_planner.h
 *  @author Simon Yu
 *  @date   01/20/2022
 *  @brief  Waypoint planner class header.
 *
 *  This file defines the waypoint planner class.
 */

/*
 *  Include guard.
 */
#ifndef PLANNER_WAYPOINT_PLANNER_H_
#define PLANNER_WAYPOINT_PLANNER_H_

/*
 *  External headers.
 */
#include <memory>
#include <vector>

/*
 *  Project headers.
 */
#include "planner/planner.h"

/*
 *  Biped namespace.
 */
namespace biped
{
/*
 *  Forward declaration.
 */
struct Waypoint;

/**
 *  @brief  Waypoint planner class.
 *
 *  This class provides functions for creating and
 *  executing a waypoint-based plan. The class
 *  inherits the planner abstract class.
 */
class WaypointPlanner : public Planner
{
public:

    /**
     *  @brief  Waypoint planner class constructor.
     *
     *  This constructor initializes all class member variables.
     *  Additionally, the constructor defines a waypoint-based plan.
     */
    WaypointPlanner();

    /**
     *  @brief  Start the defined waypoint-based plan.
     *
     *  This function starts a waypoint-based plan
     *  defined by the constructor. The function implements its
     *  pure abstract parent in the planner abstract class.
     */
    void IRAM_ATTR
    start() override;

    /**
     *  @brief  Execute the defined waypoint-based plan.
     *
     *  This function executes a waypoint-based plan
     *  defined by the constructor. The function implements its
     *  pure abstract parent in the planner abstract class.
     *  This function is expected to be called periodically.
     */
    int
    plan() override;

private:

    std::shared_ptr<Waypoint> waypoint_;    //!< Current waypoint pointer.
    std::shared_ptr<Waypoint> waypoint_start_;    //!< Start waypoint pointer.
    int waypoint_counter_;  //!< Waypoint counter.
    unsigned long waypoint_timer_;  //!< Waypoint timer, in milliseconds.
    volatile bool plan_started_;    //!< Plan started flag.
    volatile bool waypoint_started_;    //!< Waypoint started flag.
    volatile bool plan_completed_;  //!< Plan completed flag.
};
}   // namespace biped

#endif  // PLANNER_WAYPOINT_PLANNER_H_
