/**
 *  @file   planner.h
 *  @author Simon Yu
 *  @date   04/01/2022
 *  @brief  Planner abstract class header.
 *
 *  This file defines the planner abstract class.
 */

/*
 *  Include guard.
 */
#ifndef PLANNER_PLANNER_H_
#define PLANNER_PLANNER_H_

/*
 *  External headers.
 */
#include <esp_attr.h>

/*
 *  Biped namespace.
 */
namespace biped
{
/**
 *  @brief  Planner abstract class.
 *
 *  This abstract class provides pure virtual functions
 *  for various types of planners to inherit and implement.
 */
class Planner
{
public:

    /**
     *  @brief  Planner abstract class pure virtual destructor.
     *
     *  This destructor is pure virtual and prohibits the
     *  instantiation of this abstract class.
     */
    virtual
    ~Planner() = default;

    /**
     *  @brief  Start plan.
     *
     *  This function is pure virtual and its functionality
     *  is to be implemented by any child class.
     */
    virtual void IRAM_ATTR
    start() = 0;

    /**
     *  @brief  Execute plan.
     *
     *  This function is pure virtual and its functionality
     *  is to be implemented by any child class.
     */
    virtual int
    plan() = 0;
};
}   // namespace biped

#endif  // PLANNER_PLANNER_H_
