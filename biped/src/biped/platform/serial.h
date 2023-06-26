/**
 *  @file   serial.h
 *  @author Simon Yu
 *  @date   11/19/2021
 *  @brief  Serial class header.
 *
 *  This file defines the serial class.
 */

/*
 *  Include guard.
 */
#ifndef PLATFORM_SERIAL_H_
#define PLATFORM_SERIAL_H_

/*
 *  External headers.
 */
#include <sstream>

/*
 *  Project headers.
 */
#include "common/type.h"

/*
 *  Biped namespace.
 */
namespace biped
{
/**
 *  @brief  Serial class.
 *
 *  This class provides functions for printing to
 *  the serial connection.
 */
class Serial
{
public:

    /**
     *  @param  log_level Log level.
     *  @param  raw Whether to print in raw mode.
     *  @brief  Serial class constructor.
     *
     *  This constructor initializes all class member variables and
     *  streams log level tags to the class member string stream buffer
     *  depending on the printing mode.
     */
    Serial(const LogLevel& log_level, const bool& raw = false);

    /**
     *  @brief  Serial class destructor.
     *
     *  This destructor flushes the class member string stream buffer
     *  to the serial connection.
     */
    ~Serial();

    /**
     *  @brief  Initialize serial connection.
     *
     *  This function initializes the serial driver.
     */
    static void
    initialize();

    /**
     *  @return Log level.
     *  @brief  Get the current worst log level.
     *
     *  This function returns the current worst log level.
     */
    static LogLevel
    getLogLevelWorst();

    /**
     *  @param  log_level_max Maximum log level.
     *  @brief  Set the maximum log level.
     *
     *  This function sets the maximum log level.
     */
    static void
    setLogLevelMax(const LogLevel& log_level_max);

    /**
     *  @param  item Stream manipulator to perform.
     *  @return Serial object reference.
     *  @brief  Perform the given stream manipulator.
     *
     *  This operator function interprets the given stream manipulator,
     *  performs their respective functionalities, and returns a reference
     *  to this object.
     */
    Serial&
    operator<<(const StreamManipulator& item);

    /**
     *  @tparam Type Type of data to stream.
     *  @param  item data to stream.
     *  @return Serial object reference.
     *  @brief  Stream the given data.
     *
     *  This operator function streams the given data to the string stream
     *  buffer and returns a reference to this object. If the class member
     *  log level is greater than the class member maximum log level, the
     *  data would not be streamed.
     */
    template<typename Type>
    inline Serial&
    operator<<(const Type& item)
    {
        /*
         *  Stream the given data only if the class member log level
         *  is less than or equal to the class member maximum log level.
         */
        if (log_level_ > log_level_max_)
        {
            return *this;
        }

        /*
         *  Stream the given data to the string stream buffer.
         */
        ss_ << item;

        /*
         *  Return a reference to this object.
         */
        return *this;
    }

private:

    const LogLevel log_level_;  //!< Log level.
    static LogLevel log_level_max_; //!< Maximum log level.
    static LogLevel log_level_worst_;   //!< Current worst log level.
    const bool raw_;    //!< Raw printing mode flag.
    std::stringstream ss_;  //!< String stream buffer.
};
}   // namespace biped

#endif  // PLATFORM_SERIAL_H_
