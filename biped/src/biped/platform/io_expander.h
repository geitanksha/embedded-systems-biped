/**
 *  @file   io_expander.h
 *  @author Simon Yu
 *  @date   12/06/2022
 *  @brief  I/O expander class header.
 *
 *  This file defines the I/O expander class.
 */

/*
 *  Include guard.
 */
#ifndef PLATFORM_IO_EXPANDER_H_
#define PLATFORM_IO_EXPANDER_H_

/*
 *  External headers.
 */
#include <MCP23018.h>
#include <memory>
#include <vector>

/*
 *  Biped namespace.
 */
namespace biped
{
/**
 *  @brief  I/O expander class.
 *
 *  This class provides functions for performing digital
 *  read and write operations to the I/O expander pins,
 *  setting I/O expander pin modes, creating an interrupt
 *  service for the I/O expander pins, as well as for
 *  attaching and detaching interrupt handlers to and from
 *  the I/O expander pins.
 */
class IOExpander
{
public:

    /**
     *  @param  address I2C address.
     *  @brief  I/O expander class constructor.
     *
     *  This constructor initializes all class member variables and
     *  the I/O expander driver.
     */
    IOExpander(const uint8_t& address);

    /**
     *  @return I/O expander driver shared pointer.
     *  @brief  Get I/O expander driver shared pointer.
     *
     *  This function returns the I/O expander driver shared pointer.
     */
    std::shared_ptr<MCP23018>
    get() const;

    /**
     *  @return I/O expander driver raw pointer.
     *  @brief  Get I/O expander driver raw pointer.
     *
     *  This function returns the I/O expander driver raw pointer.
     */
    MCP23018*
    getRaw() const;

    /**
     *  @param  pin I/O expander pin.
     *  @param  handler Interrupt handler.
     *  @param  mode Interrupt mode.
     *  @brief  Attach an interrupt handler to port A.
     *
     *  This function configures the I/O expander to attach the given
     *  interrupt handler to the given pin on port A with the given
     *  interrupt mode.
     */
    void
    attachInterruptPortA(const uint8_t& pin, void
    (*handler)(void), const int& mode);

    /**
     *  @param  pin I/O expander pin.
     *  @param  handler Interrupt handler.
     *  @param  mode Interrupt mode.
     *  @brief  Attach an interrupt handler to port B.
     *
     *  This function configures the I/O expander to attach the given
     *  interrupt handler to the given pin on port B with the given
     *  interrupt mode.
     */
    void
    attachInterruptPortB(const uint8_t& pin, void
    (*handler)(void), const int& mode);

    /**
     *  @param  pin I/O expander pin.
     *  @param  handler Interrupt handler.
     *  @param  arg Arguments to the interrupt handler.
     *  @param  mode Interrupt mode.
     *  @brief  Attach an interrupt handler to port A.
     *
     *  This function configures the I/O expander to attach the given
     *  interrupt handler to the given pin on port A with the given
     *  interrupt mode. The given arguments are passed to the given
     *  interrupt handler when its called.
     */
    void
    attachInterruptArgPortA(const uint8_t& pin, void
    (*handler)(void*), void* arg, const int& mode);

    /**
     *  @param  pin I/O expander pin.
     *  @param  handler Interrupt handler.
     *  @param  arg Arguments to the interrupt handler.
     *  @param  mode Interrupt mode.
     *  @brief  Attach an interrupt handler to port B.
     *
     *  This function configures the I/O expander to attach the given
     *  interrupt handler to the given pin on port B with the given
     *  interrupt mode. The given arguments are passed to the given
     *  interrupt handler when its called.
     */
    void
    attachInterruptArgPortB(const uint8_t& pin, void
    (*handler)(void*), void* arg, const int& mode);

    /**
     *  @param  pin I/O expander pin.
     *  @brief  Detach interrupt handlers from port A.
     *
     *  This function configures the I/O expander to detach all
     *  interrupt handlers from the given pin on port A.
     */
    void
    detachInterruptPortA(const uint8_t& pin);

    /**
     *  @param  pin I/O expander pin.
     *  @brief  Detach interrupt handlers from port B.
     *
     *  This function configures the I/O expander to detach all
     *  interrupt handlers from the given pin on port B.
     */
    void
    detachInterruptPortB(const uint8_t& pin);

    /**
     *  @param  pin I/O expander pin.
     *  @param  mode Pin mode.
     *  @brief  Set pin mode on port A.
     *
     *  This function sets the mode of the given pin on port A.
     */
    void
    pinModePortA(const uint8_t& pin, const uint8_t& mode);

    /**
     *  @param  pin I/O expander pin.
     *  @param  mode Pin mode.
     *  @brief  Set pin mode on port B.
     *
     *  This function sets the mode of the given pin on port B.
     */
    void
    pinModePortB(const uint8_t& pin, const uint8_t& mode);

    /**
     *  @param  pin I/O expander pin.
     *  @return Pin state.
     *  @brief  Perform a digital read from the given pin on port A.
     *
     *  This function performs a digital read from the given pin
     *  on port A and returns the state of the pin read.
     */
    bool
    digitalReadPortA(const uint8_t& pin);

    /**
     *  @param  pin I/O expander pin.
     *  @return Pin state.
     *  @brief  Perform a digital read from the given pin on port B.
     *
     *  This function performs a digital read from the given pin
     *  on port B and returns the state of the pin read.
     */
    bool
    digitalReadPortB(const uint8_t& pin);

    /**
     *  @param  pin I/O expander pin.
     *  @param  state Pin state.
     *  @brief  Perform a digital write to the given pin on port A.
     *
     *  This function performs a digital write to the given pin on port A
     *  with the given pin state.
     */
    void
    digitalWritePortA(const uint8_t& pin, const bool& state);

    /**
     *  @param  pin I/O expander pin.
     *  @param  state Pin state.
     *  @brief  Perform a digital write to the given pin on port B.
     *
     *  This function performs a digital write to the given pin on port B
     *  with the given pin state.
     */
    void
    digitalWritePortB(const uint8_t& pin, const bool& state);

    /**
     *  @brief  I/O expander interrupt callback function.
     *
     *  This function processes all the interrupts from the I/O expander.
     */
    void IRAM_ATTR
    onInterrupt();

private:

    /**
     *  @brief  Interrupt handler struct.
     *
     *  This struct contains interrupt handler entries, such as the interrupt
     *  handler function pointers, interrupt handler arguments, and etc.
     */
    struct InterruptHandler
    {
        void
        (*handler)(void); //!< Interrupt handler function pointer, with no arguments.
        void
        (*handler_arg)(void*);  //!< Interrupt handler function pointer, with arguments.
        void *arg;  //!< Arguments pointer.
        int mode;   //!< Interrupt mode.
        bool with_arg;  //!< Arguments flag.

        /**
         *  @brief  Interrupt handler struct constructor.
         *
         *  This constructor initializes all interrupt handler struct entries.
         */
        InterruptHandler() : handler(nullptr), handler_arg(nullptr), arg(nullptr), mode(CHANGE),
                with_arg(false)
        {
        }
    };

    /**
     *  @param  flags I/O expander interrupt flags.
     *  @param  captures I/O expander interrupt captures.
     *  @param  interrupt_handlers I/O expander interrupt handler vector.
     *  @brief  Handle I/O expander interrupts.
     *
     *  This function handles the I/O expander interrupts by examining the
     *  given I/O expander interrupt flags and captures and subsequently
     *  calling the corresponding interrupt handlers from the given I/O
     *  expander interrupt handler vector.
     */
    void
    handleInterrupt(const uint8_t& flags, const uint8_t& captures,
            const std::vector<InterruptHandler>& interrupt_handlers);

    std::vector<InterruptHandler> interrupt_handlers_port_a_;  //!< Port A interrupt handler vector.
    std::vector<InterruptHandler> interrupt_handlers_port_b_;  //!< Port B interrupt handler vector.
    std::shared_ptr<MCP23018> mcp23018_;    //!< MCP23018 I/O expander driver object shared pointer.
};
}   // namespace biped

#endif  // PLATFORM_IO_EXPANDER_H_
