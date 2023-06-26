/**
 *  @file   io_expander.cpp
 *  @author Simon Yu
 *  @date   12/06/2022
 *  @brief  I/O expander class source.
 *
 *  This file implements the I/O expander class.
 */

/*
 *  Project headers.
 */
#include "common/global.h"
#include "platform/io_expander.h"
#include "common/parameter.h"
#include "platform/serial.h"

/*
 *  Biped namespace.
 */
namespace biped
{
IOExpander::IOExpander(const uint8_t& address) :
        interrupt_handlers_port_a_(IOExpanderParameter::num_port_pins),
        interrupt_handlers_port_b_(IOExpanderParameter::num_port_pins)
{
    /*
     *  Instantiate I/O expander driver object.
     */
    mcp23018_ = std::make_shared<MCP23018>(address);

    /*
     *  Initialize I/O expander driver object and
     *  configure I/O expander.
     */
    mcp23018_->begin();
    mcp23018_->writeToRegister(IOCON, IOCON_MIRROR | IOCON_INTPOL | IOCON_INTCC);
    mcp23018_->SetDirections(0xFF, 0xFF);
    mcp23018_->SetPullups(0xFF, 0xFF);
    mcp23018_->writePairToRegister(INTENA, 0x00, 0x00);
}

std::shared_ptr<MCP23018>
IOExpander::get() const
{
    /*
     *  Return I/O expander driver object shared pointer.
     */
    return mcp23018_;
}

MCP23018*
IOExpander::getRaw() const
{
    /*
     *  Return I/O expander driver object raw pointer.
     */
    return mcp23018_.get();
}

void
IOExpander::attachInterruptPortA(const uint8_t& pin, void
(*handler)(void), const int& mode)
{
    /*
     *  Validate pin.
     */
    if (pin >= IOExpanderParameter::num_port_pins)
    {
        Serial(LogLevel::error) << "Invalid pin.";
        return;
    }

    /*
     *  Register interrupt handler.
     */
    interrupt_handlers_port_a_.at(pin).handler = handler;
    interrupt_handlers_port_a_.at(pin).handler_arg = nullptr;
    interrupt_handlers_port_a_.at(pin).arg = nullptr;
    interrupt_handlers_port_a_.at(pin).mode = mode;
    interrupt_handlers_port_a_.at(pin).with_arg = false;

    /*
     *  Set interrupt type based on the given mode.
     */
    switch (mode)
    {
        case RISING:
        {
            mcp23018_->setBitInRegister(INTCONA, pin, false);
            break;
        }
        case FALLING:
        {
            mcp23018_->setBitInRegister(INTCONA, pin, false);
            break;
        }
        case CHANGE:
        {
            mcp23018_->setBitInRegister(INTCONA, pin, false);
            break;
        }
        default:
        {
            Serial(LogLevel::error) << "Unsupported interrupt mode.";
            break;
        }
    }

    /*
     *  Enable interrupt on the given pin.
     */
    mcp23018_->setBitInRegister(INTENA, pin, true);
}

void
IOExpander::attachInterruptPortB(const uint8_t& pin, void
(*handler)(void), const int& mode)
{
    /*
     *  Validate pin.
     */
    if (pin >= IOExpanderParameter::num_port_pins)
    {
        Serial(LogLevel::error) << "Invalid pin.";
        return;
    }

    /*
     *  Register interrupt handler.
     */
    interrupt_handlers_port_b_.at(pin).handler = handler;
    interrupt_handlers_port_b_.at(pin).handler_arg = nullptr;
    interrupt_handlers_port_b_.at(pin).arg = nullptr;
    interrupt_handlers_port_b_.at(pin).mode = mode;
    interrupt_handlers_port_b_.at(pin).with_arg = false;

    /*
     *  Set interrupt type based on the given mode.
     */
    switch (mode)
    {
        case RISING:
        {
            mcp23018_->setBitInRegister(INTCONB, pin, false);
            break;
        }
        case FALLING:
        {
            mcp23018_->setBitInRegister(INTCONB, pin, false);
            break;
        }
        case CHANGE:
        {
            mcp23018_->setBitInRegister(INTCONB, pin, false);
            break;
        }
        default:
        {
            Serial(LogLevel::error) << "Unsupported interrupt mode.";
            break;
        }
    }

    /*
     *  Enable interrupt on the given pin.
     */
    mcp23018_->setBitInRegister(INTENB, pin, true);
}

void
IOExpander::attachInterruptArgPortA(const uint8_t& pin, void
(*handler)(void*), void* arg, const int& mode)
{
    /*
     *  Validate pin.
     */
    if (pin >= IOExpanderParameter::num_port_pins)
    {
        Serial(LogLevel::error) << "Invalid pin.";
        return;
    }

    /*
     *  Register interrupt handler.
     */
    interrupt_handlers_port_a_.at(pin).handler = nullptr;
    interrupt_handlers_port_a_.at(pin).handler_arg = handler;
    interrupt_handlers_port_a_.at(pin).arg = arg;
    interrupt_handlers_port_a_.at(pin).mode = mode;
    interrupt_handlers_port_a_.at(pin).with_arg = true;

    /*
     *  Set interrupt type based on the given mode.
     */
    switch (mode)
    {
        case RISING:
        {
            mcp23018_->setBitInRegister(INTCONA, pin, false);
            break;
        }
        case FALLING:
        {
            mcp23018_->setBitInRegister(INTCONA, pin, false);
            break;
        }
        case CHANGE:
        {
            mcp23018_->setBitInRegister(INTCONA, pin, false);
            break;
        }
        default:
        {
            Serial(LogLevel::error) << "Unsupported interrupt mode.";
            break;
        }
    }

    /*
     *  Enable interrupt on the given pin.
     */
    mcp23018_->setBitInRegister(INTENA, pin, true);
}

void
IOExpander::attachInterruptArgPortB(const uint8_t& pin, void
(*handler)(void*), void* arg, const int& mode)
{
    /*
     *  Validate pin.
     */
    if (pin >= IOExpanderParameter::num_port_pins)
    {
        Serial(LogLevel::error) << "Invalid pin.";
        return;
    }

    /*
     *  Register interrupt handler.
     */
    interrupt_handlers_port_b_.at(pin).handler = nullptr;
    interrupt_handlers_port_b_.at(pin).handler_arg = handler;
    interrupt_handlers_port_b_.at(pin).arg = arg;
    interrupt_handlers_port_b_.at(pin).mode = mode;
    interrupt_handlers_port_b_.at(pin).with_arg = true;

    /*
     *  Set interrupt type based on the given mode.
     */
    switch (mode)
    {
        case RISING:
        {
            mcp23018_->setBitInRegister(INTCONB, pin, false);
            break;
        }
        case FALLING:
        {
            mcp23018_->setBitInRegister(INTCONB, pin, false);
            break;
        }
        case CHANGE:
        {
            mcp23018_->setBitInRegister(INTCONB, pin, false);
            break;
        }
        default:
        {
            Serial(LogLevel::error) << "Unsupported interrupt mode.";
            break;
        }
    }

    /*
     *  Enable interrupt on the given pin.
     */
    mcp23018_->setBitInRegister(INTENB, pin, true);
}

void
IOExpander::detachInterruptPortA(const uint8_t& pin)
{
    /*
     *  Validate pin.
     */
    if (pin >= IOExpanderParameter::num_port_pins)
    {
        Serial(LogLevel::error) << "Invalid pin.";
        return;
    }

    /*
     *  Disable interrupt on the given pin.
     */
    mcp23018_->setBitInRegister(INTENA, pin, false);

    /*
     *  Unregister interrupt handler.
     */
    interrupt_handlers_port_a_.at(pin).handler = nullptr;
    interrupt_handlers_port_a_.at(pin).handler_arg = nullptr;
    interrupt_handlers_port_a_.at(pin).arg = nullptr;
    interrupt_handlers_port_a_.at(pin).mode = ONLOW;
    interrupt_handlers_port_a_.at(pin).with_arg = false;
}

void
IOExpander::detachInterruptPortB(const uint8_t& pin)
{
    /*
     *  Validate pin.
     */
    if (pin >= IOExpanderParameter::num_port_pins)
    {
        Serial(LogLevel::error) << "Invalid pin.";
        return;
    }

    /*
     *  Disable interrupt on the given pin.
     */
    mcp23018_->setBitInRegister(INTENB, pin, false);

    /*
     *  Unregister interrupt handler.
     */
    interrupt_handlers_port_b_.at(pin).handler = nullptr;
    interrupt_handlers_port_b_.at(pin).handler_arg = nullptr;
    interrupt_handlers_port_b_.at(pin).arg = nullptr;
    interrupt_handlers_port_b_.at(pin).mode = ONLOW;
    interrupt_handlers_port_b_.at(pin).with_arg = false;
}

void
IOExpander::pinModePortA(const uint8_t& pin, const uint8_t& mode)
{
    /*
     *  Set pin mode based on the given mode.
     */
    switch (mode)
    {
        case INPUT:
        {
            mcp23018_->setBitInRegister(IODIRA, pin, true);
            mcp23018_->setBitInRegister(GPPUA, pin, false);
            break;
        }
        case INPUT_PULLUP:
        {
            mcp23018_->setBitInRegister(IODIRA, pin, true);
            mcp23018_->setBitInRegister(GPPUA, pin, true);
            break;
        }
        case OUTPUT:
        {
            mcp23018_->setBitInRegister(IODIRA, pin, false);
            mcp23018_->setBitInRegister(GPPUA, pin, true);
            break;
        }
        default:
        {
            Serial(LogLevel::error) << "Unknown pin mode.";
            break;
        }
    }
}

void
IOExpander::pinModePortB(const uint8_t& pin, const uint8_t& mode)
{
    /*
     *  Set pin mode based on the given mode.
     */
    switch (mode)
    {
        case INPUT:
        {
            mcp23018_->setBitInRegister(IODIRB, pin, true);
            mcp23018_->setBitInRegister(GPPUB, pin, false);
            break;
        }
        case INPUT_PULLUP:
        {
            mcp23018_->setBitInRegister(IODIRB, pin, true);
            mcp23018_->setBitInRegister(GPPUB, pin, true);
            break;
        }
        case OUTPUT:
        {
            mcp23018_->setBitInRegister(IODIRB, pin, false);
            mcp23018_->setBitInRegister(GPPUB, pin, true);
            break;
        }
        default:
        {
            Serial(LogLevel::error) << "Unknown pin mode.";
            break;
        }
    }
}

bool
IOExpander::digitalReadPortA(const uint8_t& pin)
{
    /*
     *  Perform a digital read from the given pin
     *  on port A and returns the state of the pin read.
     */
    const uint8_t pins = static_cast<uint8_t>(mcp23018_->GetPortA());
    return static_cast<bool>(pins & (1 << pin));
}

bool
IOExpander::digitalReadPortB(const uint8_t& pin)
{
    /*
     *  Perform a digital read from the given pin
     *  on port B and returns the state of the pin read.
     */
    const uint8_t pins = static_cast<uint8_t>(mcp23018_->GetPortB());
    return static_cast<bool>(pins & (1 << pin));
}

void
IOExpander::digitalWritePortA(const uint8_t& pin, const bool& state)
{
    /*
     *  Perform a digital write to the given pin
     *  on port A with the given pin state.
     */
    mcp23018_->SetAPin(pin, state);
}

void
IOExpander::digitalWritePortB(const uint8_t& pin, const bool& state)
{
    /*
     *  Perform a digital write to the given pin
     *  on port B with the given pin state.
     */
    mcp23018_->SetBPin(pin, state);
}

void IRAM_ATTR
IOExpander::onInterrupt()
{
    /*
     *  Atomically read interrupt flags and captures.
     */
    lock_wire_.lock();
    const uint16_t interrupt_flags = mcp23018_->readPairFromRegister(INTFA);
    const uint16_t interrupt_captures = mcp23018_->readPairFromRegister(INTCAPA);
    lock_wire_.unlock();

    /*
     *  Interpret interrupt flags and captures by port.
     */
    const uint8_t interrupt_flags_port_a = static_cast<uint8_t>(interrupt_flags);
    const uint8_t interrupt_flags_port_b = static_cast<uint8_t>(interrupt_flags
            >> IOExpanderParameter::num_port_pins);
    const uint8_t interrupt_captures_port_a = static_cast<uint8_t>(interrupt_captures);
    const uint8_t interrupt_captures_port_b = static_cast<uint8_t>(interrupt_captures
            >> IOExpanderParameter::num_port_pins);

    /*
     *  Handle interrupt by port.
     */
    handleInterrupt(interrupt_flags_port_a, interrupt_captures_port_a, interrupt_handlers_port_a_);
    handleInterrupt(interrupt_flags_port_b, interrupt_captures_port_b, interrupt_handlers_port_b_);
}

void
IOExpander::handleInterrupt(const uint8_t& flags, const uint8_t& captures,
        const std::vector<InterruptHandler>& interrupt_handlers)
{
    /*
     *  Declare mask.
     */
    uint8_t mask = 1;

    for (uint8_t pin = 0; pin < IOExpanderParameter::num_port_pins; pin ++, mask <<= 1)
    {
        if (flags & mask)
        {
            /*
             *  Validate interrupt handlers.
             */
            if (interrupt_handlers.at(pin).with_arg)
            {
                if (interrupt_handlers.at(pin).handler_arg == nullptr)
                {
                    continue;
                }
            }
            else
            {
                if (interrupt_handlers.at(pin).handler == nullptr)
                {
                    continue;
                }
            }

            /*
             *  Call interrupt handlers based on the pin mode.
             */
            if (interrupt_handlers.at(pin).mode == RISING)
            {
                if (captures & mask)
                {
                    if (interrupt_handlers.at(pin).with_arg)
                    {
                        interrupt_handlers.at(pin).handler_arg(interrupt_handlers.at(pin).arg);
                    }
                    else
                    {
                        interrupt_handlers.at(pin).handler();
                    }
                }
            }
            else if (interrupt_handlers.at(pin).mode == FALLING)
            {
                if (!(captures & mask))
                {
                    if (interrupt_handlers.at(pin).with_arg)
                    {
                        interrupt_handlers.at(pin).handler_arg(interrupt_handlers.at(pin).arg);
                    }
                    else
                    {
                        interrupt_handlers.at(pin).handler();
                    }
                }
            }
            else
            {
                if (interrupt_handlers.at(pin).with_arg)
                {
                    interrupt_handlers.at(pin).handler_arg(interrupt_handlers.at(pin).arg);
                }
                else
                {
                    interrupt_handlers.at(pin).handler();
                }
            }
        }
    }
}
}   // namespace biped
