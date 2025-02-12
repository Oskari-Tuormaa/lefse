#ifndef LEFSE_GPIO_HPP_INCLUDED
#define LEFSE_GPIO_HPP_INCLUDED

#include "sg14/inplace_function.h"

#include <zephyr/drivers/gpio.h>

namespace lefse
{

/**
 * @enum gpio_direction
 * @brief Defines the possible directions for a GPIO pin.
 */
enum class gpio_direction
{
    input        = GPIO_INPUT,       /**< GPIO is configured as an input. */
    output       = GPIO_OUTPUT,      /**< GPIO is configured as an output. */
    disconnected = GPIO_DISCONNECTED /**< GPIO is disconnected. */
};

/**
 * @enum gpio_pull
 * @brief Defines the possible pull configurations for a GPIO pin.
 */
enum class gpio_pull
{
    none      = 0,             /**< No pull resistor is enabled. */
    pull_up   = GPIO_PULL_UP,  /**< Pull-up resistor is enabled. */
    pull_down = GPIO_PULL_DOWN /**< Pull-down resistor is enabled. */
};

/**
 * @enum gpio_output_init
 * @brief Defines the possible initial states for a GPIO output pin.
 */
enum class gpio_output_init
{
    none    = 0,                       /**< No specific initial state. */
    low     = GPIO_OUTPUT_INIT_LOW,    /**< Output initializes to low state. */
    high    = GPIO_OUTPUT_INIT_HIGH,   /**< Output initializes to high state. */
    logical = GPIO_OUTPUT_INIT_LOGICAL /**< Output initializes to logical level. */
};

/**
 * @enum gpio_interrupt
 * @brief Defines the possible interrupt configurations for a GPIO pin.
 */
enum class gpio_interrupt
{
    none         = 0,                     /**< No interrupt is enabled. */
    edge_falling = GPIO_INT_EDGE_FALLING, /**< Interrupt on falling edge. */
    edge_rising  = GPIO_INT_EDGE_RISING,  /**< Interrupt on rising edge. */
    edge_both    = GPIO_INT_EDGE_BOTH     /**< Interrupt on both edges. */
};

/**
 * @class gpio_cb
 * @brief Represents a GPIO callback handler.
 *
 * This class encapsulates a GPIO callback, allowing users to associate a function
 * with GPIO events. It uses `stdext::inplace_function<void()>` for the callback,
 * ensuring efficient and flexible callback handling.
 */
class gpio_cb
{
public:
    using native_type          = struct gpio_callback;
    using native_pointer       = native_type*;
    using native_const_pointer = const native_type*;
    using callback_type        = stdext::inplace_function<void()>;

    /**
     * @brief Constructs a GPIO callback object with a specified callback function and pin mask.
     *
     * @param callback The callback function to invoke on a GPIO event.
     * @param pin_mask The GPIO pin mask for which this callback is registered.
     */
    gpio_cb(callback_type callback, gpio_port_pins_t pin_mask) noexcept
        : callback_ { callback }
    {
        gpio_init_callback(
            &callback_data_,
            [](const struct device* /*port*/, struct gpio_callback* cb, gpio_port_pins_t /*pins*/)
            {
                auto* callee = static_cast<gpio_cb*>(CONTAINER_OF(cb, gpio_cb, callback_data_));
                if (callee->callback_)
                    callee->callback_();
            },
            pin_mask);
    }

    /**
     * @brief Constructs a GPIO callback object with no initial callback function.
     *
     * @param pin_mask The GPIO pin mask for which this callback is registered.
     */
    gpio_cb(gpio_port_pins_t pin_mask) noexcept
        : gpio_cb(nullptr, pin_mask)
    {
    }

    /**
     * @brief Sets the callback function for the GPIO event.
     *
     * @param callback The callback function to invoke.
     */
    void set_callback(callback_type callback) noexcept { callback_ = callback; }

    /**
     * @brief Retrieves a pointer to the native GPIO callback structure.
     *
     * @return A mutable pointer to the native GPIO callback structure.
     */
    native_pointer native_handle() noexcept { return &callback_data_; }

    /**
     * @brief Retrieves a pointer to the native GPIO callback structure (const version).
     *
     * @return A constant pointer to the native GPIO callback structure.
     */
    native_const_pointer native_handle() const noexcept { return &callback_data_; }

private:
    native_type   callback_data_;
    callback_type callback_;
};

/**
 * @brief Base class for GPIO operations, providing common functionality.
 *
 * This class serves as a template base for GPIO handling, allowing derived classes
 * to define their specific GPIO implementations. It provides methods to configure,
 * control, and interact with GPIO pins.
 */
template <typename Gpio_T>
class gpio_base
{
public:
    using native_type          = struct gpio_dt_spec;
    using native_pointer       = native_type*;
    using native_const_pointer = const native_type*;

    /**
     * @brief Checks if the GPIO is ready for use.
     *
     * @return True if the GPIO is ready, false otherwise.
     */
    bool is_ready() noexcept { return gpio_is_ready_dt(native_handle()); }

    /**
     * @brief Configures the GPIO with the given parameters.
     *
     * @param direction The GPIO direction (input, output, or disconnected).
     * @param pull The pull-up or pull-down resistor configuration (default: none).
     * @param output_init Initial output state (default: none).
     * @return Zero on success, or a negative error code on failure.
     */
    int configure(gpio_direction   direction,
                  gpio_pull        pull        = gpio_pull::none,
                  gpio_output_init output_init = gpio_output_init::none) noexcept
    {
        gpio_flags_t flags = static_cast<gpio_flags_t>(direction) | static_cast<gpio_flags_t>(pull)
                             | static_cast<gpio_flags_t>(output_init);
        return gpio_pin_configure_dt(native_handle(), flags);
    }

    /**
     * @brief Configures the GPIO interrupt trigger mode.
     *
     * @param trigger The interrupt trigger type (e.g., rising edge, falling edge, etc.).
     * @return Zero on success, or a negative error code on failure.
     */
    int configure_interrupt(gpio_interrupt trigger)
    {
        gpio_flags_t flags = static_cast<gpio_flags_t>(trigger);
        return gpio_pin_interrupt_configure_dt(native_handle(), flags);
    }

    /**
     * @brief Adds a callback for GPIO events.
     *
     * @param callback The GPIO callback object.
     * @return Zero on success, or a negative error code on failure.
     */
    int add_callback(gpio_cb callback)
    {
        return gpio_add_callback_dt(native_handle(), callback.native_handle());
    }

    /**
     * @brief Removes a previously added GPIO callback.
     *
     * @param callback The GPIO callback object.
     * @return Zero on success, or a negative error code on failure.
     */
    int remove_callback(gpio_cb callback)
    {
        return gpio_remove_callback_dt(native_handle(), callback.native_handle());
    }

    /**
     * @brief Sets the GPIO output to a specified value.
     *
     * @param value The output value (0 or 1).
     * @return Zero on success, or a negative error code on failure.
     */
    int set(int value) noexcept { return gpio_pin_set_dt(native_handle(), value); }

    /**
     * @brief Reads the current value of the GPIO pin.
     *
     * @return The current GPIO pin value (0 or 1), or a negative error code on failure.
     */
    int get() noexcept { return gpio_pin_get_dt(native_handle()); }

    /**
     * @brief Toggles the GPIO output value.
     *
     * @return Zero on success, or a negative error code on failure.
     */
    int toggle() noexcept { return gpio_pin_toggle_dt(native_handle()); }

    /**
     * @brief Retrieves a pointer to the native GPIO descriptor.
     *
     * @return A mutable pointer to the native GPIO descriptor.
     */
    native_pointer native_handle() noexcept { return static_cast<Gpio_T*>(this)->native_handle(); }

    /**
     * @brief Retrieves a pointer to the native GPIO descriptor (const version).
     *
     * @return A constant pointer to the native GPIO descriptor.
     */
    native_const_pointer native_handle() const noexcept
    {
        return static_cast<Gpio_T*>(this)->native_handle();
    }
};

/**
 * @brief GPIO class that directly owns a GPIO descriptor.
 *
 * This class represents a GPIO pin and provides an interface for configuration
 * and control. It derives from `gpio_base` to inherit common GPIO functionality.
 */
class gpio : public gpio_base<gpio>
{
public:
    /**
     * @brief Constructs a GPIO object with a specified descriptor.
     *
     * @param spec The GPIO descriptor.
     */
    explicit gpio(gpio_dt_spec spec)
        : gpio_dt_spec_ { spec }
    {
    }

    /**
     * @brief Retrieves a pointer to the native GPIO descriptor.
     *
     * @return A mutable pointer to the native GPIO descriptor.
     */
    native_pointer native_handle() noexcept { return &gpio_dt_spec_; }

    /**
     * @brief Retrieves a pointer to the native GPIO descriptor (const version).
     *
     * @return A constant pointer to the native GPIO descriptor.
     */
    native_const_pointer native_handle() const noexcept { return &gpio_dt_spec_; }

private:
    native_type gpio_dt_spec_;
};

/**
 * @brief GPIO reference class that holds a reference to a GPIO descriptor.
 *
 * This class allows for referencing existing GPIO objects without owning them.
 * It provides the same functionality as `gpio` but operates on external descriptors.
 */
class gpio_ref : public gpio_base<gpio_ref>
{
public:
    /**
     * @brief Constructs a GPIO reference object from a GPIO descriptor pointer.
     *
     * @param spec Pointer to the GPIO descriptor.
     */
    explicit gpio_ref(gpio_dt_spec* spec)
        : gpio_dt_spec_ { spec }
    {
    }

    /**
     * @brief Constructs a GPIO reference object from another GPIO object.
     *
     * @param gpio The GPIO object to reference.
     */
    template <typename Gpio_T>
    explicit gpio_ref(Gpio_T gpio)
        : gpio_dt_spec_(gpio.native_handle())
    {
    }

    /**
     * @brief Assigns a new GPIO descriptor pointer.
     *
     * @param spec Pointer to the new GPIO descriptor.
     * @return A pointer to this `gpio_ref` object.
     */
    gpio_ref* operator=(gpio_dt_spec* spec)
    {
        gpio_dt_spec_ = spec;
        return this;
    }

    /**
     * @brief Assigns a new GPIO object reference.
     *
     * @param gpio The GPIO object to reference.
     * @return A pointer to this `gpio_ref` object.
     */
    template <typename Gpio_T>
    gpio_ref* operator=(Gpio_T gpio)
    {
        gpio_dt_spec_ = gpio.native_handle();
        return this;
    }

    /**
     * @brief Retrieves a pointer to the native GPIO descriptor.
     *
     * @return A mutable pointer to the native GPIO descriptor.
     */
    native_pointer native_handle() noexcept { return gpio_dt_spec_; }

    /**
     * @brief Retrieves a pointer to the native GPIO descriptor (const version).
     *
     * @return A constant pointer to the native GPIO descriptor.
     */
    native_const_pointer native_handle() const noexcept { return gpio_dt_spec_; }

private:
    native_pointer gpio_dt_spec_;

public:
    gpio_ref() = delete;
};

} // namespace lefse

#endif // LEFSE_GPIO_HPP_INCLUDED
