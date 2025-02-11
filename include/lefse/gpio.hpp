#ifndef LEFSE_GPIO_HPP_INCLUDED
#define LEFSE_GPIO_HPP_INCLUDED

#include <zephyr/drivers/gpio.h>

#include <functional>

namespace lefse
{

enum class gpio_direction
{
    input        = GPIO_INPUT,
    output       = GPIO_OUTPUT,
    disconnected = GPIO_DISCONNECTED,
};

enum class gpio_pull
{
    none      = 0,
    pull_up   = GPIO_PULL_UP,
    pull_down = GPIO_PULL_DOWN,
};

enum class gpio_output_init
{
    none    = 0,
    low     = GPIO_OUTPUT_INIT_LOW,
    high    = GPIO_OUTPUT_INIT_HIGH,
    logical = GPIO_OUTPUT_INIT_LOGICAL,
};

enum class gpio_interrupt
{
    none         = 0,
    edge_falling = GPIO_INT_EDGE_FALLING,
    edge_rising  = GPIO_INT_EDGE_RISING,
    edge_both    = GPIO_INT_EDGE_BOTH,
};

class gpio_cb
{
public:
    using native_type          = struct gpio_callback;
    using native_pointer       = native_type*;
    using native_const_pointer = const native_type*;
    using callback_type        = std::function<void()>;

    gpio_cb(callback_type callback, gpio_port_pins_t pin_mask) noexcept
        : callback_ { callback }
    {
        gpio_init_callback(
            &callback_data_,
            [](const struct device* /*port*/, struct gpio_callback* cb,
               gpio_port_pins_t /*pins*/)
            {
                auto* callee = static_cast<gpio_cb*>(
                    CONTAINER_OF(cb, gpio_cb, callback_data_));
                if (callee->callback_)
                    callee->callback_();
            },
            pin_mask);
    }

    gpio_cb(gpio_port_pins_t pin_mask) noexcept
        : gpio_cb(nullptr, pin_mask)
    {
    }

    void set_callback(callback_type callback) noexcept { callback_ = callback; }

    native_pointer native_handle() noexcept { return &callback_data_; }

    native_const_pointer native_handle() const noexcept
    {
        return &callback_data_;
    }

private:
    native_type   callback_data_;
    callback_type callback_;
};

template <typename Gpio_T> class gpio_base
{
public:
    using native_type          = struct gpio_dt_spec;
    using native_pointer       = native_type*;
    using native_const_pointer = const native_type*;

    bool is_ready() noexcept { return gpio_is_ready_dt(native_handle()); }

    int configure(gpio_direction direction, gpio_pull pull = gpio_pull::none,
                  gpio_output_init output_init
                  = gpio_output_init::none) noexcept
    {
        gpio_flags_t flags = static_cast<gpio_flags_t>(direction)
                           | static_cast<gpio_flags_t>(pull)
                           | static_cast<gpio_flags_t>(output_init);
        return gpio_pin_configure_dt(native_handle(), flags);
    }

    int configure_interrupt(gpio_interrupt trigger)
    {
        gpio_flags_t flags = static_cast<gpio_flags_t>(trigger);
        return gpio_pin_interrupt_configure_dt(native_handle(), flags);
    }

    int add_callback(gpio_cb callback)
    {
        return gpio_add_callback_dt(native_handle(), callback.native_handle());
    }

    int remove_callback(gpio_cb callback)
    {
        return gpio_remove_callback_dt(native_handle(),
                                       callback.native_handle());
    }

    int set(int value) noexcept
    {
        return gpio_pin_set_dt(native_handle(), value);
    }

    int get() noexcept { return gpio_pin_get_dt(native_handle()); }

    native_pointer native_handle() noexcept
    {
        return static_cast<Gpio_T*>(this)->native_handle();
    }

    native_const_pointer native_handle() const noexcept
    {
        return static_cast<Gpio_T*>(this)->native_handle();
    }
};

class gpio : public gpio_base<gpio>
{
public:
    gpio(gpio_dt_spec spec)
        : gpio_dt_spec_ { spec }
    {
    }

    native_pointer native_handle() noexcept { return &gpio_dt_spec_; }

    native_const_pointer native_handle() const noexcept
    {
        return &gpio_dt_spec_;
    }

private:
    native_type gpio_dt_spec_;
};

class gpio_ref : public gpio_base<gpio_ref>
{
public:
    gpio_ref(gpio_dt_spec* spec)
        : gpio_dt_spec_ { spec }
    {
    }

    template <typename Gpio_T>
    gpio_ref(Gpio_T gpio)
        : gpio_dt_spec_(gpio.native_handle())
    {
    }

    gpio_ref* operator=(gpio_dt_spec* spec)
    {
        gpio_dt_spec_ = spec;
        return this;
    }

    template <typename Gpio_T> gpio_ref* operator=(Gpio_T gpio)
    {
        gpio_dt_spec_ = gpio.native_handle();
        return this;
    }

    native_pointer native_handle() noexcept { return gpio_dt_spec_; }

    native_const_pointer native_handle() const noexcept
    {
        return gpio_dt_spec_;
    }

private:
    native_pointer gpio_dt_spec_;

public:
    gpio_ref() = delete;
};

} // namespace lefse

#endif // LEFSE_GPIO_HPP_INCLUDED
