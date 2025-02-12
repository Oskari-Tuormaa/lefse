#ifndef LEFSE_TIMER_HPP_INCLUDED
#define LEFSE_TIMER_HPP_INCLUDED

#include "sg14/inplace_function.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>

namespace lefse
{

/**
 * @brief Base class for Timer operations, providing common functionality.
 *
 * This class serves as a template base for Timer handling, allowing derived
 * classes to define their specific Timer implementations.
 */
template <typename Timer_T>
class timer_base
{
public:
    using native_type          = struct k_timer;
    using native_pointer       = native_type*;
    using native_const_pointer = const native_type*;

    /**
     * @brief Start a timer.
     *
     * This method starts a timer, and resets its status to zero. The timer begins counting down
     * using the specified duration and period values.
     *
     * Attempting to start a timer that is already running is permitted. The timer's status is reset
     * to zero and the timer begins counting down using the new duration and period values.
     *
     * @param duration Initial timer duration.
     * @param period Timer period.
     */
    void start(k_timeout_t duration, k_timeout_t period) noexcept
    {
        k_timer_start(native_handle(), duration, period);
    }

    /**
     * @brief Start a one-shot timer.
     *
     * This method starts a timer, and resets its status to zero. The timer begins counting down
     * using the specified duration.
     *
     * Attempting to start a timer that is already running is permitted. The timer's status is reset
     * to zero and the timer begins counting down using the new duration.
     *
     * @param duration Timer duration.
     */
    void start(k_timeout_t duration) noexcept
    {
        k_timer_start(native_handle(), duration, K_NO_WAIT);
    }

    /**
     * @brief Stop a timer.
     *
     * This method stops a running timer prematurely. The timer's stop function, if one exists, is
     * invoked by the caller.
     *
     * Attempting to stop a timer that is not running is permitted, but has no effect on the timer.
     *
     * @note The stop handler has to be callable from ISRs if @a k_timer_stop is to be called from
     * ISRs.
     */
    void stop() noexcept
    {
        k_timer_stop(native_handle());
    }

    /**
     * @brief Read timer status.
     *
     * This method reads the timer's status, which indicates the number of times it has expired
     * since its status was last read. Calling this method resets the timer's status to zero.
     *
     * @return Timer status.
     */
    auto status_get() noexcept
    {
        return k_timer_status_get(native_handle());
    }

    /**
     * @brief Synchronize thread to timer expiration.
     *
     * This method blocks the calling thread until the timer's status is non-zero (indicating that
     * it has expired at least once since it was last examined) or the timer is stopped. If the
     * timer status is already non-zero, or the timer is already stopped, the caller continues
     * without waiting.
     *
     * Calling this method resets the timer's status to zero.
     *
     * This method must not be used by interrupt handlers, since they are not allowed to block.
     *
     * @return Timer status.
     */
    auto sync() noexcept
    {
        return k_timer_status_sync(native_handle());
    }

    /**
     * @brief Get next expiration time of a timer, in system ticks
     *
     * This method returns the future system uptime reached at the next time of expiration of the
     * timer, in units of system ticks. If the timer is not running, current system time is
     * returned.
     *
     * @return Uptime of expiration, in ticks
     */
    auto expires_ticks() noexcept
    {
        return k_timer_expires_ticks(native_handle());
    }

    /**
     * @brief Get time remaining before a timer next expires, in system ticks
     *
     * This method computes the time remaining before a running timer next expires, in units of
     * system ticks. If the timer is not running, it returns zero.
     *
     * @return Remaining time until expiration, in ticks
     */
    auto remaining_ticks() noexcept
    {
        return k_timer_remaining_ticks(native_handle());
    }

    /**
     * @brief Retrieves a pointer to the native Timer descriptor.
     *
     * @return A mutable pointer to the native Timer descriptor.
     */
    native_pointer native_handle() noexcept
    {
        return static_cast<Timer_T*>(this)->native_handle();
    }

    /**
     * @brief Retrieves a pointer to the native Timer descriptor (const version).
     *
     * @return A constant pointer to the native Timer descriptor.
     */
    native_const_pointer native_handle() const noexcept
    {
        return static_cast<Timer_T*>(this)->native_handle();
    }
};

/**
 * @brief Timer class that directly owns a Timer descriptor.
 */
class timer : public timer_base<timer>
{
public:
    using callback_type = stdext::inplace_function<void()>;

    /**
     * @brief Constructs a timer given expiry and stop callbacks.
     * This method initializes a timer, prior to its first use.
     *
     * @param expiry_callback Function to invoke each time the timer expires.
     * @param stop_callback Function to invoke if the timer is stopped while running.
     */
    timer(callback_type expiry_callback, callback_type stop_callback) noexcept
        : expiry_callback_ { expiry_callback }
        , stop_callback_ { stop_callback }
    {
        k_timer_expiry_t expiry_fn = [](k_timer* t) noexcept
        {
            auto self = get_user_data(t);
            if (self != nullptr && self->expiry_callback_)
            {
                self->expiry_callback_();
            }
        };

        k_timer_stop_t stop_fn = [](k_timer* t) noexcept
        {
            auto self = get_user_data(t);
            if (self != nullptr && self->stop_callback_)
            {
                self->stop_callback_();
            }
        };

        k_timer_init(&timer_, expiry_fn, stop_fn);
        k_timer_user_data_set(native_handle(), this);
    }

    /**
     * @brief Constructs a timer given only a expiry callback.
     * This method initializes a timer, prior to its first use.
     *
     * @param expiry_callback Function to invoke each time the timer expires.
     */
    explicit timer(callback_type expiry_callback) noexcept
        : timer(expiry_callback, NULL)
    {
    }

    /**
     * @brief Constructs a timer with neither expiry nor stop callbacks.
     * This method initializes a timer, prior to its first use.
     */
    timer() noexcept
        : timer(NULL, NULL)
    {
    }

    /**
     * @brief Sets the expiry callback function.
     *
     * @param expiry_callback Function to invoke each time the timer expires.
     */
    void set_expiry_callback(callback_type expiry_callback)
    {
        expiry_callback_ = expiry_callback;
    }

    /**
     * @brief Sets the stop callback function.
     *
     * @param stop_callback Function to invoke if the timer is stopped while running.
     */
    void set_stop_callback(callback_type stop_callback)
    {
        stop_callback_ = stop_callback;
    }

    /**
     * @brief Retrieves a pointer to the native Timer descriptor.
     *
     * @return A mutable pointer to the native Timer descriptor.
     */
    native_pointer native_handle() noexcept
    {
        return &timer_;
    }

    /**
     * @brief Retrieves a pointer to the native Timer descriptor (const version).
     *
     * @return A constant pointer to the native Timer descriptor.
     */
    native_const_pointer native_handle() const noexcept
    {
        return &timer_;
    }

private:
    static timer* get_user_data(k_timer* t) noexcept
    {
        return static_cast<timer*>(k_timer_user_data_get(t));
    }

    native_type   timer_;
    callback_type expiry_callback_;
    callback_type stop_callback_;
};

/**
 * @brief Timer reference class that holds a reference to a Timer descriptor.
 *
 * This class allows for referencing existing Timer objects without owning them.
 * It provides nearly the same functionality as `timer` but operates on external
 * descriptors, and does not allow changing expiry and stop callbacks.
 */
class timer_ref : public timer_base<timer_ref>
{
public:
    /**
     * @brief Constructs a Timer reference object from a Timer descriptor pointer.
     *
     * @param timer Pointer to the Timer descriptor.
     */
    timer_ref(native_pointer timer)
        : timer_ { timer }
    {
        __ASSERT_NO_MSG(timer_ != nullptr);
    }

    /**
     * @brief Constructs a Timer reference object from another Timer object.
     *
     * @param timer The Timer object to reference.
     */
    template <typename Timer_T>
    timer_ref(Timer_T timer)
        : timer_ { timer.native_handle() }
    {
        __ASSERT_NO_MSG(timer_ != nullptr);
    }

    /**
     * @brief Assigns a new Timer descriptor pointer.
     *
     * @param spec Pointer to the new Timer descriptor.
     * @return A pointer to this `timer_ref` object.
     */
    timer_ref* operator=(native_pointer timer)
    {
        timer_ = timer;
        __ASSERT_NO_MSG(timer_ != nullptr);
        return this;
    }

    /**
     * @brief Assigns a new Timer object reference.
     *
     * @param gpio The Timer object to reference.
     * @return A pointer to this `timer_ref` object.
     */
    template <typename Timer_T>
    timer_ref* operator=(Timer_T timer)
    {
        timer_ = timer.native_handle();
        __ASSERT_NO_MSG(timer_ != nullptr);
        return this;
    }

    /**
     * @brief Retrieves a pointer to the native Timer descriptor.
     *
     * @return A mutable pointer to the native Timer descriptor.
     */
    native_pointer native_handle() noexcept
    {
        return timer_;
    }

    /**
     * @brief Retrieves a pointer to the native Timer descriptor (const version).
     *
     * @return A constant pointer to the native Timer descriptor.
     */
    native_const_pointer native_handle() const noexcept
    {
        return timer_;
    }

private:
    native_pointer timer_;

public:
    timer_ref() = delete;
};

} // namespace lefse

#endif // LEFSE_TIMER_HPP_INCLUDED
