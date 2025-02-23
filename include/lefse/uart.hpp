#ifndef LEFSE_UART_HPP_INCLUDED
#define LEFSE_UART_HPP_INCLUDED

#ifdef CONFIG_SERIAL

#include "sg14/inplace_function.h"

#include <span>
#include <string_view>
#include <utility>
#include <variant>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

namespace lefse
{

#ifdef CONFIG_UART_ASYNC_API

/**
 * @brief Contains a struct for each async event type, containing relevant data.
 */
namespace async_events
{

/** @brief Whole TX buffer was transmitted. */
struct tx_done
{
    /** std::span to current buffer. */
    const std::span<const uint8_t> span;
};

/**
 * @brief Transmitting aborted due to timeout or uart_tx_abort call
 *
 * When flow control is enabled, there is a possibility that TX transfer
 * won't finish in the allotted time. Some data may have been
 * transferred, information about it can be found in event data.
 */
struct tx_aborted
{
    /** @brief std::span to current buffer. */
    const std::span<const uint8_t> span;
};

/**
 * @brief Received data is ready for processing.
 *
 * This event is generated in the following cases:
 * - When RX timeout occurred, and data was stored in provided buffer.
 *   This can happen multiple times in the same buffer.
 * - When provided buffer is full.
 * - After uart::rx_disable().
 * - After stopping due to external event (#rx_stopped).
 */
struct rx_ready
{
    /** @brief std::span to received data. */
    std::span<uint8_t> span;
};

/**
 * @brief Driver requests next buffer for continuous reception.
 *
 * This event is triggered when receiving has started for a new buffer,
 * i.e. it's time to provide a next buffer for a seamless switchover to
 * it. For continuous reliable receiving, user should provide another RX
 * buffer in response to this event, using uart::rx_buf_rsp method
 *
 * If uart::rx_buf_rsp is not called before current buffer
 * is filled up, receiving will stop.
 */
struct rx_buf_request
{
};

/**
 * @brief Buffer is no longer used by UART driver.
 */
struct rx_buf_released
{
    /** @brief Pointer to buffer that is no longer in use. */
    uint8_t* buf;
};

/**
 * @brief RX has been disabled and can be reenabled.
 *
 * This event is generated whenever receiver has been stopped, disabled
 * or finished its operation and can be enabled again using
 * uart::rx_enable
 */
struct rx_disabled
{
};

/**
 * @brief RX has stopped due to external event.
 *
 * Reason is one of uart_rx_stop_reason.
 */
struct rx_stopped
{
    /** @brief Reason why receiving stopped */
    uart_rx_stop_reason reason;
    /** @brief Last received data. */
    rx_ready data;
};

} // namespace async_events

/**
 * @brief std::variant type for all possible async_events.
 */
using async_event = std::variant<async_events::rx_stopped,
                                 async_events::rx_disabled,
                                 async_events::rx_buf_released,
                                 async_events::rx_buf_request,
                                 async_events::rx_ready,
                                 async_events::tx_aborted,
                                 async_events::tx_done>;

namespace details
{

/**
 * @brief Converts a given uart_event struct to matching async_event variant.
 *
 * @param evt uart_event struct to convert
 * @return Event converted to matching async_event variant.
 */
inline constexpr async_event event_struct_to_variant(const uart_event& evt)
{
    using namespace async_events;
    switch (evt.type)
    {
        case UART_TX_DONE: return tx_done { { evt.data.tx.buf, evt.data.tx.len } };
        case UART_TX_ABORTED: return tx_aborted { { evt.data.tx.buf, evt.data.tx.len } };
        case UART_RX_RDY:
            return rx_ready { { evt.data.rx.buf + evt.data.rx.offset, evt.data.rx.len } };
        case UART_RX_BUF_REQUEST: return rx_buf_request {};
        case UART_RX_BUF_RELEASED: return rx_buf_released { evt.data.rx_buf.buf };
        case UART_RX_DISABLED: return rx_disabled {};
        case UART_RX_STOPPED:
            return rx_stopped { evt.data.rx_stop.reason,
                                { { evt.data.rx.buf + evt.data.rx.offset, evt.data.rx.len } } };
    }
    k_panic();
    return async_event {};
}

} // namespace details

#endif // ifdef CONFIG_UART_ASYNC_API

/**
 * @brief Base class for UART operations.
 *
 * This class serves as a template base for UART handling, allowing derived
 * classes to define their specific UART implementations.
 */
template <typename Uart_T>
class uart_base
{
public:
    using native_type = const device*;
    using value_type  = uint8_t;

    /**
     * @brief Verify that the device is ready for use.
     *
     * @retval true If the device is ready for use.
     * @retval false If the device is not ready for use or if a NULL device pointer is passed as
     *         argument.
     */
    bool is_ready() noexcept
    {
        return device_is_ready(native_handle());
    }

    /**
     * @brief Read a character from the device for input.
     *
     * @param byte_out Pointer to character.
     *
     * @retval 0 If a character arrived.
     * @retval -1 If no character was available to read (i.e. the UART input buffer was empty).
     * @retval -ENOSYS If the operation is not implemented.
     * @retval -EBUSY If async reception was enabled using @ref uart_rx_enable
     */
    int read(value_type* byte_out) noexcept
    {
        return uart_poll_in(native_handle(), byte_out);
    }

    /**
     * @brief Write a character to the device for output.
     *
     * @param byte Character to send.
     */
    void write(value_type byte) noexcept
    {
        uart_poll_out(native_handle(), byte);
    }

    /**
     * @brief Write data pointed to by a std::span to device.
     *
     * @param data Span pointing to data to send.
     */
    int write(const std::span<const value_type> data) noexcept
    {
        for (auto byte : data)
        {
            write(byte);
        }
        return 0;
    }

    /**
     * @brief Write a std::string_view to device.
     *
     * @param str String to send.
     */
    int write(const std::string_view str) noexcept
    {
        for (auto ch : str)
        {
            write(ch);
        }
        return 0;
    }

    /**
     * @brief Retrieves a pointer to the native UART descriptor.
     *
     * @return A constant pointer to the native UART descriptor.
     */
    native_type native_handle() const noexcept
    {
        return static_cast<const Uart_T*>(this)->native_handle();
    }

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
public:
    using interrupt_callback_type = stdext::inplace_function<void()>;

    /**
     * @brief Set the IRQ callback function.
     *
     * @note This method calls `uart_irq_update` and `uart_irq_rx_ready` before calling the provided
     *       callback function.
     *
     * @param callback The callback function.
     *
     * @retval 0 On success.
     * @retval -ENOSYS If this function is not implemented.
     * @retval -ENOTSUP If API is not enabled.
     */
    int set_rx_interrupt_callback(interrupt_callback_type callback) noexcept
    {
        rx_interrupt_callback_ = callback;

        auto irq_callback = [](const device* dev, void* user_data)
        {
            auto* uart = static_cast<uart_base<Uart_T>*>(user_data);

            if (!uart_irq_update(dev))
            {
                return;
            }

            if (!uart_irq_rx_ready(dev))
            {
                return;
            }

            if (uart && uart->rx_interrupt_callback_)
            {
                uart->rx_interrupt_callback_();
            }
        };

        return uart_irq_callback_user_data_set(native_handle(), irq_callback, this);
    }

    /**
     * @brief Read data from FIFO to data section pointed to by `data_out`.
     *
     * @details This function is expected to be called from UART interrupt handler (ISR).
     *
     * @param data_out Span to read to.
     *
     * @return Number of bytes read.
     * @retval -ENOSYS If this function is not implemented.
     * @retval -ENOTSUP If API is not enabled.
     */
    int fifo_read(std::span<value_type> data_out) noexcept
    {
        return uart_fifo_read(native_handle(), &data_out.front(), data_out.size_bytes());
    }

    /**
     * @brief Read single byte from FIFO.
     *
     * @details This function is expected to be called from UART interrupt handler (ISR).
     *
     * @param byte_out Pointer to byte.
     *
     * @return Number of bytes read.
     * @retval -ENOSYS If this function is not implemented.
     * @retval -ENOTSUP If API is not enabled.
     */
    int fifo_read(value_type* byte_out) noexcept
    {
        return uart_fifo_read(native_handle(), byte_out, 1);
    }

    /**
     * @brief Enable RX interrupt.
     */
    void irq_rx_enable() noexcept
    {
        uart_irq_rx_enable(native_handle());
    }

    /**
     * @brief Disable RX interrupt.
     */
    void irq_rx_disable() noexcept
    {
        uart_irq_rx_disable(native_handle());
    }

private:
    interrupt_callback_type rx_interrupt_callback_;
#endif // ifdef CONFIG_UART_INTERRUPT_DRIVEN

#ifdef CONFIG_UART_ASYNC_API
public:
    using async_callback_type = stdext::inplace_function<void(async_event)>;

    /**
     * @brief Set event handler function.
     *
     * Since it is mandatory to set callback to use other asynchronous functions, it can be used to
     * detect if the device supports asynchronous API. Remaining API does not have that detection.
     *
     * @param async_callback Event handler.
     *
     * @retval 0 If successful.
     * @retval -ENOSYS If not supported by the device.
     * @retval -ENOTSUP If API not enabled.
     */
    int set_async_callback(async_callback_type async_callback) noexcept
    {
        async_event_callback_ = async_callback;

        auto async_cb = [](const device* dev, uart_event* evt, void* user_data)
        {
            auto* uart = static_cast<uart_base<Uart_T>*>(user_data);

            if (uart && uart->async_event_callback_)
            {
                uart->async_event_callback_(details::event_struct_to_variant(*evt));
            }
        };

        return uart_callback_set(native_handle(), async_cb, this);
    }

    /**
     * @brief Provide receive buffer in response to #UART_RX_BUF_REQUEST event.
     *
     * Provide span into RX buffer, which will be used when current buffer is filled.
     *
     * @note Providing buffer that is already in usage by driver leads to undefined behavior. Buffer
     * can be reused when it has been released by driver.
     *
     * @param span Span pointing to receive buffer.
     *
     * @retval 0 If successful.
     * @retval -ENOTSUP If API is not enabled.
     * @retval -EBUSY Next buffer already set.
     * @retval -EACCES Receiver is already disabled (function called too late?).
     * @retval -errno Other negative errno value in case of failure.
     */
    int rx_buf_rsp(std::span<value_type> span) noexcept
    {
        return uart_rx_buf_rsp(native_handle(), &span.front(), span.size_bytes());
    }

    /**
     * @brief Disable RX
     *
     * #rx_buf_released event will be generated for every buffer scheduled, after that #rx_disabled
     * event will be generated. Additionally, if there is any pending received data, the #rx_ready
     * event for that data will be generated before the #rx_buf_released events.
     *
     * @retval 0 If successful.
     * @retval -ENOTSUP If API is not enabled.
     * @retval -EFAULT There is no active reception.
     * @retval -errno Other negative errno value in case of failure.
     */
    int rx_disable() noexcept
    {
        return uart_rx_disable(native_handle());
    }

    /**
     * @brief Start receiving data through UART.
     *
     * Function sets given buffer as first buffer for receiving and returns immediately. After that
     * event handler, set using @ref uart::set_async_callback, is called with #rx_ready or
     * #rx_buf_request events.
     *
     * @param span Span pointing to receive buffer.
     * @param timeout Inactivity period after receiving at least a byte which triggers #rx_ready
     *                event. Given in microseconds. @ref SYS_FOREVER_US disables timeout. See @ref
     *                uart_event_type for details.
     *
     * @retval 0 If successful.
     * @retval -ENOTSUP If API is not enabled.
     * @retval -EBUSY RX already in progress.
     * @retval -errno Other negative errno value in case of failure.
     */
    int rx_enable(std::span<value_type> span, int32_t timeout) noexcept
    {
        return uart_rx_enable(native_handle(), &span.front(), span.size_bytes(), timeout);
    }

    /**
     * @brief Send given number of bytes from buffer through UART.
     *
     * Function returns immediately and event handler, set using @ref uart::set_async_callback, is
     * called after transfer is finished.
     *
     * @param span Span pointing to transmit buffer.
     * @param timeout Timeout in microseconds. Valid only if flow control is enabled. @ref
     *                SYS_FOREVER_US disables timeout.
     *
     * @retval 0 If successful.
     * @retval -ENOTSUP If API is not enabled.
     * @retval -EBUSY If There is already an ongoing transfer.
     * @retval -errno Other negative errno value in case of failure.
     */
    int tx(const std::span<const value_type> span, int32_t timeout) noexcept
    {
        return uart_tx(native_handle(), &span.front(), span.size_bytes(), timeout);
    }

    /**
     * @brief Send a string_view through UART.
     *
     * Function returns immediately and event handler, set using @ref uart::set_async_callback, is
     * called after transfer is finished.
     *
     * @param str string_view pointing to data to send.
     * @param timeout Timeout in microseconds. Valid only if flow control is enabled. @ref
     *                SYS_FOREVER_US disables timeout.
     *
     * @retval 0 If successful.
     * @retval -ENOTSUP If API is not enabled.
     * @retval -EBUSY If There is already an ongoing transfer.
     * @retval -errno Other negative errno value in case of failure.
     */
    int tx(const std::string_view str, int32_t timeout) noexcept
    {
        return uart_tx(native_handle(),
                       reinterpret_cast<const uint8_t*>(&str.front()),
                       str.length(),
                       timeout);
    }

    /**
     * @brief Abort current TX transmission.
     *
     * #tx_done event will be generated with amount of data sent.
     *
     * @retval 0 If successful.
     * @retval -ENOTSUP If API is not enabled.
     * @retval -EFAULT There is no active transmission.
     * @retval -errno Other negative errno value in case of failure.
     */
    int tx_abort() noexcept
    {
        return uart_tx_abort(native_handle());
    }

private:
    async_callback_type async_event_callback_;
#endif // ifdef CONFIG_UART_ASYNC_API
};

/**
 * @brief UART class that holds a pointer to a UART device.
 */
class uart : public uart_base<uart>
{
public:
    /**
     * @brief Constructs a UART object from a UART device pointer.
     *
     * @param uart_dev Pointer to UART device.
     */
    uart(const device* uart_dev) noexcept
        : uart_dev_ { uart_dev }
    {
    }

    /**
     * @brief Retrieves a pointer to the native UART descriptor.
     *
     * @return A constant pointer to the native UART descriptor.
     */
    native_type native_handle() const noexcept
    {
        return uart_dev_;
    }

private:
    const device* uart_dev_;

public:
    uart() = delete;
};

} // namespace lefse

#endif // ifdef CONFIG_SERIAL

#endif // ifndef LEFSE_UART_HPP_INCLUDED
