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

namespace async_events
{

struct tx_done
{
    const std::span<const uint8_t> span;
};

struct tx_aborted
{
    const std::span<const uint8_t> span;
};

struct rx_ready
{
    std::span<uint8_t> span;
};

struct rx_buf_request
{
};

struct rx_buf_released
{
    uint8_t* buf;
};

struct rx_disabled
{
};

struct rx_stopped
{
    uart_rx_stop_reason reason;
    rx_ready            data;
};

} // namespace async_events

using async_event = std::variant<async_events::rx_stopped,
                                 async_events::rx_disabled,
                                 async_events::rx_buf_released,
                                 async_events::rx_buf_request,
                                 async_events::rx_ready,
                                 async_events::tx_aborted,
                                 async_events::tx_done>;

namespace details
{

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
 * @brief Bas class for UART operations.
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

    int rx_buf_rsp(std::span<value_type> span) noexcept
    {
        return uart_rx_buf_rsp(native_handle(), &span.front(), span.size_bytes());
    }

    int rx_disable() noexcept
    {
        return uart_rx_disable(native_handle());
    }

    int rx_enable(std::span<value_type> span, int32_t timeout) noexcept
    {
        return uart_rx_enable(native_handle(), &span.front(), span.size_bytes(), timeout);
    }

    int tx(const std::span<const value_type> span, int32_t timeout) noexcept
    {
        return uart_tx(native_handle(), &span.front(), span.size_bytes(), timeout);
    }

    int tx(const std::string_view str, int32_t timeout) noexcept
    {
        return uart_tx(native_handle(),
                       reinterpret_cast<const uint8_t*>(&str.front()),
                       str.length(),
                       timeout);
    }

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
