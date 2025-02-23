#include <lefse/uart.hpp>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main);

lefse::uart uart { DEVICE_DT_GET(DT_NODELABEL(uart0)) };

#if defined(CONFIG_UART_ASYNC_API)
#include <array>
std::array<uint8_t, 64> buf;

template <typename... Ts>
struct overload : Ts...
{
    using Ts::operator()...;
};
#endif

#define CHECK_ERR(check, ...) \
    if (check)                \
    {                         \
        LOG_ERR(__VA_ARGS__); \
        return err;           \
    }

int main()
{
    int err;

    if (!uart.is_ready())
    {
        LOG_ERR("Device %s not ready", uart.native_handle()->name);
        return -1;
    }

#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
    err = uart.set_rx_interrupt_callback(
        []
        {
            uint8_t ch;
            while (uart.fifo_read(&ch) != 0)
            {
                uart.write(ch);
            }
        });
    CHECK_ERR(err < 0,
              "Failed setting rx interrupt callback on device %s [%d]",
              uart.native_handle()->name,
              err);
    uart.irq_rx_enable();

#elif defined(CONFIG_UART_ASYNC_API)
    err = uart.set_async_callback(
        [](auto evt)
        {
            using namespace lefse::async_events;
            auto evt_handler = overload { [](rx_buf_request)
                                          {
                                              uart.rx_buf_rsp(buf);
                                          },
                                          [](rx_ready evt)
                                          {
                                              uart.write(evt.span);
                                          },
                                          [](auto) {} };
            std::visit(evt_handler, evt);
        });
    CHECK_ERR(err < 0,
              "Failed setting rx interrupt callback on device %s [%d]",
              uart.native_handle()->name,
              err);

    err = uart.rx_enable(buf, USEC_PER_MSEC);
    CHECK_ERR(err < 0,
              "Failed enabling receiving data on device %s [%d]",
              uart.native_handle()->name,
              err);

#else
    uint8_t ch;
    while (true)
    {
        while (uart.read(&ch) == 0)
        {
            uart.write(ch);
        }

        k_sleep(K_MSEC(1));
    }

#endif
}
