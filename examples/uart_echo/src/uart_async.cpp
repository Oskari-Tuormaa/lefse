#include <array>
#include <lefse/uart.hpp>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_async);

lefse::uart             uart { DEVICE_DT_GET(DT_NODELABEL(uart0)) };
std::array<uint8_t, 64> buf;

template <typename... Ts>
struct overload : Ts...
{
    using Ts::operator()...;
};

#define CHECK_ERR(check, ...) \
    if (check)                \
    {                         \
        LOG_ERR(__VA_ARGS__); \
        return err;           \
    }

int setup_uart()
{
    int err;

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

    return 0;
}

void run_uart()
{
}
