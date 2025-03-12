#include <lefse/uart.hpp>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_interrupt);

lefse::uart uart { DEVICE_DT_GET(DT_NODELABEL(uart0)) };

#define CHECK_ERR(check, ...) \
    if (check)                \
    {                         \
        LOG_ERR(__VA_ARGS__); \
        return err;           \
    }

int setup_uart()
{
    int err;

    if (!uart.is_ready())
    {
        LOG_ERR("Device %s was not ready", uart.name());
        return -1;
    }

    err = uart.configure(115200);
    CHECK_ERR(err < 0, "Failed configuring device %s [%d]", uart.name(), err);

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
              uart.name(),
              err);

    uart.irq_rx_enable();

    return 0;
}

void run_uart()
{
}
