#include <lefse/uart.hpp>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_interrupt);

lefse::uart uart { DEVICE_DT_GET(DT_NODELABEL(uart0)) };

int setup_uart()
{
    int err;

    err = uart.set_rx_interrupt_callback(
        []
        {
            uint8_t ch;
            while (uart.fifo_read(&ch) != 0)
            {
                uart.write(ch);
            }
        });

    if (err < 0)
    {
        LOG_ERR("Failed setting rx interrupt callback on device %s [%d]",
                uart.native_handle()->name,
                err);
        return err;
    }

    uart.irq_rx_enable();

    return 0;
}

void run_uart()
{
}
