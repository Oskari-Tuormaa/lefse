#include <lefse/uart.hpp>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_polling);

lefse::uart uart { DEVICE_DT_GET(DT_NODELABEL(uart0)) };

int setup_uart()
{
    int err;

    if (!uart.is_ready())
    {
        LOG_ERR("Uart %s was not ready", uart.native_handle()->name);
        return -1;
    }

    err = uart.configure(115200);
    if (err < 0)
    {
        LOG_ERR("Failed configuring device %s [%d]", uart.native_handle()->name, err);
        return err;
    }

    return 0;
}

void run_uart()
{
    uint8_t ch;
    while (true)
    {
        while (uart.read(&ch) == 0)
        {
            uart.write(ch);
        }

        k_sleep(K_MSEC(1));
    }
}
