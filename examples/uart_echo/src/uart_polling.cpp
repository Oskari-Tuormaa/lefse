#include <lefse/uart.hpp>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_polling);

lefse::uart uart { DEVICE_DT_GET(DT_NODELABEL(uart0)) };

int setup_uart()
{
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
