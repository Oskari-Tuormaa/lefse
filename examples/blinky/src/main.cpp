#include <lefse/gpio.hpp>
#include <lefse/timer.hpp>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main);

lefse::gpio led_gpio { GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios) };

lefse::timer timer;

int main()
{
    int err;

    if (!led_gpio.is_ready())
    {
        LOG_ERR("Device %s not ready", led_gpio.native_handle()->port->name);
        return -1;
    }

    err = led_gpio.configure(lefse::gpio_direction::output);
    if (err < 0)
    {
        LOG_ERR("Error configuring %s [%d]", led_gpio.native_handle()->port->name, err);
        return err;
    }

    timer.set_expiry_callback(
        []
        {
            LOG_INF("Toggling!");
            led_gpio.toggle();
        });
    timer.start(K_MSEC(1000));
}
