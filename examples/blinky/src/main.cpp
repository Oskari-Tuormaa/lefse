#include <lefse/gpio.hpp>
#include <lefse/timer.hpp>

constexpr auto timer_period = K_MSEC(1000);

lefse::gpio led { GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios) };

lefse::timer timer;

int main()
{
    int  ret;
    bool led_state = true;

    if (!led.is_ready())
    {
        return 0;
    }

    ret = led.configure(lefse::gpio_direction::output);
    if (ret < 0)
    {
        return ret;
    }

    auto expiry_callback = [&led_state]
    {
        led_state = !led_state;
        led.set(led_state);
        printf("LED state: %s\r\n", led_state ? "ON" : "OFF");
    };
    timer.set_expiry_callback(expiry_callback);
    timer.start(timer_period);
}
