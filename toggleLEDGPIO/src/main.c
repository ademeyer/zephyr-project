#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const uint32_t sleep_time_ms = 100;

int main()
{
	while(!gpio_is_ready_dt(&led));

	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

	if (ret < 0)
		return 1;

	while(1)
	{
		gpio_pin_toggle_dt(&led);
		k_msleep(sleep_time_ms);
	}
	return 0;
}
