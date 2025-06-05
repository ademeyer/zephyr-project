#include <zephyr/random/random.h>
#if CONFIG_SAY_HELLO
#include "say_hello.h"
#endif

static const int32_t sleep_time_ms = 2000;

int main()
{
	printk("random generator test\n");
	uint32_t rnd;
	double rnd_float;

	while(1)
	{
		rnd = sys_rand32_get();
		rnd_float = (double) rnd / (UINT32_MAX + 1.0);
		printk("Random value: %.3f\n", rnd_float);

#if CONFIG_SAY_HELLO
		say_hello();
#endif

		k_msleep(sleep_time_ms);
	}
	return 0;
}
