#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/audio/dmic.h>
#include <zephyr/logging/log.h>
#include "stm32_ll_sai.h"

LOG_MODULE_REGISTER(audio_capture, LOG_LEVEL_DGB);

// Audio stream information
#define SAMPLE_RATE      16000 // 16kHz
#define SAMPLE_BITS      16    // 16-bit buffer
#define CHANNELS         1     // no. of channel
#define BLOCK_SIZE_MS    100   // 100ms blocks
#define SAMPLE_PER_BLOCK ((SAMPLE_RATE * BLOCK_SIZE_MS) / 1000)

// audio buffer
static int16_t audio_buffer[SAMPLE_PER_BLOCK * CHANNELS];

int main(void)
{
	// const struct  device *const mic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic0));

	return 0;
}
