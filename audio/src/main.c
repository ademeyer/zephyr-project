#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/audio/codec.h>
#include <string.h>
#include "i2s_sai_ll_stm32.h"

#define I2S_CODEC_TX DT_ALIAS(i2s_codec_tx)

// Audio stream information
#define SAMPLE_RATE 16000       // 16kHz
#define SAMPLE_BITS (16U)       // 16-bit buffer
#define NUMBER_OF_CHANNELS (2U) // no. of channel
#define BLOCK_SIZE_MS 100       // 100ms blocks
#define BYTES_PER_SAMPLE sizeof(int16_t)
#define SAMPLE_PER_BLOCK ((SAMPLE_RATE * BLOCK_SIZE_MS) / 1000)

// audio buffer
static int16_t audio_buffer[SAMPLE_PER_BLOCK * NUMBER_OF_CHANNELS];

int main(void)
{
  const struct device *const i2s_dev_codec = DEVICE_DT_GET(I2S_CODEC_TX);
  const struct device *const codec_dev = DEVICE_DT_GET(DT_NODELABEL(audio_codec));
  return 0;
}
