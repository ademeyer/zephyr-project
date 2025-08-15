#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/audio/codec.h>
#include <string.h>
#include "i2s_sai_ll_stm32.h"
#include "sine.h"

// LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define I2S_CODEC_TX DT_ALIAS(i2s_codec_tx)

// Audio stream information
#define SAMPLE_FREQUENCY (44100U)
#define SAMPLE_RATE (16000U)    // 16kHz
#define SAMPLE_BIT_WIDTH (16U)  // 16-bit buffer
#define NUMBER_OF_CHANNELS (2U) // no. of channel
#define BLOCK_SIZE_MS (100U)    // 100ms blocks
#define BYTES_PER_SAMPLE sizeof(int16_t)
#define SAMPLE_PER_BLOCK ((SAMPLE_RATE * BLOCK_SIZE_MS) / 1000)
#define INITIAL_BLOCKS (4U)
#define TIMEOUT (2000U)

#define BLOCK_SIZE (BYTES_PER_SAMPLE * SAMPLE_PER_BLOCK)
#define BLOCK_COUNT (INITIAL_BLOCKS + 32)
K_MEM_SLAB_DEFINE_STATIC(mem_slab, BLOCK_SIZE, BLOCK_COUNT, 4);

/* Configure transmit stream */
static bool configure_tx_streams(const struct device *i2s_dev, struct i2s_config *config)
{
  int ret;
  ret = i2s_configure(i2s_dev, I2S_DIR_TX, config);
  if (ret < 0)
  {
    printk("Failed to configure codec stream: %d\n", ret);
    return false;
  }
  return true;
}

static bool trigger_command(const struct device *i2s_dev_codec, enum i2s_trigger_cmd cmd)
{
  int ret;

  ret = i2s_trigger(i2s_dev_codec, I2S_DIR_TX, cmd);
  if (ret < 0)
  {
    printk("Failed to trigger command %d on TX: %d\n", cmd, ret);
    return false;
  }

  return true;
}

int main(void)
{
  const struct device *const i2s_dev_codec = DEVICE_DT_GET(I2S_CODEC_TX);
  const struct device *const codec_dev = DEVICE_DT_GET(DT_NODELABEL(audio_codec));
  struct i2s_config config;
  struct audio_codec_cfg audio_cfg;
  int ret = 0;

  printk("codec sample\n");

  if (!device_is_ready(i2s_dev_codec))
  {
    printk("%s is not ready\n", i2s_dev_codec->name);
    return 0;
  }

  if (!device_is_ready(codec_dev))
  {
    printk("%s is not ready", codec_dev->name);
    return 0;
  }

  audio_cfg.dai_route = AUDIO_ROUTE_PLAYBACK;
  audio_cfg.dai_type = AUDIO_DAI_TYPE_I2S;
  audio_cfg.dai_cfg.i2s.word_size = SAMPLE_BIT_WIDTH;
  audio_cfg.dai_cfg.i2s.channels = NUMBER_OF_CHANNELS;
  audio_cfg.dai_cfg.i2s.format = I2S_FMT_DATA_FORMAT_I2S;
  audio_cfg.dai_cfg.i2s.options = I2S_OPT_FRAME_CLK_MASTER;
  audio_cfg.dai_cfg.i2s.frame_clk_freq = SAMPLE_RATE;
  audio_cfg.dai_cfg.i2s.mem_slab = &mem_slab;
  audio_cfg.dai_cfg.i2s.block_size = BLOCK_SIZE;
  ret = audio_codec_configure(codec_dev, &audio_cfg);
  if (ret < 0)
  {
    printk("Failed to configure audio codec: %d\n", ret);
    return 1;
  }
  k_msleep(1000);

  config.word_size = SAMPLE_BIT_WIDTH;
  config.channels = NUMBER_OF_CHANNELS;
  config.format = I2S_FMT_DATA_FORMAT_I2S;
  config.options = I2S_OPT_BIT_CLK_MASTER | I2S_OPT_FRAME_CLK_MASTER;
  config.frame_clk_freq = SAMPLE_RATE;
  config.mem_slab = &mem_slab;
  config.block_size = BLOCK_SIZE;
  config.timeout = TIMEOUT;
  if (!configure_tx_streams(i2s_dev_codec, &config))
  {
    printk("failure to config streams\n");
    return 1;
  }

  printk("start streams\n");
  for (;;)
  {
    bool started = false;
    while (1)
    {
      printk("in streams loop\n");
      void *mem_block;
      uint32_t block_size = BLOCK_SIZE;
      int ret;
      int i;

      for (i = 0; i < 2; i++)
      {
        printk("stream %d\n", i);
        /* Play a sine wave 440Hz */
        mem_block = (void *)&__16kHz16bit_stereo_sine_pcm;

        ret = i2s_write(i2s_dev_codec, mem_block, block_size);
        if (ret != 0)
        {
          printk("Failed to write data: %d\n", ret);
          break;
        }
      }
      if (ret < 0)
      {
        printk("error %d\n", ret);
        break;
      }
      if (!started)
      {
        i2s_trigger(i2s_dev_codec, I2S_DIR_TX, I2S_TRIGGER_START);
        started = true;
        printk("trigger started\n");
      }
    }

    if (!trigger_command(i2s_dev_codec, I2S_TRIGGER_DROP))
    {
      printk("Send I2S trigger DRAIN failed: %d", ret);
      return 0;
    }
    printk("Streams stopped\n");
    return 0;
  }
  return 0;
}
