/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _STM32_SAI_I2S_H_
#define _STM32_SAI_I2S_H_
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/i2s.h>
#include "stm32_ll_sai.h"

struct queue_item {
	void *mem_block;
	size_t size;
};

/* Device configuration and parameters */
struct i2s_sai_stm32_cfg {
	SAI_TypeDef *i2s;
	const struct stm32_pclken *pclken;
	size_t pclk_len;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_config)(const struct device *dev);
	bool master_clk_sel;
};

struct stream {
	int32_t state;
	struct k_msgq *msgq;

	const struct device *dev_dma;
	uint32_t dma_channel;
	struct dma_config dma_cfg;
	uint8_t priority;
	bool src_addr_increment;
	bool dst_addr_increment;
	uint8_t fifo_threshold;
	bool tx_stop_for_drain;

	struct i2s_config cfg;
	void *mem_block;
	bool last_block;
	bool master;
	int (*stream_start)(struct stream *, const struct device *dev);
	void (*stream_disable)(struct stream *, const struct device *dev);
};

/* Device run time data */
struct i2s_sai_stm32_data {
	struct stream rx;
	struct stream tx;
};

/* checks that DMA Tx packet is fully transmitted over the I2S */
static inline uint32_t ll_func_i2s_dma_busy(SAI_TypeDef *i2s)
{
	return LL_sai_get_dma_status_ll_subsystem(i2s) && LL_sai_get_FREQ_ll_subsystem(i2s);
}

#endif /* _STM32_SAI_I2S_H_ */
