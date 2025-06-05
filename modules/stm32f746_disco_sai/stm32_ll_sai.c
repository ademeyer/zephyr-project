
#include "stm32_ll_sai.h"

/* Private define ------------------------------------------------------------*/

/** @defgroup SAI_Private_Constants  SAI Private Constants
 * @{
 */
#define SAI_DEFAULT_TIMEOUT 4U /* 4ms */
#define SAI_LONG_TIMEOUT    1000U
/**
 * @}
 */

/**
 * @}
 */

/** @defgroup SAI_Private_Functions  SAI Private Functions
 * @{
 */
int LL_sai_init(sai_cfg_t *sai_cfg);
static int LL_sai_init_i2s(sai_cfg_t *sai_cfg, uint32_t protocol, uint32_t datasize,
			   uint32_t nbslot);
static int LL_sai_init_pcm(sai_cfg_t *sai_cfg, uint32_t protocol, uint32_t datasize,
			   uint32_t nbslot);
static SAI_Block_TypeDef *LL_get_sai_block(SAI_TypeDef *SAIx, uint32_t block);
static void LL_sai_clear_all_irq(SAI_Block_TypeDef *sai_x);
static uint32_t LL_get_sai_interrupt_flag(SAI_Block_TypeDef *sai, uint32_t protocol,
					  uint32_t audiomode);
static void LL_sai_flush_fifo(SAI_Block_TypeDef *sai);

/**
 * @}
 */

/* Exported functions ---------------------------------------------------------*/

/** @defgroup SAI_Exported_Functions SAI Exported Functions
 * @{
 */

/** @defgroup SAI_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
	    ##### Initialization and de-initialization functions #####
 ===============================================================================
  [..]  This subsection provides a set of functions allowing to initialize and
	de-initialize the SAIx peripheral:

      (+) Call the function sai_init() to configure the selected device with
	  the selected configuration:
	(++) Mode (Master/slave TX/RX)
	(++) Protocol
	(++) Data Size
	(++) MCLK Output
	(++) Audio frequency
	(++) FIFO Threshold
	(++) Frame Config
	(++) Slot Config

@endverbatim
  * @{
  */

/**
 * @brief  Initialize the structure FrameInit, slot_cfg and the low part of
 *         Init according to the specified parameters and call the function
 *         sai_init to initialize the SAI block.
 * @param  sai_cfg pointer to a sai_cfg structure that contains
 *               the configuration information for SAI module.
 * @param  protocol one of the supported protocol @ref SAI_Protocol
 * @param  datasize one of the supported datasize @ref SAI_Protocol_DataSize
 *                   the configuration information for SAI module.
 * @param  nbslot Number of slot.
 * @retval 0 = success, else failed
 */
int LL_sai_init_ll_subsystem(SAI_TypeDef *sai, uint32_t protocol, uint32_t datasize,
			     uint32_t audiomode, uint32_t audiofrequency, uint32_t nbslot,
			     uint32_t clk_strobs, uint32_t pclk)
{
	int valid = IS_SAI_SUPPORTED_PROTOCOL(protocol) && IS_SAI_PROTOCOL_DATASIZE(datasize) &&
		    IS_SAI_AUDIO_FREQUENCY(audiofrequency) && IS_SAI_BLOCK_MODE(audiomode) &&
		    IS_SAI_BLOCK_CLOCK_STROBING(clk_strobs);

	if (!valid || !sai) {
		return 1;
	}

	sai_cfg_t *sai_cfg = &(sai_cfg_t){.Instance = LL_get_sai_block(sai, SAIx_BLOCK_A),
					  .init.a_freq = audiofrequency,
					  .init.audio_mode = audiomode,
					  .init.clk_strobing = clk_strobs,
					  .init.sai_clock = pclk,
					  .init.b_sync = SAI_ASYNCHRONOUS,
					  .init.synchro_ext = SAI_SYNCEXT_DISABLE,
					  .init.stereo_mode = SAI_STEREOMODE,
					  .init.companding_mode = SAI_NOCOMPANDING,
					  .init.output_drv = SAI_OUTPUTDRIVE_DISABLE,
					  .init.fifo_threshold = SAI_FIFOSTATUS_EMPTY,
					  .init.first_bit = SAI_FIRSTBIT_MSB};

	int status = 0;

	switch (protocol) {
	case SAI_I2S_STANDARD:
	case SAI_I2S_MSBJUSTIFIED:
	case SAI_I2S_LSBJUSTIFIED:
		status = LL_sai_init_i2s(sai_cfg, protocol, datasize, nbslot);
		break;
	case SAI_PCM_LONG:
	case SAI_PCM_SHORT:
		status = LL_sai_init_pcm(sai_cfg, protocol, datasize, nbslot);
		break;
	default:
		status = 1;
		break;
	}

	if (status == 0) {
		LL_sai_disable_ll_subsystem(sai);
		status = LL_sai_init(sai_cfg);
	}

	return status;
}

/**
 * @brief  Initialize the SAI according to the specified parameters.
 *         in the sai_init_cfg structure and initialize the associated handle.
 * @param  sai_cfg pointer to a sai_cfg structure that contains
 *                the configuration information for SAI module.
 * @retval status (0 = success, else failed)
 */
int LL_sai_init(sai_cfg_t *sai_cfg)
{
	uint32_t tmpregisterGCR = 0;
	uint32_t ckstr_bits = 0;
	uint32_t syncen_bits = 0;

	/* Check the SAI handle allocation */
	if (!sai_cfg) {
		return 1;
	}

	/* SAI Block Synchro Configuration -----------------------------------------*/
	/* This setting must be done with both audio block (A & B) disabled         */
	switch (sai_cfg->init.synchro_ext) {
	case SAI_SYNCEXT_DISABLE:
		tmpregisterGCR = 0;
		break;
	case SAI_SYNCEXT_OUTBLOCKA_ENABLE:
		tmpregisterGCR = SAI_GCR_SYNCOUT_0;
		break;
	case SAI_SYNCEXT_OUTBLOCKB_ENABLE:
		tmpregisterGCR = SAI_GCR_SYNCOUT_1;
		break;
	default:
		break;
	}

	switch (sai_cfg->init.b_sync) {
	case SAI_ASYNCHRONOUS:
		syncen_bits = 0;
		break;
	case SAI_SYNCHRONOUS:
		syncen_bits = SAI_xCR1_SYNCEN_0;
		break;
	case SAI_SYNCHRONOUS_EXT_SAI1:
		syncen_bits = SAI_xCR1_SYNCEN_1;
		break;
	case SAI_SYNCHRONOUS_EXT_SAI2:
		syncen_bits = SAI_xCR1_SYNCEN_1;
		tmpregisterGCR |= SAI_GCR_SYNCIN_0;
		break;

	default:
		break;
	}

	if ((sai_cfg->Instance == SAI1_Block_A) || (sai_cfg->Instance == SAI1_Block_B)) {
		SAI1->GCR = tmpregisterGCR;
	} else {

		SAI2->GCR = tmpregisterGCR;
	}

	if (sai_cfg->init.a_freq != SAI_AUDIO_FREQUENCY_MCKDIV) {
		uint32_t freq = sai_cfg->init.sai_clock;
		uint32_t tmpval;

		/* Configure Master Clock using the following formula :
		   MCLK_x = SAI_CK_x / (MCKDIV[3:0] * 2) with MCLK_x = 256 * FS
		   FS = SAI_CK_x / (MCKDIV[3:0] * 2) * 256
		   MCKDIV[3:0] = SAI_CK_x / FS * 512 */
		/* (freq x 10) to keep Significant digits */
		tmpval = (freq * 10) / (sai_cfg->init.a_freq * 2 * 256);
		sai_cfg->init.mclk_div = tmpval / 10;

		/* Round result to the nearest integer */
		if ((tmpval % 10) > 8) {
			sai_cfg->init.mclk_div += 1;
		}

		/* For SPDIF protocol, SAI shall provide a bit clock twice faster the symbol-rate */
		if (sai_cfg->init.protocol == SAI_SPDIF_PROTOCOL) {
			sai_cfg->init.mclk_div = sai_cfg->init.mclk_div >> 1;
		}
	}

	/* Compute CKSTR bits of SAI CR1 according ClockStrobing and AudioMode */
	if ((sai_cfg->init.audio_mode == SAI_MODEMASTER_TX) ||
	    (sai_cfg->init.audio_mode == SAI_MODESLAVE_TX)) {
		/* Transmit */
		ckstr_bits = (sai_cfg->init.clk_strobing == SAI_CLOCKSTROBING_RISINGEDGE)
				     ? 0
				     : SAI_xCR1_CKSTR;
	} else {
		/* Receive */
		ckstr_bits = (sai_cfg->init.clk_strobing == SAI_CLOCKSTROBING_RISINGEDGE)
				     ? SAI_xCR1_CKSTR
				     : 0;
	}

	/* SAI Block Configuration -------------------------------------------------*/
	/* SAI CR1 Configuration */
	sai_cfg->Instance->CR1 &=
		~(SAI_xCR1_MODE | SAI_xCR1_PRTCFG | SAI_xCR1_DS | SAI_xCR1_LSBFIRST |
		  SAI_xCR1_CKSTR | SAI_xCR1_SYNCEN | SAI_xCR1_MONO | SAI_xCR1_OUTDRIV |
		  SAI_xCR1_DMAEN | SAI_xCR1_NODIV | SAI_xCR1_MCKDIV);

	sai_cfg->Instance->CR1 |=
		(sai_cfg->init.audio_mode | sai_cfg->init.protocol | sai_cfg->init.data_size |
		 sai_cfg->init.first_bit | ckstr_bits | syncen_bits | sai_cfg->init.stereo_mode |
		 sai_cfg->init.output_drv | sai_cfg->init.no_divider |
		 (sai_cfg->init.mclk_div << 20));

	/* SAI CR2 Configuration */
	sai_cfg->Instance->CR2 &= ~(SAI_xCR2_FTH | SAI_xCR2_FFLUSH | SAI_xCR2_COMP | SAI_xCR2_CPL);
	sai_cfg->Instance->CR2 |= (sai_cfg->init.fifo_threshold | sai_cfg->init.companding_mode |
				   sai_cfg->init.tri_state);

	/* SAI Frame Configuration -----------------------------------------*/
	sai_cfg->Instance->FRCR &= (~(SAI_xFRCR_FRL | SAI_xFRCR_FSALL | SAI_xFRCR_FSDEF |
				      SAI_xFRCR_FSPOL | SAI_xFRCR_FSOFF));
	sai_cfg->Instance->FRCR |=
		((sai_cfg->frame_init.frame_length - 1) | sai_cfg->frame_init.fs_offset |
		 sai_cfg->frame_init.fs_definition | sai_cfg->frame_init.fs_polarity |
		 ((sai_cfg->frame_init.active_frame_length - 1) << 8));

	/* SAI Block_x SLOT Configuration ------------------------------------------*/
	/* This register has no meaning in AC 97 and SPDIF audio protocol */
	sai_cfg->Instance->SLOTR &=
		(~(SAI_xSLOTR_FBOFF | SAI_xSLOTR_SLOTSZ | SAI_xSLOTR_NBSLOT | SAI_xSLOTR_SLOTEN));

	sai_cfg->Instance->SLOTR |=
		sai_cfg->slot_init.first_bit_offset | sai_cfg->slot_init.slot_size |
		(sai_cfg->slot_init.slot_active << 16) | ((sai_cfg->slot_init.n_slot - 1) << 8);

	return 0;
}

/**
 * @brief  DeInitialize the SAI peripheral.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval status (0 = success, else failed)
 */
int LL_sai_deinit_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	/* Check the SAI handle allocation */
	if (!sai) {
		return 1;
	}

	/* Disabled All interrupt and clear all the flag */
	LL_sai_clear_all_irq(sai);

	/* Disable the SAI */
	int status = LL_sai_disable_ll_subsystem(sai_x);

	/* Flush the fifo */
	LL_sai_flush_fifo(sai);

	return status;
}
/**
 * @}
 */

/** @defgroup SAI_Exported_Functions_Group2 IO operation functions
  * @brief    Data transfers functions
  *
@verbatim
  ==============================================================================
		      ##### IO operation functions #####
  ==============================================================================
  [..]
    This subsection provides a set of functions allowing to manage the SAI data
    transfers.

    (+) There are two modes of transfer:
      (++) Blocking mode : The communication is performed in the polling mode.
	   The status of all data processing is returned by the same function
	   after finishing transfer.
      (++) No-Blocking mode : The communication is performed using Interrupts
	   or DMA. These functions return the status of the transfer startup.
	   The end of the data processing will be indicated through the
	   dedicated SAI IRQ when using Interrupt mode or the DMA IRQ when
	   using DMA mode.

    (+) Blocking mode functions are :
      (++) HAL_SAI_Transmit()
      (++) HAL_SAI_Receive()

@endverbatim
  * @{
  */

/**
 * @brief Abort the current transfer and disable the SAI.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval status
 */
int LL_sai_dma_abort_ll_subsystem(SAI_TypeDef *sai_x)
{
	int status = 0;

	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);

	/* Check SAI DMA is enabled or not */
	if ((sai->CR1 & SAI_xCR1_DMAEN) == SAI_xCR1_DMAEN) {
		/* Disable the SAI DMA request */
		sai->CR1 &= ~SAI_xCR1_DMAEN;
	}

	/* Flush the fifo */
	LL_sai_flush_fifo(sai);

	return status;
}

/**
 * @brief Resume DMA request on SAI.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_dma_resume_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	sai->CR1 |= SAI_xCR1_DMAEN;
}

/**
 * @brief  Enable the Tx mute mode.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @param  val  value sent during the mute @ref SAI_Block_Mute_Value
 * @retval status (0 = success, 1 = failed)
 */
int LL_sai_enable_tx_mute_mode_ll_subsystem(SAI_TypeDef *sai_x, uint16_t val)
{
	if (!IS_SAI_BLOCK_MUTE_VALUE(val)) {
		return 1;
	}

	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);

	CLEAR_BIT(sai->CR2, SAI_xCR2_MUTEVAL | SAI_xCR2_MUTE);
	SET_BIT(sai->CR2, SAI_xCR2_MUTE | val);
	return 0;
}

/**
 * @brief  Disable the Tx mute mode.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_disable_tx_mute_mode_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	CLEAR_BIT(sai->CR2, SAI_xCR2_MUTEVAL | SAI_xCR2_MUTE);
}

/**
 * @brief  SAI Fifo request occurred.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval status (1 = occurred, 0 = no request)
 */
int LL_sai_get_FREQ_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	return (((sai->DR & SAI_xSR_FREQ) == SAI_xSR_FREQ) &&
		((sai->IMR & SAI_IT_FREQ) == SAI_IT_FREQ));
}

/**
 * @brief  SAI Disable the Overrun underrun interrupt.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_disable_OVRUDR_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_DISABLE_IT(sai, SAI_IT_OVRUDR);
}

/**
 * @brief  SAI Disable the Mute detection interrupt.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_disable_MUTEDET_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_DISABLE_IT(sai, SAI_IT_MUTEDET);
}

/**
 * @brief  SAI Disable the Wrong Clock Configuration interrupt.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_disable_WCKCFG_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_DISABLE_IT(sai, SAI_IT_WCKCFG);
}

/**
 * @brief  SAI Disable FIFO request interrupt.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_disable_FREQ_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_DISABLE_IT(sai, SAI_IT_FREQ);
}

/**
 * @brief  SAI Disable Codec not ready interrupt.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_disable_CNRDY_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_DISABLE_IT(sai, SAI_IT_CNRDY);
}

/**
 * @brief  SAI Disable Anticipated frame synchronization detection interrupt.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_disable_AFSDET_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_DISABLE_IT(sai, SAI_IT_AFSDET);
}

/**
 * @brief  SAI Disable Late frame synchronization detection interrupt.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_disable_LFSDET_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_DISABLE_IT(sai, SAI_IT_LFSDET);
}

/**
 * @brief  SAI Enable the Overrun underrun interrupt.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_enable_OVRUDR_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_ENABLE_IT(sai, SAI_IT_OVRUDR);
}

/**
 * @brief  SAI Enable the Mute detection interrupt.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_enable_MUTEDET_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_ENABLE_IT(sai, SAI_IT_MUTEDET);
}

/**
 * @brief  SAI Enable the Wrong Clock Configuration interrupt.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_enable_WCKCFG_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_ENABLE_IT(sai, SAI_IT_WCKCFG);
}

/**
 * @brief  SAI Enable FIFO request interrupt.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_enable_FREQ_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_ENABLE_IT(sai, SAI_IT_FREQ);
}

/**
 * @brief  SAI Enable Codec not ready interrupt.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_enable_CNRDY_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_ENABLE_IT(sai, SAI_IT_CNRDY);
}

/**
 * @brief  SAI Enable Anticipated frame synchronization detection interrupt.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_enable_AFSDET_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_ENABLE_IT(sai, SAI_IT_AFSDET);
}

/**
 * @brief  SAI Enable Late frame synchronization detection interrupt.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_enable_LFSDET_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_ENABLE_IT(sai, SAI_IT_LFSDET);
}

/* Get DMA SAI IO Operation */
/**
 * @brief  Get SAI DMA status.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval status (1 = busy, 0 = not busy)
 */
int LL_sai_get_dma_status_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	return ((sai->CR1 & SAI_xCR1_SAIEN) != RESET);
}

/**
 * @brief  Change Tranfer mode.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval status (0 = success, 1 = failed)
 */
int LL_sai_set_transfer_mode_ll_subsystem(SAI_TypeDef *sai_x, uint32_t mode)
{
	if (!IS_SAI_BLOCK_MODE(mode)) {
		return 1;
	}

	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);

	SET_BIT(sai->CR1, mode);
	return 0;
}
/**
 * @brief  Get SAI data register.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval pointer to __IO uint32_t
 */
__IO uint32_t *LL_sai_get_register_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	return &sai->DR;
}

/************** SAI peripheral controls **************************/
/**
 * @brief  Disable the SAI and wait for the disabling.
 * @param  sai  pointer to a SAI_Block_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval status (0 = success, 1 = failed)
 */
int LL_sai_disable_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	uint32_t count = SAI_DEFAULT_TIMEOUT * WAIT_DELAY;
	int status = 0;

	/* Disable the SAI instance */
	__SAI_DISABLE(sai);

	do {
		/* Check for the Timeout */
		if (count-- == 0) {
			status = 1;
			break;
		}
	} while ((sai->CR1 & SAI_xCR1_SAIEN) != RESET);

	return status;
}

/*********************** Get Interrupt Event *******************************/

/**
 * @brief  Overrun error occurred.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval status (1 = occurred, 0 = no request)
 */
int LL_sai_get_OVRUDR_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	return (((sai->DR & SAI_FLAG_OVRUDR) == SAI_FLAG_OVRUDR) &&
		((sai->IMR & SAI_IT_OVRUDR) == SAI_IT_OVRUDR));
}

/**
 * @brief  Anticipated frame detected.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval status (1 = occurred, 0 = no request)
 */
int LL_sai_get_AFSDET_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	return (((sai->DR & SAI_FLAG_AFSDET) == SAI_FLAG_AFSDET) &&
		((sai->IMR & SAI_IT_AFSDET) == SAI_IT_AFSDET));
}

/**
 * @brief  Late frame detected.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval status (1 = occurred, 0 = no request)
 */
int LL_sai_get_LFSDET_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	return (((sai->DR & SAI_FLAG_LFSDET) == SAI_FLAG_LFSDET) &&
		((sai->IMR & SAI_IT_LFSDET) == SAI_IT_LFSDET));
}

/**
 * @brief  Codec not ready detected (only in AC'97 mode).
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval status (1 = occurred, 0 = no request)
 */
int LL_sai_get_CNRDY_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	return (((sai->DR & SAI_FLAG_CNRDY) == SAI_FLAG_CNRDY) &&
		((sai->IMR & SAI_IT_CNRDY) == SAI_IT_CNRDY));
}

/**
 * @brief  Incorrect frame length configuration detected.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval status (1 = occurred, 0 = no request)
 */
int LL_sai_get_WCKCFG_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	return (((sai->DR & SAI_FLAG_WCKCFG) == SAI_FLAG_WCKCFG) &&
		((sai->IMR & SAI_IT_WCKCFG) == SAI_IT_WCKCFG));
}

/**
 * @brief  Mute frame detected.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval status (1 = occurred, 0 = no request)
 */
int LL_sai_get_MUTEDET_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	return (((sai->DR & SAI_FLAG_MUTEDET) == SAI_FLAG_MUTEDET) &&
		((sai->IMR & SAI_IT_MUTEDET) == SAI_IT_MUTEDET));
}

/* Clear Interrupt Event */
/**
 * @brief  Clear overrun under irq flag.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_clear_OVRUDR_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_CLEAR_FLAG(sai, SAI_FLAG_OVRUDR);
}

/**
 * @brief  Clear anticipated irq flag.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_clear_AFSDET_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_CLEAR_FLAG(sai, SAI_FLAG_AFSDET);
}

/**
 * @brief  Clear late frame irq flag.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_clear_LFSDET_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_CLEAR_FLAG(sai, SAI_FLAG_LFSDET);
}

/**
 * @brief  Clear codec not ready irq flag (only in AC'97 mode).
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_clear_CNRDY_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_CLEAR_FLAG(sai, SAI_FLAG_CNRDY);
}

/**
 * @brief  Clear incorrect frame length configuration irq flag.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_clear_WCKCFG_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_CLEAR_FLAG(sai, SAI_FLAG_WCKCFG);
}

/**
 * @brief  Clear mute frame irq flag.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
void LL_sai_clear_MUTEDET_ll_subsystem(SAI_TypeDef *sai_x)
{
	SAI_Block_TypeDef *sai = LL_get_sai_block(sai_x, SAIx_BLOCK_A);
	__SAI_CLEAR_FLAG(sai, SAI_FLAG_MUTEDET);
}

/**
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * @addtogroup SAI_Private_Functions
 *  @brief      Private functions
 * @{
 */

/**
 * @brief  Initialize the SAI I2S protocol according to the specified parameters
 *         in the SAI_InitTypeDef and create the associated handle.
 * @param  sai_cfg pointer to a sai_cfg_t structure that contains
 *                the configuration information for SAI module.
 * @param  protocol one of the supported protocol.
 * @param  datasize one of the supported datasize @ref SAI_Protocol_DataSize
 *                the configuration information for SAI module.
 * @param  nbslot number of slot minimum value is 2 and max is 16.
 *                    the value must be a multiple of 2.
 * @retval status
 */
static int LL_sai_init_i2s(sai_cfg_t *sai_cfg, uint32_t protocol, uint32_t datasize,
			   uint32_t nbslot)
{
	sai_cfg->init.protocol = SAI_FREE_PROTOCOL;
	sai_cfg->init.first_bit = SAI_FIRSTBIT_MSB;
	/* Compute ClockStrobing according AudioMode */
	if ((sai_cfg->init.audio_mode == SAI_MODEMASTER_TX) ||
	    (sai_cfg->init.audio_mode == SAI_MODESLAVE_TX)) {
		/* Transmit */
		sai_cfg->init.clk_strobing = SAI_CLOCKSTROBING_FALLINGEDGE;
	} else {
		/* Receive */
		sai_cfg->init.clk_strobing = SAI_CLOCKSTROBING_RISINGEDGE;
	}
	sai_cfg->frame_init.fs_definition = SAI_FS_CHANNEL_IDENTIFICATION;
	sai_cfg->slot_init.slot_active = SAI_SLOTACTIVE_ALL;
	sai_cfg->slot_init.first_bit_offset = 0;
	sai_cfg->slot_init.n_slot = nbslot;

	/* in IS2 the number of slot must be even */
	if ((nbslot & 0x1) != 0) {
		return 1;
	}
	if (protocol == SAI_I2S_STANDARD) {
		sai_cfg->frame_init.fs_polarity = SAI_FS_ACTIVE_LOW;
		sai_cfg->frame_init.fs_offset = SAI_FS_BEFOREFIRSTBIT;
	} else {
		/* SAI_I2S_MSBJUSTIFIED or SAI_I2S_LSBJUSTIFIED */
		sai_cfg->frame_init.fs_polarity = SAI_FS_ACTIVE_HIGH;
		sai_cfg->frame_init.fs_offset = SAI_FS_FIRSTBIT;
	}

	/* Frame definition */
	switch (datasize) {
	case SAI_PROTOCOL_DATASIZE_16BIT:
		sai_cfg->init.data_size = SAI_DATASIZE_16;
		sai_cfg->frame_init.frame_length = 32 * (nbslot / 2);
		sai_cfg->frame_init.active_frame_length = 16 * (nbslot / 2);
		sai_cfg->slot_init.slot_size = SAI_SLOTSIZE_16B;
		break;
	case SAI_PROTOCOL_DATASIZE_16BITEXTENDED:
		sai_cfg->init.data_size = SAI_DATASIZE_16;
		sai_cfg->frame_init.frame_length = 64 * (nbslot / 2);
		sai_cfg->frame_init.active_frame_length = 32 * (nbslot / 2);
		sai_cfg->slot_init.slot_size = SAI_SLOTSIZE_32B;
		break;
	case SAI_PROTOCOL_DATASIZE_24BIT:
		sai_cfg->init.data_size = SAI_DATASIZE_24;
		sai_cfg->frame_init.frame_length = 64 * (nbslot / 2);
		sai_cfg->frame_init.active_frame_length = 32 * (nbslot / 2);
		sai_cfg->slot_init.slot_size = SAI_SLOTSIZE_32B;
		break;
	case SAI_PROTOCOL_DATASIZE_32BIT:
		sai_cfg->init.data_size = SAI_DATASIZE_32;
		sai_cfg->frame_init.frame_length = 64 * (nbslot / 2);
		sai_cfg->frame_init.active_frame_length = 32 * (nbslot / 2);
		sai_cfg->slot_init.slot_size = SAI_SLOTSIZE_32B;
		break;
	default:
		return 1;
	}
	if (protocol == SAI_I2S_LSBJUSTIFIED) {
		if (datasize == SAI_PROTOCOL_DATASIZE_16BITEXTENDED) {
			sai_cfg->slot_init.first_bit_offset = 16;
		}
		if (datasize == SAI_PROTOCOL_DATASIZE_24BIT) {
			sai_cfg->slot_init.first_bit_offset = 8;
		}
	}
	return 0;
}

/**
 * @brief  Initialize the SAI PCM protocol according to the specified parameters
 *         in the SAI_InitTypeDef and create the associated handle.
 * @param  sai_cfg pointer to a sai_cfg_t structure that contains
 *                the configuration information for SAI module.
 * @param  protocol one of the supported protocol
 * @param  datasize one of the supported datasize @ref SAI_Protocol_DataSize
 * @param  nbslot number of slot minimum value is 1 and the max is 16.
 * @retval status
 */
static int LL_sai_init_pcm(sai_cfg_t *sai_cfg, uint32_t protocol, uint32_t datasize,
			   uint32_t nbslot)
{
	sai_cfg->init.protocol = SAI_FREE_PROTOCOL;
	sai_cfg->init.first_bit = SAI_FIRSTBIT_MSB;
	/* Compute ClockStrobing according AudioMode */
	if ((sai_cfg->init.audio_mode == SAI_MODEMASTER_TX) ||
	    (sai_cfg->init.audio_mode == SAI_MODESLAVE_TX)) {
		/* Transmit */
		sai_cfg->init.clk_strobing = SAI_CLOCKSTROBING_RISINGEDGE;
	} else {
		/* Receive */
		sai_cfg->init.clk_strobing = SAI_CLOCKSTROBING_FALLINGEDGE;
	}
	sai_cfg->frame_init.fs_definition = SAI_FS_STARTFRAME;
	sai_cfg->frame_init.fs_polarity = SAI_FS_ACTIVE_HIGH;
	sai_cfg->frame_init.fs_offset = SAI_FS_BEFOREFIRSTBIT;
	sai_cfg->slot_init.first_bit_offset = 0;
	sai_cfg->slot_init.n_slot = nbslot;
	sai_cfg->slot_init.slot_active = SAI_SLOTACTIVE_ALL;

	if (protocol == SAI_PCM_SHORT) {
		sai_cfg->frame_init.active_frame_length = 1;
	} else {
		/* SAI_PCM_LONG */
		sai_cfg->frame_init.active_frame_length = 13;
	}

	switch (datasize) {
	case SAI_PROTOCOL_DATASIZE_16BIT:
		sai_cfg->init.data_size = SAI_DATASIZE_16;
		sai_cfg->frame_init.frame_length = 16 * nbslot;
		sai_cfg->slot_init.slot_size = SAI_SLOTSIZE_16B;
		break;
	case SAI_PROTOCOL_DATASIZE_16BITEXTENDED:
		sai_cfg->init.data_size = SAI_DATASIZE_16;
		sai_cfg->frame_init.frame_length = 32 * nbslot;
		sai_cfg->slot_init.slot_size = SAI_SLOTSIZE_32B;
		break;
	case SAI_PROTOCOL_DATASIZE_24BIT:
		sai_cfg->init.data_size = SAI_DATASIZE_24;
		sai_cfg->frame_init.frame_length = 32 * nbslot;
		sai_cfg->slot_init.slot_size = SAI_SLOTSIZE_32B;
		break;
	case SAI_PROTOCOL_DATASIZE_32BIT:
		sai_cfg->init.data_size = SAI_DATASIZE_32;
		sai_cfg->frame_init.frame_length = 32 * nbslot;
		sai_cfg->slot_init.slot_size = SAI_SLOTSIZE_32B;
		break;
	default:
		return 1;
	}

	return 0;
}

/**
 * @brief  Disable all interrupts and clear all flags.
 * @param  sai pointer to a SAI_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */

static void LL_sai_clear_all_irq(SAI_Block_TypeDef *sai)
{
	__SAI_CLEAR_FLAG(sai, 0xFFFFFFFFUL);
	__SAI_DISABLE_IT(sai, 0UL);
}

/**
 * @brief  Flush the fifo .
 * @param  sai pointer to a SAI_Block_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @retval void
 */
static void LL_sai_flush_fifo(SAI_Block_TypeDef *sai)
{
	SET_BIT(sai->CR2, SAI_xCR2_FFLUSH);
}

/* Retrieve the SAIx Block in use*/
/**
 * @brief This retrieve the SAIx (x = 1 or 2) that is use. .
 * @attention Presently select block A for use, if GCR register is set to
 * synchronous, then we should also configure block B just as A. This
 * is a tempoarary solution till we get more information about it.
 * @param SAIx is a pointer to a SAI_TypeDef and Block to be returned
 *@retval SAI_Block_TypeDef*
 */

static SAI_Block_TypeDef *LL_get_sai_block(SAI_TypeDef *SAIx, uint32_t block)
{
	if (SAIx == SAI2) {
		return block == SAIx_BLOCK_A ? SAI2_Block_A : SAI2_Block_B;
	} else {
		return block == SAIx_BLOCK_A ? SAI1_Block_A : SAI1_Block_B;
	}
}

/**
 * @brief  Return the interrupt flag to set according the SAI setup.
 * @param  sai pointer to a SAI_Block_TypeDef structure that contains
 *                the configuration information for SAI module.
 * @param
 * @retval the list of the IT flag to enable
 */
static uint32_t LL_get_sai_interrupt_flag(SAI_Block_TypeDef *sai, uint32_t protocol,
					  uint32_t audiomode)
{
	uint32_t tmpIT = SAI_IT_OVRUDR;

	tmpIT |= SAI_IT_FREQ;

	if ((protocol == SAI_AC97_PROTOCOL) &&
	    ((audiomode == SAI_MODESLAVE_RX) || (audiomode == SAI_MODEMASTER_RX))) {
		tmpIT |= SAI_IT_CNRDY;
	}

	if ((audiomode == SAI_MODESLAVE_RX) || (audiomode == SAI_MODESLAVE_TX)) {
		tmpIT |= SAI_IT_AFSDET | SAI_IT_LFSDET;
	} else {
		/* sai has been configured in master mode */
		tmpIT |= SAI_IT_WCKCFG;
	}
	return tmpIT;
}

/**
 * @}
 */
