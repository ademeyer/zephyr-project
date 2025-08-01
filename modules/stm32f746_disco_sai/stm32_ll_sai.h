/**
 ******************************************************************************
 * @file    stm32_sai.h
 * @author  adebayotimileyin@gmail.com
 * @brief   Header file of SAI Zephyr module.
 ******************************************************************************
 * @attention
 *
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32_SAI__H_
#define __STM32_SAI__H_

#ifdef __cplusplus
extern "C"
{
#endif

#define DEBUG_ASSERT 1

/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include "stm32f746xx.h"

  /** @defgroup SAI_Init_Structure_definition SAI Init Structure definition
   * @brief  SAI Init Structure definition
   * @{
   */
  typedef struct
  {
    uint32_t sai_clock;       /* Clock applied to SAI peripherals */
    uint32_t audio_mode;      /* Specifies the SAI Block audio Mode. */
    uint32_t b_sync;          /* Specifies SAI Block synchronization.*/
    uint32_t synchro_ext;     /* Specifies SAI external output synchronization, this setup is common
               for BlockA and BlockB */
    uint32_t output_drv;      /* Specifies when SAI Block outputs are driven.*/
    uint32_t no_divider;      /* Specifies whether master clock will be divided or not, from 8 to
               256*/
    uint32_t fifo_threshold;  /* Specifies SAI Block FIFO threshold. */
    uint32_t a_freq;          /* Specifies the audio frequency sampling. */
    uint32_t mclk_div;        /* Specifies the master clock divider, between Min_Data = 0 and Max_Data
                  = 15.*/
    uint32_t stereo_mode;     /* Specifies if the mono or stereo mode is selected. */
    uint32_t companding_mode; /* Specifies the companding mode type.*/
    uint32_t tri_state;       /* Specifies the companding mode type.*/
    /* This part of the structure is automatically filled if your are using the high level
    initialisation function LL_sai_init_protocol */
    uint32_t protocol;     /* Specifies the SAI Block protocol.*/
    uint32_t data_size;    /* Specifies the SAI Block data size.*/
    uint32_t first_bit;    /* Specifies whether data transfers start from MSB or LSB bit.*/
    uint32_t clk_strobing; /* Specifies the SAI Block clock strobing edge sensitivity.*/
  } sai_init_cfg_t;

  /** @defgroup SAI_Frame_Structure_definition SAI Frame Structure definition
   * @brief  SAI Frame Init structure definition
   * @note   For SPDIF and AC97 protocol, these parameters are not used (set by hardware).
   * @{
   */
  typedef struct
  {
    uint32_t frame_length;        /* Specifies the Frame length, the number of SCK clocks for each
                      audio frame. Min_Data = 8 and Max_Data = 256. */
    uint32_t active_frame_length; /* Specifies the Frame synchronization active level
                 length.vThis Parameter specifies the length in number of
                 bit clock (SCK + 1) of the active level of FS signal in
                 audio frame. This parameter must be a number between
                 Min_Data = 1 and Max_Data = 128 */
    uint32_t fs_definition;       /* Specifies the Frame synchronization definition. */
    uint32_t fs_polarity;         /* Specifies the Frame synchronization Polarity. */
    uint32_t fs_offset;           /* Specifies the Frame synchronization Offset.*/

  } sai_frame_cfg_t;

  /** @defgroup SAI_Slot_Structure_definition SAI Slot Structure definition
   * @brief   SAI Block Slot Init Structure definition
   * @note    For SPDIF protocol, these parameters are not used (set by hardware).
   * @note    For AC97 protocol, only SlotActive parameter is used (the others are set by hardware).
   * @{
   */
  typedef struct
  {
    uint32_t first_bit_offset; /* Specifies the position of first data transfer bit in the slot.
             This parameter must be a number between Min_Data = 0 and
             Max_Data = 24 */
    uint32_t slot_size;        /* Specifies the Slot Size. */
    uint32_t n_slot;           /* Specifies the number of slot in the audio frame. This parameter must be
                   a number between Min_Data = 1 and Max_Data = 16 */
    uint32_t slot_active;      /* Specifies the slots in audio frame that will be activated. */
  } sai_slot_cfg_t;

  /** @defgroup SAI_Handle_Structure_definition SAI Handle Structure definition
   * @brief  SAI handle Structure definition
   * @{
   */
  typedef struct
  {
    SAI_Block_TypeDef *Instance; /* SAI Blockx registers base address */
    sai_init_cfg_t init;         /* SAI communication parameters */
    sai_frame_cfg_t frame_init;  /* SAI Frame configuration parameters */
    sai_slot_cfg_t slot_init;    /* SAI Slot configuration parameters */
    uint32_t sys_clk;            /* system clock used to check in disable sai function */
  } sai_cfg_t;

/* Exported constants --------------------------------------------------------*/

/** @defgroup SAI_Exported_Constants SAI Exported Constants
 * @{
 */
/************* Kernel DELAY **********/
#define WAIT_DELAY 5000UL
/** @defgroup SAI Register bit functions
 * @{
 */
/** @defgroup SAI_Error_Code SAI Error Code
 * @{
 */
#define SAI_ERROR_NONE \
  ((uint32_t)0x00000000U) /* No error                 				*/
#define SAI_ERROR_OVR \
  ((uint32_t)0x00000001U) /* Overrun Error                               		*/
#define SAI_ERROR_UDR \
  ((uint32_t)0x00000002U) /* Underrun error                              		*/
#define SAI_ERROR_AFSDET \
  ((uint32_t)0x00000004U) /* Anticipated Frame synchronisation detection  	*/
#define SAI_ERROR_LFSDET \
  ((uint32_t)0x00000008U)                         /* Late Frame synchronisation detection 		*/
#define SAI_ERROR_CNREADY ((uint32_t)0x00000010U) /* codec not ready */
#define SAI_ERROR_WCKCFG \
  ((uint32_t)0x00000020U) /* Wrong clock configuration 				*/
#define SAI_ERROR_TIMEOUT \
  ((uint32_t)0x00000040U) /* Timeout error                               		*/
#define SAI_ERROR_DMA \
  ((uint32_t)0x00000080U) /* DMA error                                   		*/

/**
 * @}
 */
/** @defgroup SAI_Block
 * @{
 */
#define SAIx_BLOCK_A ((uint32_t)0x00000000U)
#define SAIx_BLOCK_B ((uint32_t)0x00000001U)

/** @defgroup SAI_Block_SyncExt SAI External synchronisation
 * @{
 */
#define SAI_SYNCEXT_DISABLE 0
#define SAI_SYNCEXT_OUTBLOCKA_ENABLE 1
#define SAI_SYNCEXT_OUTBLOCKB_ENABLE 2
/**
 * @}
 */

/** @defgroup SAI_Protocol SAI Supported protocol
 * @{
 */
#define SAI_I2S_STANDARD 0
#define SAI_I2S_MSBJUSTIFIED 1
#define SAI_I2S_LSBJUSTIFIED 2
#define SAI_PCM_LONG 3
#define SAI_PCM_SHORT 4
/**
 * @}
 */

/** @defgroup SAI_Protocol_DataSize SAI protocol data size
 * @{
 */
#define SAI_PROTOCOL_DATASIZE_16BIT 0
#define SAI_PROTOCOL_DATASIZE_16BITEXTENDED 1
#define SAI_PROTOCOL_DATASIZE_24BIT 2
#define SAI_PROTOCOL_DATASIZE_32BIT 3
/**
 * @}
 */

/** @defgroup SAI_Audio_Frequency SAI Audio Frequency
 * @{
 */
#define SAI_AUDIO_FREQUENCY_192K ((uint32_t)192000U)
#define SAI_AUDIO_FREQUENCY_96K ((uint32_t)96000U)
#define SAI_AUDIO_FREQUENCY_48K ((uint32_t)48000U)
#define SAI_AUDIO_FREQUENCY_44K ((uint32_t)44100U)
#define SAI_AUDIO_FREQUENCY_32K ((uint32_t)32000U)
#define SAI_AUDIO_FREQUENCY_22K ((uint32_t)22050U)
#define SAI_AUDIO_FREQUENCY_16K ((uint32_t)16000U)
#define SAI_AUDIO_FREQUENCY_11K ((uint32_t)11025U)
#define SAI_AUDIO_FREQUENCY_8K ((uint32_t)8000U)
#define SAI_AUDIO_FREQUENCY_MCKDIV ((uint32_t)0U)
/**
 * @}
 */

/** @defgroup SAI_Block_Mode SAI Block Mode
 * @{
 */
#define SAI_MODEMASTER_TX ((uint32_t)0x00000000U)
#define SAI_MODEMASTER_RX ((uint32_t)SAI_xCR1_MODE_0)
#define SAI_MODESLAVE_TX ((uint32_t)SAI_xCR1_MODE_1)
#define SAI_MODESLAVE_RX ((uint32_t)(SAI_xCR1_MODE_1 | SAI_xCR1_MODE_0))

/**
 * @}
 */

/** @defgroup SAI_Block_Protocol SAI Block Protocol
 * @{
 */
#define SAI_FREE_PROTOCOL ((uint32_t)0x00000000U)
#define SAI_SPDIF_PROTOCOL ((uint32_t)SAI_xCR1_PRTCFG_0)
#define SAI_AC97_PROTOCOL ((uint32_t)SAI_xCR1_PRTCFG_1)
/**
 * @}
 */

/** @defgroup SAI_Block_Data_Size SAI Block Data Size
 * @{
 */
#define SAI_DATASIZE_8 ((uint32_t)SAI_xCR1_DS_1)
#define SAI_DATASIZE_10 ((uint32_t)(SAI_xCR1_DS_1 | SAI_xCR1_DS_0))
#define SAI_DATASIZE_16 ((uint32_t)SAI_xCR1_DS_2)
#define SAI_DATASIZE_20 ((uint32_t)(SAI_xCR1_DS_2 | SAI_xCR1_DS_0))
#define SAI_DATASIZE_24 ((uint32_t)(SAI_xCR1_DS_2 | SAI_xCR1_DS_1))
#define SAI_DATASIZE_32 ((uint32_t)(SAI_xCR1_DS_2 | SAI_xCR1_DS_1 | SAI_xCR1_DS_0))
/**
 * @}
 */

/** @defgroup SAI_Block_MSB_LSB_transmission SAI Block MSB LSB transmission
 * @{
 */
#define SAI_FIRSTBIT_MSB ((uint32_t)0x00000000U)
#define SAI_FIRSTBIT_LSB ((uint32_t)SAI_xCR1_LSBFIRST)
/**
 * @}
 */

/** @defgroup SAI_Block_Clock_Strobing SAI Block Clock Strobing
 * @{
 */
#define SAI_CLOCKSTROBING_FALLINGEDGE 0
#define SAI_CLOCKSTROBING_RISINGEDGE 1
/**
 * @}
 */

/** @defgroup SAI_Block_Synchronization SAI Block Synchronization
 * @{
 */
#define SAI_ASYNCHRONOUS 0         /* Asynchronous */
#define SAI_SYNCHRONOUS 1          /* Synchronous with other block of same SAI */
#define SAI_SYNCHRONOUS_EXT_SAI1 2 /* Synchronous with other SAI, SAI1 */
#define SAI_SYNCHRONOUS_EXT_SAI2 3 /* Synchronous with other SAI, SAI2 */
/**
 * @}
 */

/** @defgroup SAI_Block_Output_Drive SAI Block Output Drive
 * @{
 */
#define SAI_OUTPUTDRIVE_DISABLE ((uint32_t)0x00000000U)
#define SAI_OUTPUTDRIVE_ENABLE ((uint32_t)SAI_xCR1_OUTDRIV)
/**
 * @}
 */

/** @defgroup SAI_Block_NoDivider SAI Block NoDivider
 * @{
 */
#define SAI_MASTERDIVIDER_ENABLE ((uint32_t)0x00000000U)
#define SAI_MASTERDIVIDER_DISABLE ((uint32_t)SAI_xCR1_NODIV)
/**
 * @}
 */

/** @defgroup SAI_Block_FS_Definition SAI Block FS Definition
 * @{
 */
#define SAI_FS_STARTFRAME ((uint32_t)0x00000000U)
#define SAI_FS_CHANNEL_IDENTIFICATION ((uint32_t)SAI_xFRCR_FSDEF)
/**
 * @}
 */

/** @defgroup SAI_Block_FS_Polarity SAI Block FS Polarity
 * @{
 */
#define SAI_FS_ACTIVE_LOW ((uint32_t)0x00000000U)
#define SAI_FS_ACTIVE_HIGH ((uint32_t)SAI_xFRCR_FSPOL)
/**
 * @}
 */

/** @defgroup SAI_Block_FS_Offset SAI Block FS Offset
 * @{
 */
#define SAI_FS_FIRSTBIT ((uint32_t)0x00000000U)
#define SAI_FS_BEFOREFIRSTBIT ((uint32_t)SAI_xFRCR_FSOFF)
/**
 * @}
 */

/** @defgroup SAI_Block_Slot_Size SAI Block Slot Size
 * @{
 */
#define SAI_SLOTSIZE_DATASIZE ((uint32_t)0x00000000U)
#define SAI_SLOTSIZE_16B ((uint32_t)SAI_xSLOTR_SLOTSZ_0)
#define SAI_SLOTSIZE_32B ((uint32_t)SAI_xSLOTR_SLOTSZ_1)
/**
 * @}
 */

/** @defgroup SAI_Block_Slot_Active SAI Block Slot Active
 * @{
 */
#define SAI_SLOT_NOTACTIVE ((uint32_t)0x00000000U)
#define SAI_SLOTACTIVE_0 ((uint32_t)0x00000001U)
#define SAI_SLOTACTIVE_1 ((uint32_t)0x00000002U)
#define SAI_SLOTACTIVE_2 ((uint32_t)0x00000004U)
#define SAI_SLOTACTIVE_3 ((uint32_t)0x00000008U)
#define SAI_SLOTACTIVE_4 ((uint32_t)0x00000010U)
#define SAI_SLOTACTIVE_5 ((uint32_t)0x00000020U)
#define SAI_SLOTACTIVE_6 ((uint32_t)0x00000040U)
#define SAI_SLOTACTIVE_7 ((uint32_t)0x00000080U)
#define SAI_SLOTACTIVE_8 ((uint32_t)0x00000100U)
#define SAI_SLOTACTIVE_9 ((uint32_t)0x00000200U)
#define SAI_SLOTACTIVE_10 ((uint32_t)0x00000400U)
#define SAI_SLOTACTIVE_11 ((uint32_t)0x00000800U)
#define SAI_SLOTACTIVE_12 ((uint32_t)0x00001000U)
#define SAI_SLOTACTIVE_13 ((uint32_t)0x00002000U)
#define SAI_SLOTACTIVE_14 ((uint32_t)0x00004000U)
#define SAI_SLOTACTIVE_15 ((uint32_t)0x00008000U)
#define SAI_SLOTACTIVE_ALL ((uint32_t)0x0000FFFFU)
/**
 * @}
 */

/** @defgroup SAI_Mono_Stereo_Mode SAI Mono Stereo Mode
 * @{
 */
#define SAI_STEREOMODE ((uint32_t)0x00000000U)
#define SAI_MONOMODE ((uint32_t)SAI_xCR1_MONO)
/**
 * @}
 */

/** @defgroup SAI_TRIState_Management SAI TRIState Management
 * @{
 */
#define SAI_OUTPUT_NOTRELEASED ((uint32_t)0x00000000U)
#define SAI_OUTPUT_RELEASED ((uint32_t)SAI_xCR2_TRIS)
/**
 * @}
 */

/** @defgroup SAI_Block_Fifo_Threshold SAI Block Fifo Threshold
 * @{
 */
#define SAI_FIFOTHRESHOLD_EMPTY ((uint32_t)0x00000000U)
#define SAI_FIFOTHRESHOLD_1QF ((uint32_t)(SAI_xCR2_FTH_0))
#define SAI_FIFOTHRESHOLD_HF ((uint32_t)(SAI_xCR2_FTH_1))
#define SAI_FIFOTHRESHOLD_3QF ((uint32_t)(SAI_xCR2_FTH_1 | SAI_xCR2_FTH_0))
#define SAI_FIFOTHRESHOLD_FULL ((uint32_t)(SAI_xCR2_FTH_2))
/**
 * @}
 */

/** @defgroup SAI_Block_Companding_Mode SAI Block Companding Mode
 * @{
 */
#define SAI_NOCOMPANDING ((uint32_t)0x00000000U)
#define SAI_ULAW_1CPL_COMPANDING ((uint32_t)(SAI_xCR2_COMP_1))
#define SAI_ALAW_1CPL_COMPANDING ((uint32_t)(SAI_xCR2_COMP_1 | SAI_xCR2_COMP_0))
#define SAI_ULAW_2CPL_COMPANDING ((uint32_t)(SAI_xCR2_COMP_1 | SAI_xCR2_CPL))
#define SAI_ALAW_2CPL_COMPANDING ((uint32_t)(SAI_xCR2_COMP_1 | SAI_xCR2_COMP_0 | SAI_xCR2_CPL))
/**
 * @}
 */

/** @defgroup SAI_Block_Mute_Value SAI Block Mute Value
 * @{
 */
#define SAI_ZERO_VALUE ((uint32_t)0x00000000U)
#define SAI_LAST_SENT_VALUE ((uint32_t)SAI_xCR2_MUTEVAL)
/**
 * @}
 */

/** @defgroup SAI_Block_Interrupts_Definition SAI Block Interrupts Definition
 * @{
 */
#define SAI_IT_OVRUDR ((uint32_t)SAI_xIMR_OVRUDRIE)
#define SAI_IT_MUTEDET ((uint32_t)SAI_xIMR_MUTEDETIE)
#define SAI_IT_WCKCFG ((uint32_t)SAI_xIMR_WCKCFGIE)
#define SAI_IT_FREQ ((uint32_t)SAI_xIMR_FREQIE)
#define SAI_IT_CNRDY ((uint32_t)SAI_xIMR_CNRDYIE)
#define SAI_IT_AFSDET ((uint32_t)SAI_xIMR_AFSDETIE)
#define SAI_IT_LFSDET ((uint32_t)SAI_xIMR_LFSDETIE)
/**
 * @}
 */

/** @defgroup SAI_Block_Flags_Definition  SAI Block Flags Definition
 * @{
 */
#define SAI_FLAG_OVRUDR ((uint32_t)SAI_xSR_OVRUDR)
#define SAI_FLAG_MUTEDET ((uint32_t)SAI_xSR_MUTEDET)
#define SAI_FLAG_WCKCFG ((uint32_t)SAI_xSR_WCKCFG)
#define SAI_FLAG_FREQ ((uint32_t)SAI_xSR_FREQ)
#define SAI_FLAG_CNRDY ((uint32_t)SAI_xSR_CNRDY)
#define SAI_FLAG_AFSDET ((uint32_t)SAI_xSR_AFSDET)
#define SAI_FLAG_LFSDET ((uint32_t)SAI_xSR_LFSDET)
/**
 * @}
 */

/** @defgroup SAI_Block_Fifo_Status_Level   SAI Block Fifo Status Level
 * @{
 */
#define SAI_FIFOSTATUS_EMPTY ((uint32_t)0x00000000U)
#define SAI_FIFOSTATUS_LESS1QUARTERFULL ((uint32_t)0x00010000U)
#define SAI_FIFOSTATUS_1QUARTERFULL ((uint32_t)0x00020000U)
#define SAI_FIFOSTATUS_HALFFULL ((uint32_t)0x00030000U)
#define SAI_FIFOSTATUS_3QUARTERFULL ((uint32_t)0x00040000U)
#define SAI_FIFOSTATUS_FULL ((uint32_t)0x00050000U)
/**
 * @}
 */

/**
 * @}
 */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup SAI_Exported_Macros SAI Exported Macros
 * @brief macros to handle interrupts and specific configurations
 * @{
 */

/** @brief Reset SAI handle state.
 * @param  __HANDLE__ specifies the SAI Handle.
 * @retval None
 */
#define SET 1
#define RESET 0

#define __SAI_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->state = SAI_STATE_RESET)

/** @brief  Enable or disable the specified SAI interrupts.
 * @param  __HANDLE__ specifies the SAI Handle.
 * @param  __INTERRUPT__ specifies the interrupt source to enable or disable.
 *         This parameter can be one of the following values:
 *            @arg SAI_IT_OVRUDR: Overrun underrun interrupt enable
 *            @arg SAI_IT_MUTEDET: Mute detection interrupt enable
 *            @arg SAI_IT_WCKCFG: Wrong Clock Configuration interrupt enable
 *            @arg SAI_IT_FREQ: FIFO request interrupt enable
 *            @arg SAI_IT_CNRDY: Codec not ready interrupt enable
 *            @arg SAI_IT_AFSDET: Anticipated frame synchronization detection interrupt enable
 *            @arg SAI_IT_LFSDET: Late frame synchronization detection interrupt enable
 * @retval None
 */
#define __SAI_ENABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->IMR |= (__INTERRUPT__))
#define __SAI_DISABLE_IT(__HANDLE__, __INTERRUPT__) ((__HANDLE__)->IMR &= (~(__INTERRUPT__)))

/** @brief  Check whether the specified SAI interrupt source is enabled or not.
 * @param  __HANDLE__ specifies the SAI Handle.
 * @param  __INTERRUPT__ specifies the SAI interrupt source to check.
 *         This parameter can be one of the following values:
 *            @arg SAI_IT_OVRUDR: Overrun underrun interrupt enable
 *            @arg SAI_IT_MUTEDET: Mute detection interrupt enable
 *            @arg SAI_IT_WCKCFG: Wrong Clock Configuration interrupt enable
 *            @arg SAI_IT_FREQ: FIFO request interrupt enable
 *            @arg SAI_IT_CNRDY: Codec not ready interrupt enable
 *            @arg SAI_IT_AFSDET: Anticipated frame synchronization detection interrupt enable
 *            @arg SAI_IT_LFSDET: Late frame synchronization detection interrupt enable
 * @retval The new state of __INTERRUPT__ (TRUE or FALSE).
 */
#define __SAI_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) \
  ((((__HANDLE__)->IMR & (__INTERRUPT__)) == (__INTERRUPT__)) ? SET : RESET)

/** @brief  Check whether the specified SAI flag is set or not.
 * @param  __HANDLE__ specifies the SAI Handle.
 * @param  __FLAG__ specifies the flag to check.
 *         This parameter can be one of the following values:
 *            @arg SAI_FLAG_OVRUDR: Overrun underrun flag.
 *            @arg SAI_FLAG_MUTEDET: Mute detection flag.
 *            @arg SAI_FLAG_WCKCFG: Wrong Clock Configuration flag.
 *            @arg SAI_FLAG_FREQ: FIFO request flag.
 *            @arg SAI_FLAG_CNRDY: Codec not ready flag.
 *            @arg SAI_FLAG_AFSDET: Anticipated frame synchronization detection flag.
 *            @arg SAI_FLAG_LFSDET: Late frame synchronization detection flag.
 * @retval The new state of __FLAG__ (TRUE or FALSE).
 */
#define __SAI_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->SR) & (__FLAG__)) == (__FLAG__))

/** @brief  Clear the specified SAI pending flag.
 * @param  __HANDLE__ specifies the SAI Handle.
 * @param  __FLAG__ specifies the flag to check.
 *          This parameter can be any combination of the following values:
 *            @arg SAI_FLAG_OVRUDR: Clear Overrun underrun
 *            @arg SAI_FLAG_MUTEDET: Clear Mute detection
 *            @arg SAI_FLAG_WCKCFG: Clear Wrong Clock Configuration
 *            @arg SAI_FLAG_FREQ: Clear FIFO request
 *            @arg SAI_FLAG_CNRDY: Clear Codec not ready
 *            @arg SAI_FLAG_AFSDET: Clear Anticipated frame synchronization detection
 *            @arg SAI_FLAG_LFSDET: Clear Late frame synchronization detection
 *
 * @retval None
 */
#define __SAI_CLEAR_FLAG(__HANDLE__, __FLAG__) ((__HANDLE__)->CLRFR = (__FLAG__))

#define __SAI_ENABLE(__HANDLE__) ((__HANDLE__)->CR1 |= SAI_xCR1_SAIEN)
#define __SAI_DISABLE(__HANDLE__) ((__HANDLE__)->CR1 &= ~SAI_xCR1_SAIEN)

  /**
   * @}
   */

  /* Exported functions --------------------------------------------------------*/
  /** @addtogroup SAI_Exported_Functions
   * @{
   */

  /* Initialization/de-initialization functions  ********************************/
  /** @addtogroup SAI_Exported_Functions_Group1
   * @{
   */
  int LL_sai_init_ll_subsystem(SAI_TypeDef *sai, uint32_t protocol, uint32_t datasize,
                               uint32_t audiomode, uint32_t audiofrequency, uint32_t nbslot,
                               uint32_t clk_strobs, uint32_t pclk);
  int LL_sai_deinit_ll_subsystem(SAI_TypeDef *sai);

  /**
   * @}
   */

  /* I/O operation functions  ***************************************************/
  /** @addtogroup SAI_Exported_Functions_Group2
   * @{
   */

  /* Operational Functions */
  void LL_sai_set_transfer_mode_ll_subsystem(SAI_TypeDef *sai_x, uint32_t mode);
  void LL_sai_enable_tx_mute_mode_ll_subsystem(SAI_TypeDef *sai, uint16_t val);
  void LL_sai_disable_tx_mute_mode_ll_subsystem(SAI_TypeDef *sai);

  /* SAI IO Register Functions */
  __IO uint32_t *LL_sai_get_register_ll_subsystem(SAI_TypeDef *sai);

  /* DMA Related Functions */
  void LL_sai_dma_resume_ll_subsystem(SAI_TypeDef *sai);
  int LL_sai_dma_abort_ll_subsystem(SAI_TypeDef *sai);
  int LL_sai_get_dma_status_ll_subsystem(SAI_TypeDef *sai);

  /* Peripheral enable / disable */
  int LL_sai_disable_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_enable_ll_subsystem(SAI_TypeDef *sai);

  /* Disable ISR Event */
  void LL_sai_disable_OVRUDR_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_disable_MUTEDET_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_disable_WCKCFG_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_disable_FREQ_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_disable_CNRDY_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_disable_AFSDET_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_disable_LFSDET_ll_subsystem(SAI_TypeDef *sai);

  /* Enable ISR Event */
  void LL_sai_enable_OVRUDR_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_emable_MUTEDET_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_enable_WCKCFG_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_enable_FREQ_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_enable_CNRDY_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_enable_AFSDET_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_enable_LFSDET_ll_subsystem(SAI_TypeDef *sai);

  /* Get ISR Event */
  int LL_sai_get_OVRUDR_ll_subsystem(SAI_TypeDef *sai);
  int LL_sai_get_MUTEDET_ll_subsystem(SAI_TypeDef *sai);
  int LL_sai_get_WCKCFG_ll_subsystem(SAI_TypeDef *sai);
  int LL_sai_get_FREQ_ll_subsystem(SAI_TypeDef *sai);
  int LL_sai_get_CNRDY_ll_subsystem(SAI_TypeDef *sai);
  int LL_sai_get_AFSDET_ll_subsystem(SAI_TypeDef *sai);
  int LL_sai_get_LFSDET_ll_subsystem(SAI_TypeDef *sai);

  /* Set ISR Event */
  void LL_sai_clear_OVRUDR_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_clear_MUTEDET_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_clear_WCKCFG_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_clear_FREQ_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_clear_CNRDY_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_clear_AFSDET_ll_subsystem(SAI_TypeDef *sai);
  void LL_sai_clear_LFSDET_ll_subsystem(SAI_TypeDef *sai);

  /**
   * @}
   */

  /** @addtogroup SAI_Exported_Functions_Group3
   * @{
   */

#define LL_CLEAR_BIT(REG, BIT_MASK) ((REG) &= ~(BIT_MASK)) /* Clear register bit */
#define LL_SET_BIT(REG, BIT) ((REG) |= (BIT))              /* Set register bit */

#define LL_IS_SAI_ALL_INSTANCE(INSTANCE) (INSTANCE == SAI1) || (INSTANCE == SAI2)

#define LL_IS_SAI_BLOCK_SYNCEXT(STATE) \
  (((STATE) == SAI_SYNCEXT_DISABLE) || \
   ((STATE) == SAI_SYNCEXT_OUTBLOCKA_ENABLE) | ((STATE) == SAI_SYNCEXT_OUTBLOCKB_ENABLE))

#define LL_IS_SAI_SUPPORTED_PROTOCOL(PROTOCOL)                                 \
  (((PROTOCOL) == SAI_I2S_STANDARD) || ((PROTOCOL) == SAI_I2S_LSBJUSTIFIED) || \
   ((PROTOCOL) == SAI_I2S_MSBJUSTIFIED) || ((PROTOCOL) == SAI_PCM_SHORT) ||    \
   ((PROTOCOL) == SAI_PCM_LONG))

#define LL_IS_SAI_PROTOCOL_DATASIZE(DATASIZE)             \
  (((DATASIZE) == SAI_PROTOCOL_DATASIZE_16BIT) ||         \
   ((DATASIZE) == SAI_PROTOCOL_DATASIZE_16BITEXTENDED) || \
   ((DATASIZE) == SAI_PROTOCOL_DATASIZE_24BIT) ||         \
   ((DATASIZE) == SAI_PROTOCOL_DATASIZE_32BIT))

#define LL_IS_SAI_AUDIO_FREQUENCY(AUDIO)                                            \
  (((AUDIO) == SAI_AUDIO_FREQUENCY_192K) || ((AUDIO) == SAI_AUDIO_FREQUENCY_96K) || \
   ((AUDIO) == SAI_AUDIO_FREQUENCY_48K) || ((AUDIO) == SAI_AUDIO_FREQUENCY_44K) ||  \
   ((AUDIO) == SAI_AUDIO_FREQUENCY_32K) || ((AUDIO) == SAI_AUDIO_FREQUENCY_22K) ||  \
   ((AUDIO) == SAI_AUDIO_FREQUENCY_16K) || ((AUDIO) == SAI_AUDIO_FREQUENCY_11K) ||  \
   ((AUDIO) == SAI_AUDIO_FREQUENCY_8K) || ((AUDIO) == SAI_AUDIO_FREQUENCY_MCKDIV))

#define LL_IS_SAI_BLOCK_MODE(MODE)                                   \
  (((MODE) == SAI_MODEMASTER_TX) || ((MODE) == SAI_MODEMASTER_RX) || \
   ((MODE) == SAI_MODESLAVE_TX) || ((MODE) == SAI_MODESLAVE_RX))

#define LL_IS_SAI_BLOCK_PROTOCOL(PROTOCOL)                                   \
  (((PROTOCOL) == SAI_FREE_PROTOCOL) || ((PROTOCOL) == SAI_AC97_PROTOCOL) || \
   ((PROTOCOL) == SAI_SPDIF_PROTOCOL))

#define LL_IS_SAI_BLOCK_DATASIZE(DATASIZE)                               \
  (((DATASIZE) == SAI_DATASIZE_8) || ((DATASIZE) == SAI_DATASIZE_10) ||  \
   ((DATASIZE) == SAI_DATASIZE_16) || ((DATASIZE) == SAI_DATASIZE_20) || \
   ((DATASIZE) == SAI_DATASIZE_24) || ((DATASIZE) == SAI_DATASIZE_32))

#define LL_IS_SAI_BLOCK_FIRST_BIT(BIT) (((BIT) == SAI_FIRSTBIT_MSB) || ((BIT) == SAI_FIRSTBIT_LSB))

#define LL_IS_SAI_BLOCK_CLOCK_STROBING(CLOCK) \
  (((CLOCK) == SAI_CLOCKSTROBING_FALLINGEDGE) || ((CLOCK) == SAI_CLOCKSTROBING_RISINGEDGE))

#define LL_IS_SAI_BLOCK_SYNCHRO(SYNCHRO)                                \
  (((SYNCHRO) == SAI_ASYNCHRONOUS) || ((SYNCHRO) == SAI_SYNCHRONOUS) || \
   ((SYNCHRO) == SAI_SYNCHRONOUS_EXT_SAI1) || ((SYNCHRO) == SAI_SYNCHRONOUS_EXT_SAI2))

#define LL_IS_SAI_BLOCK_OUTPUT_DRIVE(DRIVE) \
  (((DRIVE) == SAI_OUTPUTDRIVE_DISABLE) || ((DRIVE) == SAI_OUTPUTDRIVE_ENABLE))

#define LL_IS_SAI_BLOCK_NODIVIDER(NODIVIDER) \
  (((NODIVIDER) == SAI_MASTERDIVIDER_ENABLE) || ((NODIVIDER) == SAI_MASTERDIVIDER_DISABLE))

#define LL_IS_SAI_BLOCK_MUTE_COUNTER(COUNTER) ((COUNTER) <= 63)

#define LL_IS_SAI_BLOCK_MUTE_VALUE(VALUE) \
  (((VALUE) == SAI_ZERO_VALUE) || ((VALUE) == SAI_LAST_SENT_VALUE))

#define LL_IS_SAI_BLOCK_COMPANDING_MODE(MODE)                                      \
  (((MODE) == SAI_NOCOMPANDING) || ((MODE) == SAI_ULAW_1CPL_COMPANDING) ||         \
   ((MODE) == SAI_ALAW_1CPL_COMPANDING) || ((MODE) == SAI_ULAW_2CPL_COMPANDING) || \
   ((MODE) == SAI_ALAW_2CPL_COMPANDING))

#define LL_IS_SAI_BLOCK_FIFO_THRESHOLD(THRESHOLD)                                        \
  (((THRESHOLD) == SAI_FIFOTHRESHOLD_EMPTY) || ((THRESHOLD) == SAI_FIFOTHRESHOLD_1QF) || \
   ((THRESHOLD) == SAI_FIFOTHRESHOLD_HF) || ((THRESHOLD) == SAI_FIFOTHRESHOLD_3QF) ||    \
   ((THRESHOLD) == SAI_FIFOTHRESHOLD_FULL))

#define LL_IS_SAI_BLOCK_TRISTATE_MANAGEMENT(STATE) \
  (((STATE) == SAI_OUTPUT_NOTRELEASED) || ((STATE) == SAI_OUTPUT_RELEASED))

#define LL_IS_SAI_MONO_STEREO_MODE(MODE) (((MODE) == SAI_MONOMODE) || ((MODE) == SAI_STEREOMODE))

#define LL_IS_SAI_SLOT_ACTIVE(ACTIVE) ((ACTIVE) <= SAI_SLOTACTIVE_ALL)

#define LL_IS_SAI_BLOCK_SLOT_NUMBER(NUMBER) ((1 <= (NUMBER)) && ((NUMBER) <= 16))

#define LL_IS_SAI_BLOCK_SLOT_SIZE(SIZE)                                 \
  (((SIZE) == SAI_SLOTSIZE_DATASIZE) || ((SIZE) == SAI_SLOTSIZE_16B) || \
   ((SIZE) == SAI_SLOTSIZE_32B))

#define LL_IS_SAI_BLOCK_FIRSTBIT_OFFSET(OFFSET) ((OFFSET) <= 24)

#define LL_IS_SAI_BLOCK_FS_OFFSET(OFFSET) \
  (((OFFSET) == SAI_FS_FIRSTBIT) | ((OFFSET) == SAI_FS_BEFOREFIRSTBIT))

#define LL_IS_SAI_BLOCK_FS_POLARITY(POLARITY) \
  (((POLARITY) == SAI_FS_ACTIVE_LOW) || ((POLARITY) == SAI_FS_ACTIVE_HIGH))

#define LL_IS_SAI_BLOCK_FS_DEFINITION(DEFINITION) \
  (((DEFINITION) == SAI_FS_STARTFRAME) || ((DEFINITION) == SAI_FS_CHANNEL_IDENTIFICATION))

#define LL_IS_SAI_BLOCK_MASTER_DIVIDER(DIVIDER) ((DIVIDER) <= 15U)

#define LL_IS_SAI_BLOCK_FRAME_LENGTH(LENGTH) ((8 <= (LENGTH)) && ((LENGTH) <= 256))

#define LL_IS_SAI_BLOCK_ACTIVE_FRAME(LENGTH) ((1 <= (LENGTH)) && ((LENGTH) <= 128))

#ifdef __cplusplus
}
#endif
#endif
