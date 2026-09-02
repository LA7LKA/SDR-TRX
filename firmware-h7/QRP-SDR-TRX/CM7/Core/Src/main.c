/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "dsp.h"
#include "modes.h"
#include "filters.h"
#include "rx_chain.h"
#include "tx_chain.h"
#include "freedv_chain.h"
#include "cw_paddle.h"
#include "ipc_mailbox.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
/* Permanently disabled: CM4's own peripherals (GPIO/I2C for the OLED)
   don't depend on CM7 finishing SystemClock_Config() first - D2 is a
   mostly-independent clock domain - so this synchronization wasn't
   actually needed. Keeping it enabled meant every SWD reset re-entered
   CM4's Stop-mode wait, racing OpenOCD's post-reset halt/examine against
   CM4 going back to sleep; losing that race needed a full flash mass
   erase to recover from. Not worth the debugging friction for a
   guarantee CM4 doesn't rely on. */
// #define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* Which HSE source is physically wired to OSC_IN - 0 means "don't use HSE
   at all, run from the internal HSI".
     0     = HSI only. Current default and, measured, the best available
             on this board without hardware work.
     8000  = ST-LINK MCO (the factory SB72-ON wiring). Tried 2026-08-15
             with HSE_ON so the signal goes through the crystal oscillator
             amplifier rather than BYPASS's direct path, on the theory that
             the amp might filter the MCO's jitter the way F746 appeared to
             benefit. It does not: measured 3x worse short-term wander than
             HSI (std 3.59 Hz vs 1.20 Hz on a 1 kHz SSB tone, 60 s), and
             worse drift. Don't re-try this without new evidence.
     25000 = X2 crystal (25 MHz, 20 ppm), after the UM2408 sec 7.9.1
             rework: SB3/SB4 ON, C74/C76 fitted with 5.1 pF, SB72/SB71/
             SB90 OFF - and note X2 itself looks to be an unpopulated
             footprint on this board, so the crystal likely has to be
             sourced and fitted too. This is the path worth taking, or a
             TCXO in its place. */
#define HSE_SRC_KHZ  0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;
DMA_HandleTypeDef hdma_dac1_ch2;

OPAMP_HandleTypeDef hopamp1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/*
 * ADC/DAC ring buffers, shared scratch, and RX/TX dispatch - ported from
 * firmware/Core/Src/main.c. NOT YET PORTED from F746, deliberately: the
 * USB Audio glue (usb_audio_tx_source()/usb_audio_rx_capture()/
 * usb_audio_mic_tx_update(), audio_source switching) - F746's is built on
 * a hand-written composite Audio+CDC class (usbd_audio_duplex.c) that
 * doesn't exist here yet; this project's CubeMX-generated USB Audio class
 * is the stock single-direction one. Analog RX/TX works fully below; USB
 * audio integration is separate follow-up work, not a shortcut taken here.
 * Likewise the UART console/telemetry (uart_puts() etc. are no-op stubs -
 * USART3 isn't enabled in this .ioc yet) and the CAT command layer.
 */

#define ADC_RING_BLOCKS 4
#define ADC_RING_LEN    (ADC_RING_BLOCKS * (BLOCK_SIZE_MAX / 2))
#define DAC_RING_BLOCKS 4
#define DAC_RING_LEN    (DAC_RING_BLOCKS * (BLOCK_SIZE_MAX / 2))

/* Active block size, swapped by radio_set_mode()/radio_apply_mode(). */
static volatile uint32_t block_size = BLOCK_SIZE_ANALOG;

/* No static initializer: this section is non-cacheable/NOLOAD (see the
   linker script), so these aren't zero-init'd by the startup .bss copy -
   each is explicitly filled at runtime below, before its DMA is started. */
uint16_t dac_buffer[DAC_RING_LEN] __attribute__((section(".dma_buffers")));
uint16_t adc_buffer[ADC_RING_LEN] __attribute__((section(".dma_buffers")));
uint16_t sidetone_buffer[BLOCK_SIZE_ANALOG] __attribute__((section(".dma_buffers")));   /* CW sidetone only */

volatile uint8_t block_ready = 0;          /* 0 none, 1 first half, 2 second half */
volatile uint8_t dac_block_processing = 0;

int freedv_ok = 0;

#define SCRATCH_N (BLOCK_SIZE_MAX / 2)
float dsp_scratch[10 * SCRATCH_N];

/* RX telemetry - what the IPC mailbox forwards to CM4's S-meter. */
volatile uint32_t rx_blocks = 0;
volatile float    rx_peak   = 0.0f;
volatile float    rx_rms    = 0.0f;
volatile float    rx_env_min = 0.0f;
volatile float    rx_env_avg = 0.0f;
volatile float    rx_adc_peak = 0.0f;
volatile uint32_t rx_cycles_max = 0;
volatile uint32_t rx_fdv_cycles_max = 0;
volatile uint32_t poll_gap_cycles_max = 0;   /* see mode_is_buffered() dispatch below */

/* Which oscillator SystemClock_Config() actually managed to start: 1 = the
   X2 crystal on HSE, 0 = the internal HSI RC it falls back to. All three
   PLLs share PLLCKSELR.PLLSRC, so PeriphCommonClock_Config()'s PLL2 has to
   branch on this too, not just PLL1. */
volatile int32_t clock_src_hse = 0;
volatile uint32_t rx_adc_cycles = 0;
volatile uint32_t tx_cushion_min = 999999;
volatile uint32_t rx_overruns = 0;

volatile int tx_active = 0;
volatile int tx_rearm  = 0;

/* No USART3 in this .ioc yet - stubs so freedv_chain.c's alloc-failure
   reporting (codec2_alloc_report_fail()) links; wire these to a real UART
   once USART3 is enabled in CubeMX. */
void uart_puts(const char *s) { (void)s; }
void uart_flush_blocking(void) { }

static uint32_t adc_rd;

static uint32_t adc_dma_pos(void)
{
    ADC_HandleTypeDef *adc = tx_active ? &hadc2 : &hadc1;
    return ADC_RING_LEN - __HAL_DMA_GET_COUNTER(adc->DMA_Handle);
}

static uint32_t dac_dma_pos(void)
{
    DMA_HandleTypeDef *dma = tx_active ? &hdma_dac1_ch2 : &hdma_dac1_ch1;
    return DAC_RING_LEN - __HAL_DMA_GET_COUNTER(dma);
}

static uint32_t adc_avail(void)
{
    uint32_t w = adc_dma_pos();
    return (w >= adc_rd) ? (w - adc_rd) : (ADC_RING_LEN - adc_rd + w);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    if (mode_is_buffered(MODE)) return;
    if (block_ready) rx_overruns++;
    block_ready = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    if (mode_is_buffered(MODE)) return;
    if (block_ready) rx_overruns++;
    block_ready = 2;
}

void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac) { (void)hdac; dac_block_processing = 1; }
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)     { (void)hdac; dac_block_processing = 2; }
void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac) { (void)hdac; dac_block_processing = 1; }
void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac)     { (void)hdac; dac_block_processing = 2; }

#define DAC_MID 2048

/*
 * (Re)initialise every filter/NCO/oscillator (filters.c) - called on mode
 * change, PTT, and (once ported) an audio-source switch, so stale state
 * from before never bleeds into the new stream.
 */
static void dsp_and_dma_flush(void)
{
    dsp_filters_init();
}

/*
 * Half duplex: exactly one ADC and DAC_OUT1/DAC_OUT2 pair carries the
 * "live" signal at a time, chosen by tx_active. RX reads IF off hadc1
 * (ADC1 ch4, via OPAMP1) and writes demodulated audio to DAC_OUT1. TX
 * reads the mic off hadc2 (ADC2 ch3) and writes modulated IF to DAC_OUT2.
 * One exception: DAC_OUT1 also keeps running during CW TX, carrying the
 * sidetone, same as F746.
 */
static void audio_dma_start(void)
{
    for (uint32_t i = 0; i < DAC_RING_LEN; i++) dac_buffer[i] = DAC_MID;
    for (uint32_t i = 0; i < BLOCK_SIZE_ANALOG; i++) sidetone_buffer[i] = DAC_MID;
    memset(adc_buffer, 0, sizeof(adc_buffer));

    block_ready = 0;
    dac_block_processing = 0;
    adc_rd = 0;

    uint32_t dac_len = (tx_active && mode_is_buffered(MODE)) ? DAC_RING_LEN : block_size_for(MODE);
    uint32_t adc_len = mode_is_buffered(MODE) ? ADC_RING_LEN : block_size_for(MODE);

    if (tx_active)
    {
        HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t *)dac_buffer, dac_len, DAC_ALIGN_12B_R);
        if (MODE == MODE_CW)
            HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)sidetone_buffer, dac_len, DAC_ALIGN_12B_R);
        else
        {
            HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
            HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_MID);
        }
        HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_buffer, adc_len);
    }
    else
    {
        HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)dac_buffer, dac_len, DAC_ALIGN_12B_R);
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, adc_len);
    }
}

static void audio_dma_restart(void)
{
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_2);
    audio_dma_start();
}

/* Does the real work of switching to `mode`. Split from radio_set_mode()
   so main() can run this exact sequence unconditionally at boot. */
static void radio_apply_mode(int mode)
{
    uint32_t want = block_size_for(mode);

    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_2);

    MODE       = mode;
    block_size = want;

    dsp_and_dma_flush();

    if (mode_is_freedv(mode))
        freedv_ok = (freedv_chain_init(chain_mode_for(mode)) == 0);
    else if (freedv_ok)
        freedv_chain_reset();

    audio_dma_start();
}

void radio_set_mode(int mode)
{
    uint32_t want = block_size_for(mode);
    if (mode == MODE && want == block_size)
        return;
    radio_apply_mode(mode);
}

int radio_tx_on(void)
{
    if (!tx_supported(MODE))
        return 0;

    if (MODE == MODE_CW) cw_tx_restart();
    else                 ssb_tx_restart();   /* also inits the AM carrier osc */

    if (mode_is_freedv(MODE) && freedv_ok) freedv_chain_set_tx(1);

    tx_rearm  = 1;
    tx_active = 1;
    dsp_and_dma_flush();
    audio_dma_restart();
    /* No TX indicator LED wired up in this .ioc yet (F746 used LD3, not
       present here) - add one once the pin exists. */
    return 1;
}

void radio_tx_off(void)
{
    tx_active = 0;
    if (mode_is_freedv(MODE) && freedv_ok) freedv_chain_set_tx(0);
    dsp_and_dma_flush();
    audio_dma_restart();
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* Clear the clock-ready flag (see Boot_Mode_Sequence_2 below) as the very
     first thing, before anything else - SRAM4/D3 isn't zero-inited on
     reset like .bss is, so a stale "1" left over from a previous run
     (soft reset, not a full power cycle) would otherwise let CM4 skip
     its wait entirely and race ahead on the pre-config clock again. This
     runs far enough ahead of CM4 reaching its check (which needs
     HAL_Init() + MX_DMA_Init() + MX_GPIO_Init() first) that the two
     never race in practice. */
  *(volatile uint32_t *)(D3_SRAM_BASE + 0x3000UL) = 0;

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Harmless forward-looking insurance in case CM7 ever uses a low-power
     mode of its own - not tied to any bug fix. (An earlier hypothesis
     during bring-up blamed the SWD lockup on the D2-domain equivalent of
     this bit; the real cause turned out to be a board/firmware
     SMPS-vs-LDO power-supply mismatch, see HAL_PWREx_ConfigSupply()
     below. The dual-core boot sync that would have needed the D2 bit is
     now disabled anyway - see DUAL_CORE_BOOT_SYNC_SEQUENCE above.) */
  SET_BIT(DBGMCU->CR, DBGMCU_CR_DBG_STOPD1 | DBGMCU_CR_DBG_STANDBYD1 | DBGMCU_CR_DBG_SLEEPD1);

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
  /* Tell CM4 the shared clock tree (D2PCLK1, which its I2C1 depends on)
     is finally configured - CM4 busy-waits on this before touching I2C1/
     the OLED, now that the old Stop-mode-based boot sync (which gave the
     same guarantee) is disabled. A plain polled flag rather than HSEM +
     STOP mode: no low-power state for CM4 to fall asleep in, so nothing
     for a debugger to race against on reset. Safe to use before caches
     are enabled (SCB_EnableICache() is later, doesn't affect D3/SRAM4
     reads by CM4 either way) and before ipc_mailbox_init()'s MPU setup -
     this is a single word write/read, not a structured shared buffer. */
  *(volatile uint32_t *)(D3_SRAM_BASE + 0x3000UL) = 1;

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* Moved here from before HAL_Init() (this port's first attempt) - that
     ran ahead of the CubeMX-generated dual-core HSEM boot sync above,
     which is timing-sensitive, and on first hardware test the target's
     SWD port became unreachable after reset. The actual root cause turned
     out to be a board/firmware power-supply mismatch (see
     HAL_PWREx_ConfigSupply above), not this placement - but there's no
     reason to move it back, since nothing here needs to run before the
     sync completes. */
  ipc_mailbox_init();

  /* Flash runs with wait states at this clock, so without the instruction
     cache every fetch stalls. I-cache costs nothing in correctness and is
     worth a lot on this DSP-heavy path. */
  SCB_EnableICache();

  /* D-Cache was left disabled through earlier bring-up because the ADC1/
     ADC2/DAC1 DMA channels read/write adc_buffer/dac_buffer/
     sidetone_buffer directly and aren't cache-coherent with the CPU - with
     D-Cache on, the CPU could read stale cached ADC samples, or DMA could
     push out stale cached DAC values written by the CPU. Measured on real
     hardware (console `status`), that left FreeDV 700D's process_block()
     at ~200% of its real-time budget even at -O2, never reaching sync -
     every RAM access from codec2/OFDM's FFT- and table-heavy work was
     paying full AXI latency instead of hitting cache.
     Fix: those three buffers now live in their own linker section
     (.dma_buffers, stm32h755xx_flash_CM7.ld), a single 32KB/32KB-aligned
     block sized to hold all three. One MPU region marks exactly that
     block as non-cacheable/shareable (same pattern as ipc_mailbox.c's
     region for the D3 SRAM4 IPC struct, which is shared with CM4 for the
     same reason) so the rest of AXI SRAM - all of codec2/DSP's working
     buffers and tables - can be cached. */
  {
    extern uint32_t __dma_buffers_start;
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable();

    MPU_InitStruct.Enable           = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress      = (uint32_t)&__dma_buffers_start;
    MPU_InitStruct.Size             = MPU_REGION_SIZE_32KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable      = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.Number           = MPU_REGION_NUMBER0;   /* Number1 used by ipc_mailbox.c */
    MPU_InitStruct.TypeExtField     = MPU_TEX_LEVEL0;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;   /* data only */

    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
  }

  SCB_EnableDCache();

  HAL_OPAMP_Start(&hopamp1);

  radio_apply_mode(MODE);

  HAL_TIM_Base_Start(&htim2);

  cw_paddle_gpio_init();

  /* rx_cycles_max/rx_fdv_cycles_max (process_block()'s DWT->CYCCNT deltas,
     rx_chain.c) are meaningless until the cycle counter is actually
     running - CubeMX never enables it by default. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Live CW key/paddle: session start/stop only (PTT-equivalent) - the
       actual element timing happens at sample rate inside
       cw_tx_process_block(), not here. */
    cw_keyer_poll();

    /* Control changes from the HMI (CM4) - mode, PTT, CW pitch/WPM/keyer
       type. Polled at main-loop rate, not audio-block rate - this is
       control-plane only, same cadence hmi_poll() ran at on F746. */
    {
        static ipc_cm4_to_cm7_t last_cmd;
        ipc_cm4_to_cm7_t cmd;
        ipc_cm4_to_cm7_read(&cmd);

        if (cmd.mode != last_cmd.mode)
            radio_set_mode(cmd.mode);

        if (cmd.cw_pitch_hz != last_cmd.cw_pitch_hz)
            cw_set_pitch(cmd.cw_pitch_hz);
        if (cmd.cw_wpm != last_cmd.cw_wpm)
            cw_set_wpm(cmd.cw_wpm);
        if (cmd.cw_keyer_type != last_cmd.cw_keyer_type)
            cw_keyer_type_set(cmd.cw_keyer_type);
        if (cmd.mic_gain != last_cmd.mic_gain)
            mic_gain = cmd.mic_gain;

        /* CW's own live key/paddle (cw_keyer_poll() above) starts/stops TX
           itself, independent of this ptt field - only apply it for the
           voice modes. */
        if (cmd.ptt != last_cmd.ptt && MODE != MODE_CW)
        {
            if (cmd.ptt) radio_tx_on();
            else         radio_tx_off();
        }

        last_cmd = cmd;
    }

    if (tx_active)
    {
      uint32_t n = block_size / 2;

      if (mode_is_buffered(MODE))
      {
        /* FreeDV TX: decoupled from DAC timing. Mic follows the ADC ring's
           write pointer; the encode (up to ~53 ms for 700E) runs here in
           the main loop; the DAC is a deep circular buffer the DMA plays
           through while we fill ahead of its read pointer. */
        static uint16_t tx_mic[BLOCK_SIZE_MAX / 2];
        static uint32_t tx_rd;
        static uint32_t dac_wr;

        if (tx_rearm)
        {
          tx_rd    = (adc_dma_pos() + ADC_RING_LEN - 2 * n) % ADC_RING_LEN;
          dac_wr   = 0;
          tx_rearm = 0;
        }

        while (((adc_dma_pos() + ADC_RING_LEN - tx_rd) % ADC_RING_LEN) >= n)
        {
          for (uint32_t i = 0; i < n; i++)
            tx_mic[i] = adc_buffer[(tx_rd + i) % ADC_RING_LEN];

          tx_rd = (tx_rd + n) % ADC_RING_LEN;
          freedv_tx_feed(tx_mic, n);
        }

        freedv_chain_encode();

        {
          uint32_t play  = dac_dma_pos();
          uint32_t ahead = (dac_wr + DAC_RING_LEN - play) % DAC_RING_LEN;

          if (ahead < tx_cushion_min) tx_cushion_min = ahead;

          while (ahead <= DAC_RING_LEN - 2 * n &&
                 freedv_chain_modem_avail48() >= (int)n)
          {
            freedv_tx_produce(&dac_buffer[dac_wr], n);
            dac_wr = (dac_wr + n) % DAC_RING_LEN;
            ahead += n;
          }
        }
      }
      else if (dac_block_processing == 1)
      {
          tx_process_block(&adc_buffer[0], &dac_buffer[0], &sidetone_buffer[0], n);
          dac_block_processing = 0;
      }
      else if (dac_block_processing == 2)
      {
          tx_process_block(&adc_buffer[n], &dac_buffer[n], &sidetone_buffer[n], n);
          dac_block_processing = 0;
      }
    }
    else if (mode_is_buffered(MODE))
    {
      static int dac_half = 0;
      uint32_t n = block_size / 2;
      int      guard = 4;

      /* Diagnostic: how long between successive passes through this check.
         If the main loop is genuinely as tight as it looks (nothing
         blocking above this point), consecutive passes should be far
         faster than a block period (n/48000 s) whenever there's no backlog
         to drain - a slow poll_gap_cycles_max here, comparable to or
         exceeding a block period, would mean something upstream in the
         loop is occasionally stalling it, independent of process_block()'s
         own cost (which rx_cycles_max already accounts for separately). */
      {
          static uint32_t last_poll_cycle;
          uint32_t        now_cycle = DWT->CYCCNT;
          uint32_t        gap       = now_cycle - last_poll_cycle;
          if (last_poll_cycle != 0 && gap > poll_gap_cycles_max)
              poll_gap_cycles_max = gap;
          last_poll_cycle = now_cycle;
      }

      if (adc_avail() > ADC_RING_LEN - n)
      {
        rx_overruns++;
        adc_rd = (adc_dma_pos() / n * n) % ADC_RING_LEN;
      }

      while (adc_avail() >= n && guard--)
      {
        uint32_t play = block_size - __HAL_DMA_GET_COUNTER(&hdma_dac1_ch1);
        int dac_half_free = dac_half ? (play < n) : (play >= n);
        if (!dac_half_free)
          break;

        process_block(&adc_buffer[adc_rd], &dac_buffer[dac_half ? n : 0], n);
        adc_rd = (adc_rd + n) % ADC_RING_LEN;
        dac_half ^= 1;
      }
    }
    else if (block_ready == 1)
    {
      process_block(&adc_buffer[0], &dac_buffer[0], block_size / 2);
      block_ready = 0;
    }
    else if (block_ready == 2)
    {
      process_block(&adc_buffer[block_size / 2], &dac_buffer[block_size / 2], block_size / 2);
      block_ready = 0;
    }

    /* Telemetry to CM4's HMI - S-meter, RX/TX state - at a UI-relevant
       rate, not audio-block rate. */
    {
        static uint32_t last_status_tick = 0;
        uint32_t now = HAL_GetTick();
        if (now - last_status_tick >= 50)
        {
            last_status_tick = now;
            ipc_cm7_to_cm4_t status;
            status.rx_rms         = rx_rms;
            status.tx_active      = tx_active;
            status.mode           = MODE;
            status.freedv_synced  = freedv_ok ? freedv_chain_synced() : 0;
            status.freedv_snr_db  = freedv_ok ? freedv_chain_snr() : 0.0f;
            status.freedv_foff_hz     = freedv_ok ? freedv_chain_foff() : 0.0f;
            status.freedv_sync_metric = freedv_ok ? freedv_chain_sync_metric() : 0.0f;
            status.freedv_clock_ppm   = freedv_ok ? freedv_chain_clock_offset() : 0.0f;

            /* process_block() runs on n = block_size_for(MODE)/2 samples
               per call (see the block_ready dispatch below) at 48 kHz -
               that's its real-time budget in cycles. */
            {
                float budget_cycles = ((float)block_size_for(MODE) / 2.0f)
                                       / 48000.0f * (float)SystemCoreClock;
                status.rx_load_pct     = 100.0f * (float)rx_cycles_max     / budget_cycles;
                status.rx_fdv_load_pct = 100.0f * (float)rx_fdv_cycles_max / budget_cycles;
            }

            status.rx_overruns      = rx_overruns;
            status.freedv_underruns = freedv_ok ? freedv_chain_underruns() : 0;
            status.clock_src_khz    = clock_src_hse ? HSE_SRC_KHZ : 0;
            status.poll_gap_us      = (float)poll_gap_cycles_max / ((float)SystemCoreClock / 1.0e6f);
            if (freedv_ok)
            {
                freedv_chain_nin_stats(&status.freedv_nin_min, &status.freedv_nin_max, &status.freedv_nin_avg);
                status.freedv_in_drops = freedv_chain_in_drops();
            }
            else
            {
                status.freedv_nin_min = 0;
                status.freedv_nin_max = 0;
                status.freedv_nin_avg = 0;
                status.freedv_in_drops = 0;
            }

            ipc_cm7_to_cm4_write(&status);
        }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 480;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 16;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 75;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief OPAMP1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP1_Init(void)
{

  /* USER CODE BEGIN OPAMP1_Init 0 */

  /* USER CODE END OPAMP1_Init 0 */

  /* USER CODE BEGIN OPAMP1_Init 1 */

  /* USER CODE END OPAMP1_Init 1 */
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.Mode = OPAMP_FOLLOWER_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP1_Init 2 */

  /* USER CODE END OPAMP1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 10-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 500-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
