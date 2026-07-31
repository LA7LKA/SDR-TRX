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
#include "string.h"
#include <stdlib.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f7xx.h"      // <-- MÅ være først
#include "arm_math.h"       // <-- CMSIS-DSP
#include "dsp.h"            // <-- dine DSP-typer
#include "freedv_chain.h"   // <-- FreeDV 1600 RX-kjede
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TWO_PI 6.28318530718f
/*
 * Block size is per mode, so the buffers are allocated for the largest case and
 * the DMA is restarted with the size the current mode wants.
 *
 * FreeDV: half a buffer is 1920 samples = 40 ms, and 1920/6 = 320, which is
 * exactly one FDMDV frame. That makes every block cost the same instead of
 * alternating cheap/expensive, which is what it takes to stay real time.
 *
 * FreeDV 2400B: 96 bits at 4800 baud, so 1920 samples at 48 kHz, the same 40 ms
 * as 1600. Its modem is already at 48 kHz, so unlike 1600 there is no
 * resampling on the way in.
 *
 * Analog: 256 samples = 5.3 ms. SSB, AM and NBFM do not care much, but CW
 * break-in does, and 40 ms of round trip delay is very audible on a key.
 */
#define BLOCK_SIZE_MAX     3840
#define BLOCK_SIZE_FREEDV  3840
#define BLOCK_SIZE_2400B   3840
#define BLOCK_SIZE_700D    3840
#define BLOCK_SIZE_700E    3840
#define BLOCK_SIZE_ANALOG   512

// Active size, swapped by radio_set_mode(). Buffers stay BLOCK_SIZE_MAX.
static volatile uint32_t block_size = BLOCK_SIZE_FREEDV;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2004c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2004c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2004c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2004c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ADC_HandleTypeDef hadc1;   /* RX: IF in, PC0 */
DMA_HandleTypeDef hdma_adc1;

ADC_HandleTypeDef hadc2;   /* TX: mic in, PA3 */
DMA_HandleTypeDef hdma_adc2;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;   /* RX: audio out, PA4 (DAC_OUT1) */
DMA_HandleTypeDef hdma_dac2;   /* TX: IF out, PA5 (DAC_OUT2) */

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/*
 * The ADC DMA buffer doubles as the input FIFO for the buffered modes, so it
 * is deeper than one block: five blocks, i.e. 200 ms at 48 kHz. The DMA writes
 * into it continuously and the DSP reads behind, which decouples processing
 * time from the block period without a second copy of the data. Halfword
 * samples because the ADC is 12-bit.
 */
#define ADC_RING_BLOCKS 4
#define ADC_RING_LEN    (ADC_RING_BLOCKS * (BLOCK_SIZE_MAX / 2))

/*
 * Transmit DAC buffer, three blocks deep. In the buffered (FreeDV) modes the
 * DMA plays through it continuously while the main loop encodes and fills
 * ahead, so a 53 ms OFDM encode draws down the cushion instead of stalling the
 * output. The analog modes keep using the first block_size of it as a simple
 * double buffer.
 */
#define DAC_RING_BLOCKS 4
#define DAC_RING_LEN    (DAC_RING_BLOCKS * (BLOCK_SIZE_MAX / 2))

uint16_t dac_buffer[DAC_RING_LEN] = {0};
uint16_t adc_buffer[ADC_RING_LEN] = {0};

volatile uint8_t block_ready = 0; // 0: no block ready, 1: first half ready, 2: second half ready
volatile uint8_t dac_block_processing = 0; // 0: not processing, 1: processing

int freedv_ok = 0;          // set once freedv_chain_init() has succeeded

// RX telemetry, printed once a second over the ST-Link VCP
volatile uint32_t rx_blocks = 0;   // blocks through process_block()
volatile float    rx_peak   = 0.0f; // peak |audio| at the modem input
volatile float    rx_rms    = 0.0f; // rms of the same, to judge peak/rms
volatile float    rx_env_min = 0.0f; // AM: smallest envelope value in the block
volatile float    rx_env_avg = 0.0f; // AM: mean envelope, i.e. the carrier level
volatile float    rx_adc_peak = 0.0f; // peak |ADC| before any filtering
volatile uint32_t rx_cycles_max = 0;  // worst-case cycles for one block
volatile uint32_t rx_fdv_cycles_max = 0; // worst-case cycles inside the FreeDV chain
volatile uint32_t rx_adc_cycles = 0;     // TX: worst-case encode cycles
volatile uint32_t tx_cushion_min = 999999; // TX: smallest DAC lead, in samples
static   uint32_t last_report = 0;
static   int      debug_on    = 0;   // continuous telemetry print, off by default

// DWT cycle counter, used to measure how much of each block period the DSP
// actually consumes. One block is block_size/2 samples at 48 kHz.
static void cyclecount_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Cortex-M7 gates the DWT registers behind the lock access register; without
    // this CYCCNT stays at 0 and every measurement reads back as zero.
    DWT->LAR = 0xC5ACCE55;

    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}
nco_t nco; 


// NCO for IF-mixing
nco_t nco_if;



// Pre-demod IF-filter (kompleks, men vi bruker samme koeff for I og Q)
arm_fir_instance_f32 pre_demod_filter;


/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 48000 Hz

* 0 Hz - 4000 Hz
  gain = 1
  desired ripple = 0.1 dB
  actual ripple = 0.06811726811384558 dB

* 6000 Hz - 24000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = -60.68801081580469 dB

*/

#define PRE_DEMOD_TAPS 73

static float pre_demod_coeffs[PRE_DEMOD_TAPS] = {
  -0.0004930375303321366,
  0.0001064813438661428,
  0.0005748457570259578,
  0.0011898445128213748,
  0.0016502280581233504,
  0.001605207031383133,
  0.000831446560674343,
  -0.0005973708354363059,
  -0.002228627696676951,
  -0.0033377455060532744,
  -0.0032076016149152073,
  -0.0014978471407907723,
  0.001459284866735388,
  0.004620535335059582,
  0.00653528877009168,
  0.0059370991846072535,
  0.00240691382833557,
  -0.003197318174051976,
  -0.008810233649088216,
  -0.0118211865758766,
  -0.010154929160148466,
  -0.003367069515417546,
  0.006744477803887592,
  0.016400388788851774,
  0.021048898446146783,
  0.017199001020187076,
  0.0042202354750242735,
  -0.014651068016007818,
  -0.032657469215897225,
  -0.04131750763191529,
  -0.03320662673344374,
  -0.004808839248635039,
  0.04161063532888555,
  0.09790380688954856,
  0.1519161665559878,
  0.1908448312364271,
  0.20501884795915196,
  0.1908448312364271,
  0.1519161665559878,
  0.09790380688954856,
  0.04161063532888555,
  -0.004808839248635039,
  -0.03320662673344374,
  -0.04131750763191529,
  -0.032657469215897225,
  -0.014651068016007818,
  0.0042202354750242735,
  0.017199001020187076,
  0.021048898446146783,
  0.016400388788851774,
  0.006744477803887592,
  -0.003367069515417546,
  -0.010154929160148466,
  -0.0118211865758766,
  -0.008810233649088216,
  -0.003197318174051976,
  0.00240691382833557,
  0.0059370991846072535,
  0.00653528877009168,
  0.004620535335059582,
  0.001459284866735388,
  -0.0014978471407907723,
  -0.0032076016149152073,
  -0.0033377455060532744,
  -0.002228627696676951,
  -0.0005973708354363059,
  0.000831446560674343,
  0.001605207031383133,
  0.0016502280581233504,
  0.0011898445128213748,
  0.0005748457570259578,
  0.0001064813438661428,
  -0.0004930375303321366
};







/*
 * Scratch shared by ssb_process_block() and nbfm_process_block().
 *
 * process_block() dispatches to exactly one of them, so they are never live at
 * the same time and there is no reason for each to hold its own statics. Both
 * also used to over-allocate: the interleaved buffers only ever index up to
 * 2*n, and the real buffers up to n, where n = BLOCK_SIZE/2.
 *
 * Layout is 10*n floats, which is what nbfm needs (4 real + 3 interleaved);
 * ssb uses 9*n of it. Keeping these separate is what made BLOCK_SIZE 3840
 * overflow RAM.
 */
#define SCRATCH_N (BLOCK_SIZE_MAX / 2)
static float dsp_scratch[10 * SCRATCH_N];

/*
 * CW audio filter: three identical biquad band-pass sections at the tone pitch.
 *
 * Biquads rather than a FIR because pitch and bandwidth are meant to become
 * front panel controls. Retuning this is five coefficients; a FIR with the
 * same skirts would be hundreds of taps to redesign on every turn of a knob.
 *
 * Each section is deliberately wider than the target: cascading three narrows
 * the result, so Q 1.42 per section gives about 250 Hz overall. Going much
 * below that starts to ring and smears the elements at speed, which reads
 * worse than a wider filter.
 */
#define CW_PITCH_HZ   700.0f
#define CW_STAGES     3

static const float cw_coeffs[5 * CW_STAGES] = {
    +3.1213224677e-02f,
    +0.0000000000e+00f,
    -3.1213224677e-02f,
    +1.9294452893e+00f,
    -9.3757355065e-01f,
    +3.1213224677e-02f,
    +0.0000000000e+00f,
    -3.1213224677e-02f,
    +1.9294452893e+00f,
    -9.3757355065e-01f,
    +3.1213224677e-02f,
    +0.0000000000e+00f,
    -3.1213224677e-02f,
    +1.9294452893e+00f,
    -9.3757355065e-01f
};

arm_biquad_casd_df1_inst_f32 cw_bpf;
static float cw_state[4 * CW_STAGES];

arm_fir_instance_f32 pre_fm_I;
arm_fir_instance_f32 pre_fm_Q;

static float pre_fm_state_I[PRE_DEMOD_TAPS + BLOCK_SIZE_MAX/2];
static float pre_fm_state_Q[PRE_DEMOD_TAPS + BLOCK_SIZE_MAX/2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ETH_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * USART3 is wired to the ST-Link virtual COM port (/dev/ttyACM0), 115200 8N1.
 *
 * Queued rather than sent with HAL_UART_Transmit, which blocks: a 123 character
 * telemetry line takes 10.7 ms at 115200, and the analog block period is
 * 5.3 ms, so every report used to cost two blocks of audio. Here uart_puts()
 * only appends and uart_pump() moves one byte per main loop pass, which is
 * ample when the loop turns over hundreds of thousands of times a second.
 */
#define TX_RING 512   /* power of two */

static char              tx_ring[TX_RING];
static volatile uint16_t tx_wr, tx_rd;

static void uart_pump(void)
{
    if (tx_rd != tx_wr && (huart3.Instance->ISR & USART_ISR_TXE))
        huart3.Instance->TDR = tx_ring[tx_rd++ & (TX_RING - 1)];
}

void uart_puts(const char *s)
{
    while (*s)
    {
        /* Only ever blocks if the queue backs up, which startup can do. */
        while ((uint16_t)(tx_wr - tx_rd) >= TX_RING)
            uart_pump();

        tx_ring[tx_wr++ & (TX_RING - 1)] = *s++;
    }
}

// Integer-only formatting on purpose: printf("%f") needs -u _printf_float,
// which nano.specs leaves out.
static void uart_kv(const char *key, int value)
{
    char buf[48];
    char *p = buf;
    int   n = value;
    int   digits = 0;
    char  tmp[12];

    while (*key)
        *p++ = *key++;
    *p++ = '=';

    if (n < 0) { *p++ = '-'; n = -n; }
    do { tmp[digits++] = '0' + (n % 10); n /= 10; } while (n);
    while (digits)
        *p++ = tmp[--digits];

    *p++ = ' ';
    *p   = '\0';

    uart_puts(buf);
}

// If block_ready is still set when the next half completes, the main loop did
// not keep up and a block of samples is lost. FreeDV cannot tolerate that: the
// modem needs an unbroken sample stream to hold sync.
volatile uint32_t rx_overruns = 0;

/*
 * Read position into the ADC ring. The DMA controller is the writer and its
 * position is the transfer counter, so nothing has to be tracked in an ISR.
 * The ring is a whole number of blocks, so a block read never wraps.
 *
 * Half duplex, so exactly one ADC and one DAC channel is ever running:
 * hadc1/DAC_OUT1 (IF in / audio out) on RX, hadc2/DAC_OUT2 (mic in / IF out)
 * on TX. tx_active picks which pair's DMA position is live.
 */
extern volatile int tx_active;

static uint32_t adc_rd;

static uint32_t adc_dma_pos(void)
{
    ADC_HandleTypeDef *adc = tx_active ? &hadc2 : &hadc1;
    return ADC_RING_LEN - __HAL_DMA_GET_COUNTER(adc->DMA_Handle);
}

static uint32_t dac_dma_pos(void)
{
    DMA_HandleTypeDef *dma = tx_active ? hdac.DMA_Handle2 : hdac.DMA_Handle1;
    return DAC_RING_LEN - __HAL_DMA_GET_COUNTER(dma);
}

static uint32_t adc_avail(void)
{
    uint32_t w = adc_dma_pos();
    return (w >= adc_rd) ? (w - adc_rd) : (ADC_RING_LEN - adc_rd + w);
}

extern int MODE;                 /* defined further down, with the mode list */
int mode_is_buffered(int mode);

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (mode_is_buffered(MODE)) return;   /* the DSP reads the ring directly */

    if (block_ready) rx_overruns++;
    block_ready = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (mode_is_buffered(MODE)) return;   /* the DSP reads the ring directly */

    if (block_ready) rx_overruns++;
    block_ready = 2;
}



void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    // DAC half conversion complete callback (channel 1)
    // You can add code here if needed
    dac_block_processing = 1;
}


void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    // DAC conversion complete callback (channel 1)
    // You can add code here if needed
    dac_block_processing = 2;
}

/*
 * Channel 2 (TX IF out) mirrors channel 1: dac_block_processing is what
 * paces tx_process_block() in the main loop, and on TX it is channel 2 that
 * is actually running the DMA, not channel 1.
 */
void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
    dac_block_processing = 1;
}

void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
    dac_block_processing = 2;
}


/*
//Hilbert filter for SSB
#define HILBERT_TAPS 63

static float hilbert_coeffs[HILBERT_TAPS] = {
    -0.0041, -0.0037, -0.0028, -0.0014, 0.0004, 0.0025, 0.0047,
    0.0067, 0.0082, 0.0089, 0.0086, 0.0071, 0.0044, 0.0007,
    -0.0034, -0.0076, -0.0114, -0.0143, -0.0158, -0.0156,
    -0.0135, -0.0096, -0.0043, 0.0020, 0.0087, 0.0151,
    0.0204, 0.0239, 0.0251, 0.0239, 0.0204, 0.0151,
    0.0087, 0.0020, -0.0043, -0.0096, -0.0135, -0.0156,
    -0.0158, -0.0143, -0.0114, -0.0076, -0.0034, 0.0007,
    0.0044, 0.0071, 0.0086, 0.0089, 0.0082, 0.0067,
    0.0047, 0.0025, 0.0004, -0.0014, -0.0028, -0.0037,
    -0.0041
};
*/

//Hilbert filter for NBFM
/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 48000 Hz

* 0 Hz - 6000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = -50.41488654693752 dB

* 8000 Hz - 16000 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 15.449541453020096 dB

* 18000 Hz - 24000 Hz
  gain = 0
  desired attenuation = -60 dB
  actual attenuation = -50.41488654693752 dB

*/

#define HILBERT_TAPS 17

static float hilbert_coeffs[HILBERT_TAPS] = {
  0.02352204878641468,
  -7.964549159078516e-16,
  -0.094806149454076,
  5.916427478655016e-16,
  0.21352320600109478,
  -7.4996177568773785e-16,
  -0.3325693585500034,
  4.3317065620714974e-17,
  0.3830170947566392,
  4.3317065620714974e-17,
  -0.3325693585500034,
  -7.4996177568773785e-16,
  0.21352320600109478,
  5.916427478655016e-16,
  -0.094806149454076,
  -7.964549159078516e-16,
  0.02352204878641468
};



static float hilbert_state[HILBERT_TAPS + BLOCK_SIZE_MAX/2];
arm_fir_instance_f32 hilbert;


#define IF_TAPS 101

static float if_coeffs[101] = {
    -0.0003, -0.0004, -0.0005, -0.0006, -0.0007, -0.0008,
    -0.0009, -0.0010, -0.0011, -0.0012, -0.0013, -0.0014,
    -0.0015, -0.0016, -0.0017, -0.0018, -0.0019, -0.0020,
    -0.0021, -0.0022, -0.0023, -0.0024, -0.0025, -0.0026,
    -0.0027, -0.0028, -0.0029, -0.0030, -0.0031, -0.0032,
    -0.0033, -0.0034, -0.0035, -0.0036, -0.0037, -0.0038,
    -0.0039, -0.0040, -0.0041, -0.0042, -0.0043, -0.0044,
    -0.0045, -0.0046, -0.0047, -0.0048, -0.0049, -0.0050,
    0.9950,
    -0.0050, -0.0049, -0.0048, -0.0047, -0.0046, -0.0045,
    -0.0044, -0.0043, -0.0042, -0.0041, -0.0040, -0.0039,
    -0.0038, -0.0037, -0.0036, -0.0035, -0.0034, -0.0033,
    -0.0032, -0.0031, -0.0030, -0.0029, -0.0028, -0.0027,
    -0.0026, -0.0025, -0.0024, -0.0023, -0.0022, -0.0021,
    -0.0020, -0.0019, -0.0018, -0.0017, -0.0016, -0.0015,
    -0.0014, -0.0013, -0.0012, -0.0011, -0.0010, -0.0009,
    -0.0008, -0.0007, -0.0006, -0.0005, -0.0004, -0.0003
};




static float if_state[IF_TAPS + BLOCK_SIZE_MAX/2];
arm_fir_instance_f32 pre_demod_filter;

#define AUDIO_TAPS 63

static float audio_lpf_coeffs[AUDIO_TAPS] = {
    // 3 kHz LPF, 48 kHz fs, Hamming window
    -0.0007, -0.0010, -0.0013, -0.0016, -0.0019, -0.0022,
    -0.0025, -0.0028, -0.0031, -0.0034, -0.0037, -0.0040,
    -0.0043, -0.0046, -0.0049, -0.0052, -0.0055, -0.0058,
    -0.0061, -0.0064, -0.0067, -0.0070, -0.0073, -0.0076,
    -0.0079, -0.0082, -0.0085, -0.0088, -0.0091, -0.0094,
    0.9900,
    -0.0094, -0.0091, -0.0088, -0.0085, -0.0082, -0.0079,
    -0.0076, -0.0073, -0.0070, -0.0067, -0.0064, -0.0061,
    -0.0058, -0.0055, -0.0052, -0.0049, -0.0046, -0.0043,
    -0.0040, -0.0037, -0.0034, -0.0031, -0.0028, -0.0025,
    -0.0022, -0.0019, -0.0016, -0.0013, -0.0010, -0.0007
};

static float audio_state[AUDIO_TAPS + BLOCK_SIZE_ANALOG/2];
arm_fir_instance_f32 audio_lpf;


#define MODE_NBFM 0
#define MODE_USB  1
#define MODE_LSB  2
#define MODE_FREEDV 3   // FreeDV 1600, received as USB
#define MODE_FREEDV_2400B 4  // FreeDV 2400B, through a normal FM audio path
#define MODE_FREEDV_700D  5  // FreeDV 700D, OFDM + LDPC on SSB for weak signals
#define MODE_FREEDV_700E  6  // FreeDV 700E, shorter frame than 700D, faster reacquire
#define MODE_AM           7  // AM, envelope detection off the complex baseband
#define MODE_CW           8  // CW, SSB demod into a narrow filter at the tone pitch

int MODE = MODE_USB;  // default mode

/*
 * Fill an interleaved cos/sin buffer for the IF mixer.
 *
 * Recursive phasor rotation rather than cosf()/sinf() per sample: two libm
 * calls per sample was one of the larger costs in the block. Magnitude drifts
 * slowly, so it is renormalised once per call instead of per sample.
 */
static void nco_block_iq(float *NCO_buf, int n)
{
    static float osc_re = 1.0f, osc_im = 0.0f;
    static float step_re = 1.0f, step_im = 0.0f;
    static float step_dphi = 0.0f;

    if (step_dphi != nco.dphi)
    {
        step_dphi = nco.dphi;
        step_re   = cosf(nco.dphi);
        step_im   = sinf(nco.dphi);
    }

    for (int i = 0; i < n; i++)
    {
        NCO_buf[2*i + 0] = osc_re;
        NCO_buf[2*i + 1] = osc_im;

        float nre = osc_re * step_re - osc_im * step_im;
        float nim = osc_re * step_im + osc_im * step_re;

        osc_re = nre;
        osc_im = nim;
    }

    float mag2 = osc_re * osc_re + osc_im * osc_im;
    float g    = 1.5f - 0.5f * mag2;   // one Newton step toward 1/sqrt(mag2)

    osc_re *= g;
    osc_im *= g;
}

void ssb_process_block(const uint16_t *in, uint16_t *out, int n)
{
    // Carved out of the shared scratch pool; see dsp_scratch above.
    float *I_buf     = &dsp_scratch[0 * SCRATCH_N];
    float *Q_buf     = &dsp_scratch[1 * SCRATCH_N];
    float *audio_buf = &dsp_scratch[2 * SCRATCH_N];

    float *IQ_in     = &dsp_scratch[3 * SCRATCH_N];   // interleaved I/Q,   2n
    float *NCO_buf   = &dsp_scratch[5 * SCRATCH_N];   // interleaved cos/sin, 2n
    float *IQ_mix    = &dsp_scratch[7 * SCRATCH_N];   // interleaved I/Q,   2n

    // ---------------------------------------------------------
    // 1) ADC → normalisert I
    // ---------------------------------------------------------
    float adc_peak = 0.0f;

    for (int i = 0; i < n; i++)
    {
        uint32_t raw = in[i] & 0x0FFF;
        I_buf[i] = ((float)raw - 2048.0f) / 2048.0f;

        float a = fabsf(I_buf[i]);
        if (a > adc_peak) adc_peak = a;
    }

    rx_adc_peak = adc_peak;

    // ---------------------------------------------------------
    // 2) Hilbert → Q (for SSB og FM)
    // ---------------------------------------------------------
    arm_fir_f32(&hilbert, I_buf, Q_buf, n);

    // ---------------------------------------------------------
    // 3) Lag interleaved I/Q
    // ---------------------------------------------------------
    for (int i = 0; i < n; i++)
    {
        IQ_in[2*i + 0] = I_buf[i];
        IQ_in[2*i + 1] = Q_buf[i];
    }

    // ---------------------------------------------------------
    // 4) NCO → interleaved cos/sin
    // ---------------------------------------------------------
    nco_block_iq(NCO_buf, n);

    // ---------------------------------------------------------
    // 5) Kompleks mixing
    // ---------------------------------------------------------
    arm_cmplx_mult_cmplx_f32(IQ_in, NCO_buf, IQ_mix, n);

    // ---------------------------------------------------------
    // 6) Splitt ut I/Q etter mixing
    // ---------------------------------------------------------
    for (int i = 0; i < n; i++)
    {
        I_buf[i] = IQ_mix[2*i + 0];
        Q_buf[i] = IQ_mix[2*i + 1];
    }

    // ---------------------------------------------------------
    // 7) Pre-demod IF-filter
    // ---------------------------------------------------------
    arm_fir_f32(&pre_fm_I, I_buf, I_buf, n);
    arm_fir_f32(&pre_fm_Q, Q_buf, Q_buf, n);

    // ---------------------------------------------------------
    // 8) MODE-basert demod
    // ---------------------------------------------------------
    if(MODE == MODE_USB)
    {
        // USB = I - Q  (swapped to match how the sideband lands on the air
        // after the up-conversion, confirmed against an FT-857D)
        for (int i = 0; i < n; i++)
            audio_buf[i] = I_buf[i] - Q_buf[i];
    }
    else if (MODE == MODE_LSB)
    {
        // LSB = I + Q
        for (int i = 0; i < n; i++)
            audio_buf[i] = I_buf[i] + Q_buf[i];
    }
    else if (MODE == MODE_CW)
    {
        /*
         * CW is just SSB into a narrow filter. Tuning is what places the
         * carrier at the wanted pitch; the filter is centred there rather
         * than at zero, which is why the pitch is a system setting and not
         * simply a bandwidth.
         */
        static agc_t agc_cw = {1.0f};

        for (int i = 0; i < n; i++)
            audio_buf[i] = I_buf[i] + Q_buf[i];

        arm_biquad_cascade_df1_f32(&cw_bpf, audio_buf, audio_buf, n);

        float peak = 0.0f, sumsq = 0.0f;
        for (int i = 0; i < n; i++)
        {
            float a = fabsf(audio_buf[i]);
            if (a > peak) peak = a;
            sumsq += audio_buf[i] * audio_buf[i];
        }
        rx_peak = peak;
        rx_rms  = sqrtf(sumsq / (float)n);
        rx_blocks++;

        /* Slow decay so the gain does not wind up between elements. */
        agc_block_cfg(audio_buf, n, &agc_cw, 0.25f, 0.002f);
    }
    else if (MODE == MODE_FREEDV || MODE == MODE_FREEDV_700D
                                 || MODE == MODE_FREEDV_700E)
    {
        // FreeDV rides on an ordinary SSB signal, so demodulate as USB first.
        float peak = 0.0f;

        for (int i = 0; i < n; i++)
        {
            audio_buf[i] = I_buf[i] + Q_buf[i];

            float a = fabsf(audio_buf[i]);
            if (a > peak) peak = a;
        }

        rx_peak = peak;
        rx_blocks++;

        // 48k -> 8k -> FreeDV 1600 demod -> 8k -> 48k
        if (freedv_ok)
        {
            uint32_t t0 = DWT->CYCCNT;

            freedv_chain_put_audio48(audio_buf, n);
            freedv_chain_get_speech48(audio_buf, n);

            uint32_t dt = DWT->CYCCNT - t0;
            if (dt > rx_fdv_cycles_max)
                rx_fdv_cycles_max = dt;
        }
    }
    else
    {
        for (int i = 0; i < n; i++)
            audio_buf[i] = 0.0f;
    }

    // ---------------------------------------------------------
    // 9) Audio gain før DAC
    // ---------------------------------------------------------
    for (int i = 0; i < n; i++)
        audio_buf[i] *= 1500.0f;

    // ---------------------------------------------------------
    // 10) DAC output
    // ---------------------------------------------------------
    for (int i = 0; i < n; i++)
    {
        float y = audio_buf[i] + 2048.0f;

        if (y < 0.0f)    y = 0.0f;
        if (y > 4095.0f) y = 4095.0f;

        out[i] = (uint16_t)y;
    }
}

void nbfm_process_block(const uint16_t *in, uint16_t *out, int n)
{
    // Carved out of the shared scratch pool; see dsp_scratch above.
    float *I_if          = &dsp_scratch[0 * SCRATCH_N];
    float *Q_if          = &dsp_scratch[1 * SCRATCH_N];
    float *audio_fm      = &dsp_scratch[2 * SCRATCH_N];   // FM-demod audio
    float *audio_lpf_out = &dsp_scratch[3 * SCRATCH_N];

    float *IQ_in         = &dsp_scratch[4 * SCRATCH_N];   // interleaved I/Q,   2n
    float *NCO_buf       = &dsp_scratch[6 * SCRATCH_N];   // interleaved cos/sin, 2n
    float *IQ_mix        = &dsp_scratch[8 * SCRATCH_N];   // interleaved I/Q,   2n

    // -----------------------------
    // 1) ADC → normalisert IF (I)
    // -----------------------------
    for (int i = 0; i < n; i++)
    {
        uint32_t raw = in[i] & 0x0FFF;
        I_if[i] = ((float)raw - 2048.0f) / 2048.0f;
        //Q_if[i] = 0.0f;   // ren IF, ingen Hilbert her
    }


    arm_fir_f32(&hilbert, I_if, Q_if, n);

    // -----------------------------
    // 2) Lag interleaved I/Q
    // -----------------------------
    for (int i = 0; i < n; i++)
    {
        IQ_in[2*i + 0] = I_if[i];
        IQ_in[2*i + 1] = Q_if[i];
    }

    // -----------------------------
    // 3) NCO → interleaved cos/sin
    // -----------------------------
    for (int i = 0; i < n; i++)
    {
        float cs = cosf(nco_if.phase);
        float sn = sinf(nco_if.phase);
        nco_step(&nco_if);

        NCO_buf[2*i + 0] = cs;
        NCO_buf[2*i + 1] = sn;
    }

    // -----------------------------
    // 4) Kompleks mixing (IF → baseband FM)
    // -----------------------------
    arm_cmplx_mult_cmplx_f32(IQ_in, NCO_buf, IQ_mix, n);

    // -----------------------------
    // 5) Splitt ut I/Q etter mixing
    // -----------------------------
    for (int i = 0; i < n; i++)
    {
        I_if[i] = IQ_mix[2*i + 0];
        Q_if[i] = IQ_mix[2*i + 1];
    }

    // -----------------------------
    // 6) Pre-demod IF-filter (kompleks)
    // -----------------------------
    arm_fir_f32(&pre_fm_I, I_if, I_if, n);
    arm_fir_f32(&pre_fm_Q, Q_if, Q_if, n);

    // -----------------------------
    // 7) FM-demodulator (din nbfm_demod)
    // -----------------------------
    nbfm_demod(I_if, Q_if, audio_fm, n);

    // -----------------------------
    // 8) Audio LPF etter FM-demod
    // -----------------------------
    arm_fir_f32(&audio_lpf, audio_fm, audio_lpf_out, n);

    // -----------------------------
    // 9) Gain + DAC-skalering
    // -----------------------------
    for (int i = 0; i < n; i++)
    {
        float y = audio_lpf_out[i] * 1500.0f + 2048.0f;

        if (y < 0.0f)    y = 0.0f;
        if (y > 4095.0f) y = 4095.0f;

        out[i] = (uint16_t)y;
    }
}

extern float mic_gain;   /* defined with the SSB modulator below */

/*
 * Frequency modulate a ready audio buffer.
 *
 * Deviation is a parameter because voice and data want different values:
 * 5 kHz for speech, but 2.5 kHz for FreeDV 2400B, which is what its
 * demodulator expects and what stops the discriminator running into atan2's
 * wrap point at the far end.
 */
static void fm_modulate(const float *audio, uint16_t *out, int n, float dev_hz)
{
    static float phase = 0.0f;

    const float kf   = 2.0f * (float)M_PI * dev_hz  / 48000.0f;
    const float w_if = 2.0f * (float)M_PI * 12000.0f / 48000.0f;

    for (int i = 0; i < n; i++)
    {
        phase += w_if + kf * audio[i];

        if (phase >  (float)M_PI) phase -= TWO_PI;
        if (phase < -(float)M_PI) phase += TWO_PI;

        out[i] = (uint16_t)((cosf(phase) * 2048.0f) + 2048.0f);
    }
}

void nbfm_tx_process_block(const uint16_t *in, uint16_t *out, int n)
{
    /* Shared with the receive path: transmit and receive never run together. */
    float *audio_in       = &dsp_scratch[0 * SCRATCH_N];
    float *audio_filtered = &dsp_scratch[1 * SCRATCH_N];

    const float dev_hz = 5000.0f;

    float peak = 0.0f;

    for (int i = 0; i < n; i++)
    {
        float x = (((float)(in[i] & 0x0FFF)) - 2048.0f) / 2048.0f;

        float a = fabsf(x);
        if (a > peak) peak = a;

        audio_in[i] = x * mic_gain;
    }

    rx_adc_peak = peak;

    arm_fir_f32(&audio_lpf, audio_in, audio_filtered, n);

    float dpk = 0.0f;
    for (int i = 0; i < n; i++)
    {
        float a = fabsf(audio_filtered[i]);
        if (a > dpk) dpk = a;
    }

    fm_modulate(audio_filtered, out, n, dev_hz);

    rx_peak = dpk * dev_hz / 1000.0f;   /* peak deviation in kHz */
    rx_blocks++;
}


void nbfm_init(float if_freq_hz, float fs_hz);
static void ssb_tx_init(void);
extern volatile int am_testtone;
static void audio_dma_start(void);
extern volatile int tx_active;

/*
 * Filters are (re)initialised here rather than inline in main() so a mode
 * change can flush them. The FIR instances are sized for BLOCK_SIZE_MAX;
 * calling them later with a smaller block is safe, the state buffer is simply
 * bigger than that call needs.
 */
static void dsp_filters_init(void)
{
    // ssb_process_block() mixes with 'nco', not the 'nco_if' that nbfm_init()
    // sets up, so it needs its own init or dphi stays 0 and nothing is
    // downconverted.
    nco_init(&nco, 12000.0f, 48000.0f);

    // NCO for the FM path, plus pre_fm_I/Q and the post-demod audio LPF.
    nbfm_init(12000.0f, 48000.0f);

    ssb_tx_init();

    arm_biquad_cascade_df1_init_f32(&cw_bpf, CW_STAGES,
                                    (float *)cw_coeffs, cw_state);

    // Hilbert for SSB
    arm_fir_init_f32(&hilbert,
                     HILBERT_TAPS,
                     hilbert_coeffs,
                     hilbert_state,
                     BLOCK_SIZE_MAX/2);

}

// The DAC idles at mid-scale; zero would slam the output to the rail.
#define DAC_MID 2048

static void audio_dma_restart(void)
{
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);
    audio_dma_start();
}

/*
 * Half duplex: exactly one ADC and one DAC channel run at a time, chosen by
 * tx_active. RX reads IF off hadc1 (PC0) and writes demodulated audio to
 * DAC_OUT1 (PA4). TX reads the mic off hadc2 (PA3) and writes modulated IF
 * to DAC_OUT2 (PA5). adc_buffer/dac_buffer stay the generic "current input/
 * output" pair either way -- the DSP code does not need to know which side
 * is live.
 */
static void audio_dma_start(void)
{
    for (uint32_t i = 0; i < DAC_RING_LEN; i++)
        dac_buffer[i] = DAC_MID;

    memset(adc_buffer, 0, sizeof(adc_buffer));

    block_ready          = 0;
    dac_block_processing = 0;

    adc_rd = 0;

    uint32_t dac_len = (tx_active && mode_is_buffered(MODE)) ? DAC_RING_LEN
                                                             : block_size;
    uint32_t adc_len = mode_is_buffered(MODE) ? ADC_RING_LEN : block_size;

    if (tx_active)
    {
        HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t *)dac_buffer, dac_len, DAC_ALIGN_12B_R);
        HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_buffer, adc_len);
    }
    else
    {
        HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)dac_buffer, dac_len, DAC_ALIGN_12B_R);
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, adc_len);
    }
}

/*
 * Switch demodulator, and with it the block size.
 *
 * FreeDV wants 40 ms blocks so each one is exactly a modem frame; the analog
 * modes want short ones so CW break-in is not buried under latency. The DMA
 * length is fixed when it is started, so changing it means stopping and
 * restarting both streams.
 */
/*
 * Everything that varies per mode, in one table.
 *
 * Keeping it here rather than scattered through if-chains means a front panel
 * menu only has to index this array, and adding a mode is one row rather than
 * edits in five places.
 *
 * buffered: run the DSP out of a ring buffer instead of straight off the DMA
 * half-buffer. Buffered decouples processing time from the block period, which
 * the FreeDV modes need; direct keeps latency at one block, which is what CW
 * break-in wants. See the audio input ring below.
 */
typedef struct {
    const char *name;
    uint32_t    block_size;
    uint8_t     buffered;
    int8_t      chain_mode;   /* FREEDV_CHAIN_MODE_*, or -1 if not FreeDV */
} mode_cfg_t;

static const mode_cfg_t mode_cfg[] = {
    [MODE_NBFM]         = { "NBFM",  BLOCK_SIZE_ANALOG, 0, -1 },
    [MODE_USB]          = { "USB",   BLOCK_SIZE_ANALOG, 0, -1 },
    [MODE_LSB]          = { "LSB",   BLOCK_SIZE_ANALOG, 0, -1 },
    [MODE_FREEDV]       = { "FreeDV 1600",  BLOCK_SIZE_FREEDV, 1, FREEDV_CHAIN_MODE_1600  },
    [MODE_FREEDV_2400B] = { "FreeDV 2400B", BLOCK_SIZE_2400B,  1, FREEDV_CHAIN_MODE_2400B },
    [MODE_FREEDV_700D]  = { "FreeDV 700D",  BLOCK_SIZE_700D,   1, FREEDV_CHAIN_MODE_700D  },
    [MODE_FREEDV_700E]  = { "FreeDV 700E",  BLOCK_SIZE_700E,   1, FREEDV_CHAIN_MODE_700E  },
    [MODE_AM]           = { "AM",    BLOCK_SIZE_ANALOG, 0, -1 },
    [MODE_CW]           = { "CW",    BLOCK_SIZE_ANALOG, 0, -1 },
};

#define MODE_COUNT ((int)(sizeof(mode_cfg) / sizeof(mode_cfg[0])))

static const mode_cfg_t *cfg_for(int mode)
{
    if (mode < 0 || mode >= MODE_COUNT || mode_cfg[mode].name == NULL)
        return &mode_cfg[MODE_USB];
    return &mode_cfg[mode];
}

uint32_t block_size_for(int mode) { return cfg_for(mode)->block_size; }
int      mode_is_freedv(int mode) { return cfg_for(mode)->chain_mode >= 0; }
int      chain_mode_for(int mode) { return cfg_for(mode)->chain_mode; }
int      mode_is_buffered(int mode) { return cfg_for(mode)->buffered; }
const char *mode_name(int mode)   { return cfg_for(mode)->name; }

/*
 * Largest block malloc() can still hand out. codec2 allocates its modem state
 * in a few big chunks (2400A's f_dc alone is about 65K), so this is a better
 * predictor of whether freedv_open() will succeed than total free bytes.
 */
static uint32_t heap_largest_free(void)
{
    uint32_t lo = 0, hi = 256u * 1024u;

    while (lo < hi)
    {
        uint32_t mid = (lo + hi + 1u) / 2u;
        void    *p   = malloc(mid);

        if (p) { free(p); lo = mid; }
        else   { hi = mid - 1u; }
    }
    return lo;
}

void radio_set_mode(int mode)
{
    uint32_t want = block_size_for(mode);

    if (mode == MODE && want == block_size)
        return;

    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_2);

    MODE       = mode;
    block_size = want;

    // Filter history and the FreeDV resamplers hold audio from the old mode at
    // the old block size, so flush both rather than let it bleed through.
    dsp_filters_init();

    if (mode_is_freedv(mode))
    {
        // 1600 and 2400A are different modems, so this reopens codec2 rather
        // than just flushing what is buffered.
        freedv_ok = (freedv_chain_init(chain_mode_for(mode)) == 0);

        uart_puts(freedv_ok ? "freedv: ready\r\n"
                            : "freedv: init FAILED\r\n");
    }
    else if (freedv_ok)
    {
        freedv_chain_reset();
    }

    audio_dma_start();
}

/*
 * FreeDV 2400A receive.
 *
 * 2400A rides inside an ordinary FM channel, so this is the FM path, not the
 * SSB one: mix the 12 kHz IF down, filter the channel wide enough to keep the
 * 4FSK tones intact, and FM demodulate. What comes out is the modem's own
 * 48 kHz signal, which goes straight to the demodulator without resampling.
 *
 * Deliberately no post-demod audio LPF: the voice filter would cut the 4800 Hz
 * tone and the modem would never sync.
 */
void freedv2400b_process_block(const uint16_t *in, uint16_t *out, int n)
{
    float *I_buf     = &dsp_scratch[0 * SCRATCH_N];
    float *Q_buf     = &dsp_scratch[1 * SCRATCH_N];
    float *audio_buf = &dsp_scratch[2 * SCRATCH_N];

    float *IQ_in     = &dsp_scratch[3 * SCRATCH_N];
    float *NCO_buf   = &dsp_scratch[5 * SCRATCH_N];
    float *IQ_mix    = &dsp_scratch[7 * SCRATCH_N];

    float adc_peak = 0.0f;

    for (int i = 0; i < n; i++)
    {
        uint32_t raw = in[i] & 0x0FFF;
        I_buf[i] = ((float)raw - 2048.0f) / 2048.0f;

        float a = fabsf(I_buf[i]);
        if (a > adc_peak) adc_peak = a;
    }
    rx_adc_peak = adc_peak;

    arm_fir_f32(&hilbert, I_buf, Q_buf, n);

    for (int i = 0; i < n; i++)
    {
        IQ_in[2*i + 0] = I_buf[i];
        IQ_in[2*i + 1] = Q_buf[i];
    }

    nco_block_iq(NCO_buf, n);
    arm_cmplx_mult_cmplx_f32(IQ_in, NCO_buf, IQ_mix, n);

    for (int i = 0; i < n; i++)
    {
        I_buf[i] = IQ_mix[2*i + 0];
        Q_buf[i] = IQ_mix[2*i + 1];
    }

    // The ordinary voice channel filter is right here: 2400B is specified to
    // survive a 300-3000 Hz audio path, so it fits inside NBFM as it stands.
    arm_fir_f32(&pre_fm_I, I_buf, I_buf, n);
    arm_fir_f32(&pre_fm_Q, Q_buf, Q_buf, n);

    // Data discriminator, not the voice one: no de-emphasis, no voice AGC.
    fm_discriminate(I_buf, Q_buf, audio_buf, n);

    /*
     * Peak alone cannot tell an over-deviated signal from an occasional noise
     * spike, so track rms as well. A clean FM data signal sits around peak/rms
     * of 3-4; a much larger ratio means the discriminator is spiking, and a
     * peak near 1.0 with high rms means the deviation itself is too big and
     * atan2 is wrapping.
     */
    float peak = 0.0f;
    float sumsq = 0.0f;

    for (int i = 0; i < n; i++)
    {
        float a = fabsf(audio_buf[i]);
        if (a > peak) peak = a;
        sumsq += audio_buf[i] * audio_buf[i];
    }

    rx_peak = peak;
    rx_rms  = sqrtf(sumsq / (float)n);
    rx_blocks++;

    if (freedv_ok)
    {
        uint32_t t0 = DWT->CYCCNT;

        freedv_chain_put_audio48(audio_buf, n);
        freedv_chain_get_speech48(audio_buf, n);

        uint32_t dt = DWT->CYCCNT - t0;
        if (dt > rx_fdv_cycles_max)
            rx_fdv_cycles_max = dt;
    }

    for (int i = 0; i < n; i++)
    {
        float y = audio_buf[i] * 1500.0f + 2048.0f;

        if (y < 0.0f)    y = 0.0f;
        if (y > 4095.0f) y = 4095.0f;

        out[i] = (uint16_t)y;
    }
}

/*
 * AM receive.
 *
 * Deliberately not routed through the SSB path. That one builds its complex
 * signal with a Hilbert transformer, which AM does not need: mixing the real
 * IF against cos and -sin gives I and Q directly, and the existing channel
 * filter removes the sum-frequency image. Skipping the Hilbert removes both
 * its quadrature error and its group delay, neither of which the envelope
 * detector can tolerate -- any I/Q imbalance turns straight into amplitude
 * error, and the whole point of AM is that the amplitude is the signal.
 *
 * The envelope is then |I + jQ|, which needs no knowledge of the carrier
 * phase and, unlike SSB, is completely unaffected by a tuning offset.
 */
void am_process_block(const uint16_t *in, uint16_t *out, int n)
{
    float *I_buf     = &dsp_scratch[0 * SCRATCH_N];
    float *Q_buf     = &dsp_scratch[1 * SCRATCH_N];
    float *audio_buf = &dsp_scratch[2 * SCRATCH_N];
    float *NCO_buf   = &dsp_scratch[3 * SCRATCH_N];   /* interleaved, 2n */

    static dc_block_t dc_am  = {0};
    static agc_t      agc_am = {1.0f};

    float adc_peak = 0.0f;

    nco_block_iq(NCO_buf, n);

    for (int i = 0; i < n; i++)
    {
        float x = ((float)(in[i] & 0x0FFF) - 2048.0f) / 2048.0f;

        float a = fabsf(x);
        if (a > adc_peak) adc_peak = a;

        /* Quadrature downconversion: multiply by exp(-j*w*t). */
        I_buf[i] =  x * NCO_buf[2*i + 0];
        Q_buf[i] = -x * NCO_buf[2*i + 1];
    }

    rx_adc_peak = adc_peak;

    /* Removes the image at twice the IF that the mixing leaves behind. */
    arm_fir_f32(&pre_fm_I, I_buf, I_buf, n);
    arm_fir_f32(&pre_fm_Q, Q_buf, Q_buf, n);

    float peak = 0.0f, sumsq = 0.0f;
    float env_min = 1e9f, env_sum = 0.0f;

    for (int i = 0; i < n; i++)
    {
        float env = sqrtf(I_buf[i] * I_buf[i] + Q_buf[i] * Q_buf[i]);

        /*
         * env_min against env_avg says whether the transmitter sent a carrier.
         * Proper AM never lets the envelope reach zero, so env_min stays near
         * (1 - m) of the mean. If env_min collapses to zero the envelope is
         * being rectified, which is what a suppressed carrier looks like and
         * why the tone comes out at twice its frequency.
         */
        if (env < env_min) env_min = env;
        env_sum += env;

        /* The carrier is a DC pedestal under the envelope; drop it. */
        audio_buf[i] = dc_block(env, &dc_am);

        float a = fabsf(audio_buf[i]);
        if (a > peak) peak = a;
        sumsq += audio_buf[i] * audio_buf[i];
    }

    rx_env_min = env_min;
    rx_env_avg = env_sum / (float)n;
    rx_peak = peak;
    rx_rms  = sqrtf(sumsq / (float)n);
    rx_blocks++;

    agc_block(audio_buf, n, &agc_am);

    for (int i = 0; i < n; i++)
    {
        float y = audio_buf[i] * 1500.0f + 2048.0f;

        if (y < 0.0f)    y = 0.0f;
        if (y > 4095.0f) y = 4095.0f;

        out[i] = (uint16_t)y;
    }
}


/* ------------------------------------------------------------------------
 * SSB transmit, phasing method
 *
 * Audio is band limited, split into a quadrature pair, and mixed up to the IF:
 *
 *   USB:  I*cos(wt) - Q*sin(wt)
 *   LSB:  I*cos(wt) + Q*sin(wt)
 *
 * where Q is the Hilbert transform of the audio and I is the audio delayed by
 * the transformer's group delay, so the two stay aligned.
 *
 * The Hilbert is 301 taps, which looks excessive until you notice that 300 Hz
 * is 0.6 % of Nyquist at 48 kHz, and a Hilbert transformer is at its worst
 * near DC. Fewer taps cost real opposite-sideband suppression: 129 taps manage
 * only about 18 dB, 201 give 30 dB, 301 give 56 dB. Transmitting the unwanted
 * sideband is other people's problem as much as ours, so it is worth the
 * arithmetic -- and transmit does not run at the same time as receive, so the
 * whole CPU budget is free anyway.
 * --------------------------------------------------------------------- */

#define SSB_HIL_TAPS  301
#define SSB_HIL_DELAY ((SSB_HIL_TAPS - 1) / 2)

static float ssb_hil_coeffs[SSB_HIL_TAPS];
/*
 * State is sized for the analog block, so anything longer must be fed through
 * in pieces: CMSIS writes numTaps + blockSize - 1 floats here, and the FreeDV
 * path calls this with 1920 where the analog modes use 256.
 */
#define SSB_HIL_MAXBLK (BLOCK_SIZE_ANALOG / 2)

static float ssb_hil_state[SSB_HIL_TAPS + SSB_HIL_MAXBLK];
static arm_fir_instance_f32 ssb_hil;

/* Speech band limiting: high pass at 300 Hz, low pass at 2700 Hz. */
static const float ssb_audio_coeffs[10] = {
    +9.7260993065e-01f,
    -1.9452198613e+00f,
    +9.7260993065e-01f,
    +1.9444697251e+00f,
    -9.4596999747e-01f,
    +2.4827170061e-02f,
    +4.9654340121e-02f,
    +2.4827170061e-02f,
    +1.5074026397e+00f,
    -6.0671131993e-01f
};
static float ssb_audio_state[8];
static arm_biquad_casd_df1_inst_f32 ssb_audio;

/* Delay line for the I branch, matching the Hilbert group delay. */
static float ssb_delay[SSB_HIL_DELAY];
static int   ssb_delay_pos;

/*
 * Microphone gain. A dynamic mic into a 3.3 V ADC input needs a lot of it, and
 * without any the modulator is fed a signal that barely leaves the noise
 * floor. Adjustable from the console so the level can be set while watching
 * adc_x1000 and the scope.
 */
float mic_gain = 1.0f;

static float ssb_osc_re = 1.0f, ssb_osc_im = 0.0f;
static float ssb_step_re, ssb_step_im;

static void ssb_tx_init(void)
{
    const int M = SSB_HIL_DELAY;

    for (int i = 0; i < SSB_HIL_TAPS; i++)
    {
        int n = i - M;

        if (n == 0 || (n % 2) == 0)
        {
            ssb_hil_coeffs[i] = 0.0f;   /* a Hilbert has no even-index taps */
            continue;
        }

        float w = 0.54f - 0.46f * cosf(2.0f * (float)M_PI * (float)i
                                       / (float)(SSB_HIL_TAPS - 1));

        ssb_hil_coeffs[i] = 2.0f / ((float)M_PI * (float)n) * w;
    }

    arm_fir_init_f32(&ssb_hil, SSB_HIL_TAPS, ssb_hil_coeffs,
                     ssb_hil_state, SSB_HIL_MAXBLK);
    arm_biquad_cascade_df1_init_f32(&ssb_audio, 2,
                                    (float *)ssb_audio_coeffs, ssb_audio_state);
}

void ssb_tx_restart(void)
{
    float dphi = 2.0f * (float)M_PI * 12000.0f / 48000.0f;

    ssb_step_re = cosf(dphi);
    ssb_step_im = sinf(dphi);
    ssb_osc_re  = 1.0f;
    ssb_osc_im  = 0.0f;

    for (int i = 0; i < SSB_HIL_DELAY; i++) ssb_delay[i] = 0.0f;
    ssb_delay_pos = 0;
}

/*
 * Run the Hilbert in chunks no larger than the state was sized for. The filter
 * keeps its own history between calls, so splitting the block changes nothing
 * about the output -- it only keeps CMSIS from writing past the state buffer.
 */
static void ssb_hilbert(const float *in, float *out, int n)
{
    for (int off = 0; off < n; off += SSB_HIL_MAXBLK)
    {
        int m = n - off;

        if (m > SSB_HIL_MAXBLK) m = SSB_HIL_MAXBLK;

        arm_fir_f32(&ssb_hil, (float *)&in[off], &out[off], m);
    }
}

/*
 * Modulate a ready audio buffer. Split out so the FreeDV path can reach it:
 * a modem waveform must not be run through the speech band pass or the
 * microphone gain, since it is already shaped and any further filtering or
 * compression distorts it.
 */
static void ssb_modulate(const float *audio, const float *q,
                         uint16_t *out, int n, int lsb)
{
    for (int i = 0; i < n; i++)
    {
        float d = ssb_delay[ssb_delay_pos];

        ssb_delay[ssb_delay_pos] = audio[i];
        if (++ssb_delay_pos >= SSB_HIL_DELAY) ssb_delay_pos = 0;

        float s = lsb ? (d * ssb_osc_re + q[i] * ssb_osc_im)
                      : (d * ssb_osc_re - q[i] * ssb_osc_im);

        float nre = ssb_osc_re * ssb_step_re - ssb_osc_im * ssb_step_im;
        float nim = ssb_osc_re * ssb_step_im + ssb_osc_im * ssb_step_re;
        float g   = 1.5f - 0.5f * (nre * nre + nim * nim);

        ssb_osc_re = nre * g;
        ssb_osc_im = nim * g;

        float y = s * 1800.0f + 2048.0f;

        if (y < 0.0f)    y = 0.0f;
        if (y > 4095.0f) y = 4095.0f;

        out[i] = (uint16_t)y;
    }
}

void ssb_tx_process_block(const uint16_t *in, uint16_t *out, int n)
{
    float *audio = &dsp_scratch[0 * SCRATCH_N];
    float *q     = &dsp_scratch[1 * SCRATCH_N];

    float peak = 0.0f;

    for (int i = 0; i < n; i++)
    {
        float x = ((float)(in[i] & 0x0FFF) - 2048.0f) / 2048.0f;

        float a = fabsf(x);
        if (a > peak) peak = a;

        audio[i] = x * mic_gain;
    }

    rx_adc_peak = peak;                 /* raw mic level, before the gain */
    rx_blocks++;

    arm_biquad_cascade_df1_f32(&ssb_audio, audio, audio, n);
    ssb_hilbert(audio, q, n);

    /* USB/LSB swapped vs the phasing convention so the label matches the air,
       confirmed against an FT-857D. FreeDV keeps the plain USB call below. */
    ssb_modulate(audio, q, out, n, MODE == MODE_USB);
}


/* ------------------------------------------------------------------------
 * CW transmit
 *
 * Sends a beacon at the IF the receiver is tuned to. The carrier sits at
 * CW_PITCH_HZ above the IF centre, which is the same offset the receive
 * filter is centred on: a station whose carrier gives us a 700 Hz tone is on
 * the frequency we have to answer on, so transmit has to land in the same
 * place.
 *
 * The envelope is shaped rather than switched. Hard keying splatters well
 * outside the occupied bandwidth -- key clicks are one of the more common
 * complaints on the CW bands -- so each edge is a raised cosine a few
 * milliseconds long.
 * --------------------------------------------------------------------- */

#define CW_MSG      "CQ CQ CQ DE LA7LKA"
#define CW_UNITS    256                     /* dit units in the keyed message */
#define CW_RAMP     240                     /* 5 ms edge at 48 kHz */

static const char *cw_morse(char c)
{
    switch (c) {
    case 'A': return ".-";    case 'B': return "-...";  case 'C': return "-.-.";
    case 'D': return "-..";   case 'E': return ".";     case 'F': return "..-.";
    case 'G': return "--.";   case 'H': return "....";  case 'I': return "..";
    case 'J': return ".---";  case 'K': return "-.-";   case 'L': return ".-..";
    case 'M': return "--";    case 'N': return "-.";    case 'O': return "---";
    case 'P': return ".--.";  case 'Q': return "--.-";  case 'R': return ".-.";
    case 'S': return "...";   case 'T': return "-";     case 'U': return "..-";
    case 'V': return "...-";  case 'W': return ".--";   case 'X': return "-..-";
    case 'Y': return "-.--";  case 'Z': return "--..";
    case '0': return "-----"; case '1': return ".----"; case '2': return "..---";
    case '3': return "...--"; case '4': return "....-"; case '5': return ".....";
    case '6': return "-...."; case '7': return "--..."; case '8': return "---..";
    case '9': return "----.";
    default:  return "";
    }
}

static uint8_t cw_key[CW_UNITS];    /* one entry per dit unit, 1 = key down */
static int     cw_key_len;

static int      cw_wpm = 20;
static uint32_t cw_dit_samples = 48000 * 12 / (10 * 20);

static int      cw_unit;            /* index into cw_key */
static uint32_t cw_tick;            /* samples into the current unit */
static float    cw_env;             /* shaped envelope, 0..1 */
static int      cw_ramp;            /* position within an edge */

static float    cw_osc_re = 1.0f, cw_osc_im = 0.0f;
static float    cw_step_re, cw_step_im;

static void cw_build_message(void)
{
    int n = 0;

    for (const char *p = CW_MSG; *p; p++)
    {
        if (*p == ' ')
        {
            /* Word gap is 7 units; 3 were already emitted after the letter. */
            for (int i = 0; i < 4 && n < CW_UNITS; i++) cw_key[n++] = 0;
            continue;
        }

        for (const char *e = cw_morse(*p); *e; e++)
        {
            int len = (*e == '-') ? 3 : 1;

            for (int i = 0; i < len && n < CW_UNITS; i++) cw_key[n++] = 1;
            if (n < CW_UNITS) cw_key[n++] = 0;          /* inter-element gap */
        }

        for (int i = 0; i < 2 && n < CW_UNITS; i++) cw_key[n++] = 0;  /* -> 3 */
    }

    for (int i = 0; i < 7 && n < CW_UNITS; i++) cw_key[n++] = 0;      /* tail */

    cw_key_len = n;
}

void cw_set_wpm(int wpm)
{
    cw_wpm = wpm;
    cw_dit_samples = (uint32_t)(48000.0f * 1.2f / (float)wpm);
}

int cw_get_wpm(void) { return cw_wpm; }

void cw_tx_restart(void)
{
    if (!cw_key_len) cw_build_message();

    cw_unit = 0;
    cw_tick = 0;
    cw_env  = 0.0f;
    cw_ramp = 0;

    /* Carrier at the IF centre plus the tone pitch, same as receive expects. */
    float dphi = 2.0f * (float)M_PI * (12000.0f + CW_PITCH_HZ) / 48000.0f;

    cw_step_re = cosf(dphi);
    cw_step_im = sinf(dphi);
    cw_osc_re  = 1.0f;
    cw_osc_im  = 0.0f;
}

void cw_tx_process_block(uint16_t *out, int n)
{
    for (int i = 0; i < n; i++)
    {
        int want = cw_key[cw_unit];

        if (++cw_tick >= cw_dit_samples)
        {
            cw_tick = 0;
            if (++cw_unit >= cw_key_len) cw_unit = 0;
        }

        /* Raised-cosine edge, so the spectrum stays where it belongs. */
        if (want && cw_ramp < CW_RAMP) cw_ramp++;
        else if (!want && cw_ramp > 0) cw_ramp--;

        cw_env = 0.5f * (1.0f - cosf((float)M_PI * (float)cw_ramp / (float)CW_RAMP));

        float y = cw_env * cw_osc_re * 1800.0f + 2048.0f;

        float nre = cw_osc_re * cw_step_re - cw_osc_im * cw_step_im;
        float nim = cw_osc_re * cw_step_im + cw_osc_im * cw_step_re;
        float g   = 1.5f - 0.5f * (nre * nre + nim * nim);

        cw_osc_re = nre * g;
        cw_osc_im = nim * g;

        if (y < 0.0f)    y = 0.0f;
        if (y > 4095.0f) y = 4095.0f;

        out[i] = (uint16_t)y;
    }
}


/*
 * FreeDV transmit.
 *
 * The microphone goes into codec2, and what comes back is a modem waveform
 * that still has to be put on the air by one of the analog modulators: 1600,
 * 700D and 700E ride on SSB, 2400B on FM. So this is the vocoder and modem in
 * front of the modulators, not a modulator of its own.
 *
 * Note what is deliberately absent: no speech band pass, no microphone gain,
 * no compression on the modem waveform. Those belong to voice. Applying them
 * to a modem signal distorts the very thing the far end has to demodulate,
 * which is why FreeDV operating tells you to set drive by peak and leave ALC
 * out of it.
 */
/*
 * Feed one block of microphone audio into the FreeDV chain (decimate only).
 * The encode is separate, so this stays cheap and can run every block.
 */
void freedv_tx_feed(const uint16_t *in, int n)
{
    float *audio = &dsp_scratch[0 * SCRATCH_N];
    float  peak  = 0.0f;

    for (int i = 0; i < n; i++)
    {
        float x = ((float)(in[i] & 0x0FFF) - 2048.0f) / 2048.0f;

        float a = fabsf(x);
        if (a > peak) peak = a;

        audio[i] = x * mic_gain;        /* gain on speech, before codec2 */
    }

    rx_adc_peak = peak;
    rx_blocks++;

    if (freedv_ok) freedv_chain_put_speech48(audio, n);
}

/*
 * Produce one block of modem IF into the DAC. Pulls interpolated modem samples
 * and runs them through the SSB or FM modulator. Cheap -- no encode here.
 */
void freedv_tx_produce(uint16_t *out, int n)
{
    float *audio = &dsp_scratch[0 * SCRATCH_N];
    float *q     = &dsp_scratch[1 * SCRATCH_N];

    if (!freedv_ok)
    {
        for (int i = 0; i < n; i++) out[i] = DAC_MID;
        return;
    }

    freedv_chain_get_modem48(audio, n);     /* modem waveform, not speech */

    float mpk = 0.0f;
    for (int i = 0; i < n; i++)
    {
        float a = fabsf(audio[i]);
        if (a > mpk) mpk = a;
    }
    rx_peak = mpk;                          /* modem drive level */

    if (MODE == MODE_FREEDV_2400B)
    {
        fm_modulate(audio, out, n, 2500.0f);   /* what 2400B expects */
    }
    else
    {
        ssb_hilbert(audio, q, n);
        ssb_modulate(audio, q, out, n, 0);  /* FreeDV rides on USB */
    }
}

/*
 * AM transmit.
 *
 * Amplitude modulation is a carrier plus the audio riding on it: the envelope
 * is (1 + m*audio) and that multiplies a cos at the IF. Unlike SSB there is no
 * Hilbert and no sideband selection -- both sidebands are sent, symmetric
 * about the carrier -- so this is simpler than the SSB modulator, not harder.
 *
 * The modulation index m is kept a little under 1 so the envelope never
 * reaches zero; at m = 1 it just touches zero (100 %), and above that it would
 * go negative and distort, exactly the over-modulation the receiver side
 * showed. Mic gain sets the drive into it.
 */
void am_tx_process_block(const uint16_t *in, uint16_t *out, int n)
{
    float *audio = &dsp_scratch[0 * SCRATCH_N];

    float peak = 0.0f;

    if (am_testtone)
    {
        /* Fixed internal 1 kHz tone at 50% depth -- removes the mic entirely,
           so a clean AM signal here proves the modulator. */
        static float ph = 0.0f;
        const float dph = 2.0f * (float)M_PI * 1000.0f / 48000.0f;

        for (int i = 0; i < n; i++)
        {
            audio[i] = 0.85f * cosf(ph);   /* ~80%% modulation depth */
            ph += dph;
            if (ph > 2.0f * (float)M_PI) ph -= 2.0f * (float)M_PI;
        }
        rx_adc_peak = 0.85f;
    }
    else
    {
        for (int i = 0; i < n; i++)
        {
            float x = ((float)(in[i] & 0x0FFF) - 2048.0f) / 2048.0f;

            float a = fabsf(x);
            if (a > peak) peak = a;

            audio[i] = x * mic_gain;
        }

        rx_adc_peak = peak;

        /* Speech band limiting, reusing the SSB audio filter. */
        arm_biquad_cascade_df1_f32(&ssb_audio, audio, audio, n);
    }

    rx_blocks++;

    const float m = 0.95f;      /* keep the envelope non-negative */

    float mpk = 0.0f;

    for (int i = 0; i < n; i++)
    {
        float env = 1.0f + m * audio[i];

        if (env < 0.0f) env = 0.0f;              /* no over-modulation */
        if (env > mpk)  mpk  = env;

        float carrier = ssb_osc_re;              /* cos at the IF */

        float nre = ssb_osc_re * ssb_step_re - ssb_osc_im * ssb_step_im;
        float nim = ssb_osc_re * ssb_step_im + ssb_osc_im * ssb_step_re;
        float g   = 1.5f - 0.5f * (nre * nre + nim * nim);

        ssb_osc_re = nre * g;
        ssb_osc_im = nim * g;

        /* 1000 is about the ceiling: full modulation takes the envelope to ~2x
           the carrier, so 2 * 1000 fits the 2048 DAC half-swing. */
        float y = env * carrier * 1000.0f + 2048.0f;

        if (y < 0.0f)    y = 0.0f;
        if (y > 4095.0f) y = 4095.0f;

        out[i] = (uint16_t)y;
    }

    rx_peak = mpk;                                /* peak envelope */
}

/*
 * Transmit dispatch. CW keys its own carrier and takes no input; the voice
 * modes modulate whatever is on the microphone ADC.
 */
static void tx_process_block(const uint16_t *in, uint16_t *out, int n)
{
    if (MODE == MODE_CW)                            cw_tx_process_block(out, n);
    else if (MODE == MODE_USB || MODE == MODE_LSB)  ssb_tx_process_block(in, out, n);
    else if (MODE == MODE_NBFM)                     nbfm_tx_process_block(in, out, n);
    else if (MODE == MODE_AM)                       am_tx_process_block(in, out, n);
    else
    {
        for (int i = 0; i < n; i++) out[i] = 2048;  /* nothing to send yet */
    }
}

static int tx_supported(int mode)
{
    return mode == MODE_CW || mode == MODE_USB || mode == MODE_LSB
        || mode == MODE_NBFM || mode == MODE_AM || mode_is_freedv(mode);
}

/* ------------------------------------------------------------------------
 * UART console
 *
 * Exists so mode changes, PTT and levels can be driven over the ST-Link VCP
 * instead of by reflashing. A front panel comes later; putting the control
 * layer in first means the OLED and switches become a second way to reach
 * commands that already work, rather than a second thing to debug at the same
 * time as transmit.
 * --------------------------------------------------------------------- */

volatile int tx_active = 0;      /* set by ptt/tx, cleared by rx */
volatile int tx_rearm  = 1;      /* re-align the TX mic read on each PTT */
volatile int am_testtone = 0;    /* AM: modulate an internal 1 kHz tone, not the mic */

static char cons_line[64];
static int  cons_len;

static int str_eq(const char *a, const char *b)
{
    while (*a && *b && *a == *b) { a++; b++; }
    return *a == 0 && *b == 0;
}

static int str_num(const char *s, int *out)
{
    int v = 0, any = 0;
    while (*s >= '0' && *s <= '9') { v = v * 10 + (*s++ - '0'); any = 1; }
    return any ? (*out = v, 1) : 0;
}

static void console_help(void)
{
    uart_puts("\r\ncommands:\r\n"
              "  mode          list modes\r\n"
              "  mode <n>      select mode by number\r\n"
              "  tx            key the CW beacon\r\n"
              "  rx            back to receive\r\n"
              "  mic <n>       microphone gain, 1..200\r\n"
              "  amtone        toggle AM 1 kHz test tone (50%%)\r\n"
              "  debug [on|off] toggle continuous telemetry\r\n"
              "  wpm <n>       CW speed\r\n"
              "  stat          current state\r\n");
}

static void console_modes(void)
{
    uart_puts("\r\n");
    for (int i = 0; i < MODE_COUNT; i++)
    {
        if (mode_cfg[i].name == NULL) continue;
        uart_kv("", i);
        uart_puts(mode_cfg[i].name);
        uart_puts(i == MODE ? "   <= current\r\n" : "\r\n");
    }
}

static void console_exec(char *line)
{
    char *arg = line;

    while (*arg && *arg != ' ') arg++;
    if (*arg == ' ') *arg++ = 0;

    if (str_eq(line, "help") || str_eq(line, "?"))
    {
        console_help();
    }
    else if (str_eq(line, "mode"))
    {
        int n;
        if (str_num(arg, &n) && n >= 0 && n < MODE_COUNT && mode_cfg[n].name)
        {
            tx_active = 0;
            radio_set_mode(n);
            uart_puts("mode: ");
            uart_puts(mode_name(n));
            uart_puts("\r\n");
        }
        else console_modes();
    }
    else if (str_eq(line, "tx"))
    {
        if (!tx_supported(MODE))
        {
            uart_puts("no modulator for this mode yet\r\n");
        }
        else
        {
            if (MODE == MODE_CW) cw_tx_restart();
            else                 ssb_tx_restart();  /* also inits the AM carrier osc */

            if (mode_is_freedv(MODE) && freedv_ok) freedv_chain_set_tx(1);

            tx_rearm  = 1;
            tx_active = 1;
            audio_dma_restart();  /* mic-in ADC / IF-out DAC, deep ring if buffered */
            HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
            uart_puts("tx: ");
            uart_puts(mode_name(MODE));
            uart_puts("\r\n");
        }
    }
    else if (str_eq(line, "rx"))
    {
        tx_active = 0;
        if (mode_is_freedv(MODE) && freedv_ok) freedv_chain_set_tx(0);
        audio_dma_restart();   /* back to IF-in ADC / audio-out DAC */
        HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
        uart_puts("rx\r\n");
    }
    else if (str_eq(line, "debug"))
    {
        if      (str_eq(arg, "on"))  debug_on = 1;
        else if (str_eq(arg, "off")) debug_on = 0;
        else                         debug_on = !debug_on;
        uart_puts(debug_on ? "debug telemetry ON\r\n" : "debug telemetry off\r\n");
    }
    else if (str_eq(line, "amtone"))
    {
        am_testtone = !am_testtone;
        uart_puts(am_testtone ? "AM test tone ON (1 kHz, 50%)\r\n"
                              : "AM test tone off\r\n");
    }
    else if (str_eq(line, "mic"))
    {
        int n;
        if (str_num(arg, &n) && n >= 1 && n <= 200)
        {
            mic_gain = (float)n;
            uart_kv("mic gain", n);
            uart_puts("\r\n");
        }
        else uart_puts("mic 1..200\r\n");
    }
    else if (str_eq(line, "wpm"))
    {
        int n;
        if (str_num(arg, &n) && n >= 5 && n <= 60) { cw_set_wpm(n); uart_kv("wpm", n); uart_puts("\r\n"); }
        else uart_puts("wpm 5..60\r\n");
    }
    else if (str_eq(line, "stat"))
    {
        uart_puts("\r\nmode: ");
        uart_puts(mode_name(MODE));
        uart_kv("  tx", tx_active);
        uart_kv("wpm", cw_get_wpm());
        uart_kv("mic", (int)mic_gain);
        uart_kv("block", (int)block_size / 2);
        uart_puts("\r\n");
    }
    else if (*line)
    {
        uart_puts("? try help\r\n");
    }
}

static void console_poll(void)
{
    if (!(huart3.Instance->ISR & USART_ISR_RXNE))
        return;

    char c = (char)(huart3.Instance->RDR & 0xFF);

    if (c == '\r' || c == '\n')
    {
        uart_puts("\r\n");
        cons_line[cons_len] = 0;
        console_exec(cons_line);
        cons_len = 0;
        uart_puts("> ");
    }
    else if ((c == 8 || c == 127) && cons_len)
    {
        cons_len--;
        uart_puts("\b \b");
    }
    else if (c >= ' ' && cons_len < (int)sizeof(cons_line) - 1)
    {
        cons_line[cons_len++] = c;
        char e[2] = { c, 0 };
        uart_puts(e);
    }
}

void process_block(const uint16_t *in, uint16_t *out, int n)
{
    uint32_t t0 = DWT->CYCCNT;

    if (MODE == MODE_NBFM)
    {
        nbfm_process_block(in, out, n);
    }
    else if (MODE == MODE_FREEDV_2400B)
    {
        freedv2400b_process_block(in, out, n);  // FM-bane, ikke SSB
    }
    else if (MODE == MODE_AM)
    {
        am_process_block(in, out, n);           // egen bane, ingen Hilbert
    }
    else
    {
        ssb_process_block(in, out, n);  // din eksisterende SSB-pipe
    }

    uint32_t dt = DWT->CYCCNT - t0;
    if (dt > rx_cycles_max)
        rx_cycles_max = dt;
}


void nbfm_init(float if_freq_hz, float fs_hz)
{
    // NCO for IF-mixing
    nco_init(&nco_if, if_freq_hz, fs_hz);

    // Pre-demod IF-filter (bruk dine egne koeffisienter)
arm_fir_init_f32(&pre_fm_I,
                 PRE_DEMOD_TAPS,
                 pre_demod_coeffs,
                 pre_fm_state_I,
                 BLOCK_SIZE_MAX/2);

arm_fir_init_f32(&pre_fm_Q,
                 PRE_DEMOD_TAPS,
                 pre_demod_coeffs,
                 pre_fm_state_Q,
                 BLOCK_SIZE_MAX/2);


    // Audio LPF etter FM-demod
    arm_fir_init_f32(&audio_lpf,
                     AUDIO_TAPS,
                     audio_lpf_coeffs,
                     audio_state,
                     BLOCK_SIZE_ANALOG/2);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  // At 216 MHz flash runs with 7 wait states, so without the instruction cache
  // every fetch stalls. Enabling it costs nothing in correctness (unlike the
  // D-cache, which would need coherency handling for the ADC/DAC DMA buffers)
  // and is worth several times the throughput on this DSP path.
  SCB_EnableICache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ETH_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  dsp_filters_init();

  block_size = block_size_for(MODE);
  audio_dma_start();

  //Start TIM6
  HAL_TIM_Base_Start(&htim6);

  // Brought up after the DMAs are running, and deliberately not fatal: if
  // codec2 cannot allocate we still want the radio and the UART alive so the
  // failure is visible instead of silently hanging the board.
  cyclecount_init();

  uart_puts("\r\n=== qrp-sdr-trx ===\r\n");

  uart_kv("heap_largest", (int)heap_largest_free());
  uart_puts("\r\n");

  freedv_ok = mode_is_freedv(MODE)
            ? (freedv_chain_init(chain_mode_for(MODE)) == 0)
            : 0;

  if (freedv_ok)
  {
      uart_puts("freedv: ");
      uart_puts(mode_name(MODE));
      uart_puts(" RX ready\r\n");
  }
  else if (mode_is_freedv(MODE))
  {
      uart_puts("freedv: init FAILED (out of heap?) - audio will be silent\r\n");
  }

  uart_kv("heap_left", (int)heap_largest_free());
  uart_puts("\r\ntype 'help' for commands\r\n> ");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    uart_pump();
    console_poll();

    if (tx_active)
    {
      uint32_t n = block_size / 2;

      if (mode_is_buffered(MODE))
      {
        /*
         * FreeDV transmit is decoupled from DAC timing. The mic is read from
         * the ADC ring following its write pointer; the encode (up to 53 ms for
         * 700E) runs here in the main loop; and the DAC is a deep circular
         * buffer the DMA plays through while we fill ahead of its read pointer.
         * A slow encode draws down the cushion instead of stalling the output.
         */
        static uint16_t tx_mic[BLOCK_SIZE_MAX / 2];
        static uint32_t tx_rd;    /* mic read pos in the ADC ring */
        static uint32_t dac_wr;   /* fill pos in the DAC ring */

        if (tx_rearm)
        {
          tx_rd    = (adc_dma_pos() + ADC_RING_LEN - 2 * n) % ADC_RING_LEN;
          dac_wr   = 0;
          tx_rearm = 0;
        }

        /* Feed every mic block the ADC has captured since last time. */
        while (((adc_dma_pos() + ADC_RING_LEN - tx_rd) % ADC_RING_LEN) >= n)
        {
          for (uint32_t i = 0; i < n; i++)
            tx_mic[i] = adc_buffer[(tx_rd + i) % ADC_RING_LEN];

          tx_rd = (tx_rd + n) % ADC_RING_LEN;
          freedv_tx_feed(tx_mic, n);
        }

        /* Encode ahead -- this is the heavy, bursty part. */
        {
          uint32_t te = DWT->CYCCNT;
          freedv_chain_encode();
          uint32_t enc = DWT->CYCCNT - te;
          if (enc > rx_adc_cycles) rx_adc_cycles = enc;
        }

        /* Fill the DAC ahead of where the DMA is reading. */
        {
          uint32_t play  = dac_dma_pos();
          uint32_t ahead = (dac_wr + DAC_RING_LEN - play) % DAC_RING_LEN;

          if (ahead < tx_cushion_min) tx_cushion_min = ahead;

          while (ahead <= DAC_RING_LEN - 2 * n &&
                 freedv_chain_modem_avail48() >= (int)n)
          {
            uint32_t t0 = DWT->CYCCNT;
            freedv_tx_produce(&dac_buffer[dac_wr], n);
            uint32_t dt = DWT->CYCCNT - t0;
            if (dt > rx_cycles_max)     rx_cycles_max     = dt;
            if (dt > rx_fdv_cycles_max) rx_fdv_cycles_max = dt;

            dac_wr = (dac_wr + n) % DAC_RING_LEN;
            ahead += n;
          }
        }
      }
      else if (dac_block_processing == 1)
      { tx_process_block(&adc_buffer[0], &dac_buffer[0], n); dac_block_processing = 0; }
      else if (dac_block_processing == 2)
      { tx_process_block(&adc_buffer[n], &dac_buffer[n], n); dac_block_processing = 0; }
    }
    else
    // RX path. Swap this with the nbfm_tx_process_block() block below to go
    // back to transmit testing.
    if (mode_is_buffered(MODE))
    {
      /*
       * Drain the ring as fast as it fills rather than once per DMA deadline,
       * so a slow frame is caught up on afterwards instead of leaving a
       * backlog that grows until the ring overruns. Output alternates between
       * the DAC halves; if we are behind, the DAC briefly repeats what is
       * already there, which is audible but costs no modem samples.
       */
      static int dac_half = 0;
      uint32_t n = block_size / 2;
      int      guard = 4;   /* keep the loop from starving the telemetry */

      /* Writer lapped us: data was lost, so resync rather than read garbage. */
      if (adc_avail() > ADC_RING_LEN - n)
      {
        rx_overruns++;
        adc_rd = (adc_dma_pos() / n * n) % ADC_RING_LEN;
      }

      while (adc_avail() >= n && guard--)
      {
        process_block(&adc_buffer[adc_rd], &dac_buffer[dac_half ? n : 0], n);
        adc_rd = (adc_rd + n) % ADC_RING_LEN;
        dac_half ^= 1;
      }
    }
    else if (block_ready == 1)
    {
      process_block(&adc_buffer[0], &dac_buffer[0], block_size/2);
      block_ready = 0;
    }
    else if (block_ready == 2)
    {
      process_block(&adc_buffer[block_size/2], &dac_buffer[block_size/2], block_size/2);
      block_ready = 0;
    }

    // Once a second: are blocks flowing, is there signal, has the modem synced?
    // Only when the continuous debug print is enabled -- off by default so the
    // console prompt stays clean. Toggle with the 'debug' command.
    if (debug_on && HAL_GetTick() - last_report >= 1000)
    {
        last_report = HAL_GetTick();

        // Derived from the active block size so it stays correct across a
        // mode change: at 48 kHz there are 48 samples per millisecond, and the
        // core runs at 216 MHz.
        // In microseconds, not milliseconds: an analog block is 5.33 ms, and
        // rounding that to 5 would overstate the load by 6 %.
        const uint32_t block_us     = (block_size / 2) * 1000u / 48u;
        const uint32_t block_cycles = 216u * block_us;
        uint32_t load_pct = (uint32_t)(((uint64_t)rx_cycles_max * 100u) / block_cycles);

        uart_kv("mode", MODE);
        uart_kv("blocks", (int)rx_blocks);
        uart_kv("ovr", (int)rx_overruns);
        uart_kv("ring", mode_is_buffered(MODE) ? (int)adc_avail() : 0);
        uart_kv("load_pct", (int)load_pct);
        uart_kv("us_max", (int)(rx_cycles_max / 216));
        uart_kv("us_fdv", (int)(rx_fdv_cycles_max / 216));
        uart_kv("us_enc", (int)(rx_adc_cycles / 216));
        uart_kv("cush_us", (int)(tx_cushion_min / 48));
        rx_adc_cycles = 0;
        tx_cushion_min = 999999;
        {
            extern volatile unsigned freedv_tx_underruns;
            uart_kv("txun", (int)freedv_tx_underruns);
            freedv_tx_underruns = 0;
        }
        uart_kv("adc_x1000", (int)(rx_adc_peak * 1000.0f));
        uart_kv("peak_x1000", (int)(rx_peak * 1000.0f));
        uart_kv("rms_x1000", (int)(rx_rms * 1000.0f));
        if (MODE == MODE_AM)
        {
            uart_kv("env_min_x1000", (int)(rx_env_min * 1000.0f));
            uart_kv("env_avg_x1000", (int)(rx_env_avg * 1000.0f));
        }
        uart_kv("sync", freedv_chain_synced());
        uart_kv("snr_x10", (int)(freedv_chain_snr() * 10.0f));
        uart_puts("\r\n");

        rx_blocks     = 0;
        rx_overruns   = 0;
        rx_cycles_max = 0;
        rx_fdv_cycles_max = 0;
    }
    /*
    if(dac_block_processing == 1)
    {
        nbfm_tx_process_block(&adc_buffer[0], &dac_buffer[0], block_size/2);
        dac_block_processing = 0;
    }
    else if(dac_block_processing == 2)
    {
        nbfm_tx_process_block(&adc_buffer[block_size/2], &dac_buffer[block_size/2], block_size/2);
        dac_block_processing = 0;
    }
*/


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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T6_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  * RX IF in, PC0 (ADC123_IN10). Was PA3/channel 3 when this ADC also stood
  * in for the mic; the mic moved to ADC2 below so the two directions no
  * longer share a channel.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function -- TX mic in, PA3 (ADC123_IN3).
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T6_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config -- RX audio out, PA4
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config -- TX IF out, PA5
  */
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20404768;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim6.Init.Prescaler = 44;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 49;
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
static void MX_USART3_UART_Init(void)
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
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration (DAC channel 2, TX IF out) */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration (ADC2, TX mic in) */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
