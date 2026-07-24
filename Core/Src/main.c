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

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
uint32_t dac_buffer[BLOCK_SIZE_MAX] = {0};

/*
 * The ADC DMA buffer doubles as the input FIFO for the buffered modes, so it
 * is deeper than one block: five blocks, i.e. 200 ms at 48 kHz. The DMA writes
 * into it continuously and the DSP reads behind, which decouples processing
 * time from the block period without a second copy of the data. Halfword
 * samples because the ADC is 12-bit.
 */
#define ADC_RING_BLOCKS 5
#define ADC_RING_LEN    (ADC_RING_BLOCKS * (BLOCK_SIZE_MAX / 2))

uint16_t adc_buffer[ADC_RING_LEN] = {0};

volatile uint8_t block_ready = 0; // 0: no block ready, 1: first half ready, 2: second half ready
volatile uint8_t dac_block_processing = 0; // 0: not processing, 1: processing

int freedv_ok = 0;          // set once freedv_chain_init() has succeeded

// RX telemetry, printed once a second over the ST-Link VCP
volatile uint32_t rx_blocks = 0;   // blocks through process_block()
volatile float    rx_peak   = 0.0f; // peak |audio| at the modem input
volatile float    rx_rms    = 0.0f; // rms of the same, to judge peak/rms
volatile float    rx_adc_peak = 0.0f; // peak |ADC| before any filtering
volatile uint32_t rx_cycles_max = 0;  // worst-case cycles for one block
volatile uint32_t rx_fdv_cycles_max = 0; // worst-case cycles inside the FreeDV chain
static   uint32_t last_report = 0;

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// USART3 is wired to the ST-Link virtual COM port (/dev/ttyACM0), 115200 8N1.
void uart_puts(const char *s)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)s, strlen(s), 100);
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
 */
static uint32_t adc_rd;

static uint32_t adc_dma_pos(void)
{
    return ADC_RING_LEN - __HAL_DMA_GET_COUNTER(hadc1.DMA_Handle);
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

static float audio_state[AUDIO_TAPS + BLOCK_SIZE_MAX/2];
arm_fir_instance_f32 audio_lpf;


#define MODE_NBFM 0
#define MODE_USB  1
#define MODE_LSB  2
#define MODE_FREEDV 3   // FreeDV 1600, received as USB
#define MODE_FREEDV_2400B 4  // FreeDV 2400B, through a normal FM audio path
#define MODE_FREEDV_700D  5  // FreeDV 700D, OFDM + LDPC on SSB for weak signals

int MODE = MODE_FREEDV_700D;  // default mode

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

void ssb_process_block(const uint16_t *in, uint32_t *out, int n)
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
        // USB = I + Q
        for (int i = 0; i < n; i++)
            audio_buf[i] = I_buf[i] + Q_buf[i];
    }
    else if (MODE == MODE_LSB)
    {
        // LSB = I - Q
        for (int i = 0; i < n; i++)
            audio_buf[i] = I_buf[i] - Q_buf[i];
    }
    else if (MODE == MODE_FREEDV || MODE == MODE_FREEDV_700D)
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

        out[i] = (uint32_t)y;
    }
}

void nbfm_process_block(const uint16_t *in, uint32_t *out, int n)
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

        out[i] = (uint32_t)y;
    }
}

void nbfm_tx_process_block(const uint16_t *in, uint32_t *out, int n)
{
    static float audio_in[BLOCK_SIZE_MAX];
    static float audio_filtered[BLOCK_SIZE_MAX];
    static float phase = 0.0f;

    const float kf = 2.0f * M_PI * 5000.0f / 48000.0f;   // FM deviation
    const float w_if = 2.0f * M_PI * 12000.0f / 48000.0f; // 12 kHz IF

    // 1) ADC → normalisert audio
    for (int i = 0; i < n; i++) {
        uint32_t raw = in[i] & 0x0FFF;
        audio_in[i] = (((float)raw - 2048.0f) / 2048.0f) * 10.0f; // scale to ±10
    }

    // 2) Audio LPF
    arm_fir_f32(&audio_lpf, audio_in, audio_filtered, n);

    // 3) FM direkte på NCO
    for (int i = 0; i < n; i++) {

        // FM-modulasjon: fase += kf * audio
        phase += w_if + kf * audio_filtered[i];

        if (phase > M_PI) phase -= TWO_PI;
        if (phase < -M_PI) phase += TWO_PI;

        float s = cosf(phase);   // real FM på 12 kHz IF

        out[i] = (uint32_t)((s * 2048.0f) + 2048.0f);
    }
}


void nbfm_init(float if_freq_hz, float fs_hz);

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

    // Hilbert for SSB
    arm_fir_init_f32(&hilbert,
                     HILBERT_TAPS,
                     hilbert_coeffs,
                     hilbert_state,
                     BLOCK_SIZE_MAX/2);

}

// The DAC idles at mid-scale; zero would slam the output to the rail.
#define DAC_MID 2048

static void audio_dma_start(void)
{
    for (uint32_t i = 0; i < BLOCK_SIZE_MAX; i++)
        dac_buffer[i] = DAC_MID;

    memset(adc_buffer, 0, sizeof(adc_buffer));

    block_ready          = 0;
    dac_block_processing = 0;

    adc_rd = 0;

    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, dac_buffer, block_size, DAC_ALIGN_12B_R);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer,
                      mode_is_buffered(MODE) ? ADC_RING_LEN : block_size);
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
    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);

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
void freedv2400b_process_block(const uint16_t *in, uint32_t *out, int n)
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

        out[i] = (uint32_t)y;
    }
}

void process_block(const uint16_t *in, uint32_t *out, int n)
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
                     BLOCK_SIZE_MAX/2);
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
      uart_puts(MODE == MODE_FREEDV_2400B ? "freedv: 2400B RX ready\r\n"
              : MODE == MODE_FREEDV_700D  ? "freedv: 700D RX ready\r\n"
                                          : "freedv: 1600 RX ready\r\n");
  else if (mode_is_freedv(MODE))
      uart_puts("freedv: init FAILED (out of heap?) - audio will be silent\r\n");

  uart_kv("heap_left", (int)heap_largest_free());
  uart_puts("\r\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
    if (HAL_GetTick() - last_report >= 1000)
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
        uart_kv("ring", (int)adc_avail());
        uart_kv("load_pct", (int)load_pct);
        uart_kv("us_max", (int)(rx_cycles_max / 216));
        uart_kv("us_fdv", (int)(rx_fdv_cycles_max / 216));
        uart_kv("adc_x1000", (int)(rx_adc_peak * 1000.0f));
        uart_kv("peak_x1000", (int)(rx_peak * 1000.0f));
        uart_kv("rms_x1000", (int)(rx_rms * 1000.0f));
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
  */
  sConfig.Channel = ADC_CHANNEL_3;
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

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
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
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
