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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f7xx.h"      // <-- MÅ være først
#include "arm_math.h"       // <-- CMSIS-DSP
#include "dsp.h"            // <-- dine DSP-typer
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
#define BLOCK_SIZE 2048
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
uint32_t dac_buffer[BLOCK_SIZE] = {0};
uint32_t adc_buffer[BLOCK_SIZE] = {0};

volatile uint8_t block_ready = 0; // 0: no block ready, 1: first half ready, 2: second half ready
volatile uint8_t dac_block_processing = 0; // 0: not processing, 1: processing
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







arm_fir_instance_f32 pre_fm_I;
arm_fir_instance_f32 pre_fm_Q;

static float pre_fm_state_I[PRE_DEMOD_TAPS + BLOCK_SIZE/2];
static float pre_fm_state_Q[PRE_DEMOD_TAPS + BLOCK_SIZE/2];

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

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    block_ready = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
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



static float hilbert_state[HILBERT_TAPS + BLOCK_SIZE/2];
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




static float if_state[IF_TAPS + BLOCK_SIZE/2];
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

static float audio_state[AUDIO_TAPS + BLOCK_SIZE/2];
arm_fir_instance_f32 audio_lpf;


#define MODE_NBFM 0
#define MODE_USB  1
#define MODE_LSB  2

int MODE = MODE_NBFM;  // default mode

void ssb_process_block(uint32_t *in, uint32_t *out, int n)
{
    static float I_buf[BLOCK_SIZE/2];
    static float Q_buf[BLOCK_SIZE/2];

    static float IQ_in[BLOCK_SIZE];     // interleaved I/Q
    static float NCO_buf[BLOCK_SIZE];   // interleaved cos/sin
    static float IQ_mix[BLOCK_SIZE];    // interleaved I/Q

    static float audio_buf[BLOCK_SIZE/2];

    // ---------------------------------------------------------
    // 1) ADC → normalisert I
    // ---------------------------------------------------------
    for (int i = 0; i < n; i++)
    {
        uint32_t raw = in[i] & 0x0FFF;
        I_buf[i] = ((float)raw - 2048.0f) / 2048.0f;
    }

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
    for (int i = 0; i < n; i++)
    {
        float cs = cosf(nco.phase);
        float sn = sinf(nco.phase);
        nco_step(&nco);

        NCO_buf[2*i + 0] = cs;
        NCO_buf[2*i + 1] = sn;
    }

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

void nbfm_process_block(uint32_t *in, uint32_t *out, int n)
{
    static float I_if[BLOCK_SIZE];
    static float Q_if[BLOCK_SIZE];

    static float IQ_in[BLOCK_SIZE * 2];    // interleaved I/Q
    static float NCO_buf[BLOCK_SIZE * 2];  // interleaved cos/sin
    static float IQ_mix[BLOCK_SIZE * 2];   // interleaved I/Q etter mixing

    static float audio_fm[BLOCK_SIZE];     // FM-demod audio (float)
    static float audio_lpf_out[BLOCK_SIZE];

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

void nbfm_tx_process_block(uint32_t *in, uint32_t *out, int n)
{
    static float audio_in[BLOCK_SIZE];
    static float audio_filtered[BLOCK_SIZE];
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


void process_block(uint32_t *in, uint32_t *out, int n)
{
    if (MODE == MODE_NBFM)
    {
        nbfm_process_block(in, out, n);
    }
    else
    {
        ssb_process_block(in, out, n);  // din eksisterende SSB-pipe
    }
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
                 BLOCK_SIZE/2);

arm_fir_init_f32(&pre_fm_Q,
                 PRE_DEMOD_TAPS,
                 pre_demod_coeffs,
                 pre_fm_state_Q,
                 BLOCK_SIZE/2);


    // Audio LPF etter FM-demod
    arm_fir_init_f32(&audio_lpf,
                     AUDIO_TAPS,
                     audio_lpf_coeffs,
                     audio_state,
                     BLOCK_SIZE/2);
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

  //nco_init(&nco, 12000.0f, 48000.0f);
  nbfm_init(12000.0f, 48000.0f);

// Kun én init av pre_demod_filter
/*
arm_fir_init_f32(&pre_demod_filter,
                 IF_TAPS,
                 if_coeffs,
                 if_state,
                 BLOCK_SIZE/2);
*/
// Hilbert for SSB
arm_fir_init_f32(&hilbert,
                 HILBERT_TAPS,
                 hilbert_coeffs,
                 hilbert_state,
                 BLOCK_SIZE/2);

                 /*
//Pre FM-demod filter
arm_fir_init_f32(&pre_demod_filter,
                 PRE_DEMOD_TAPS,
                 pre_demod_coeffs,
                 pre_demod_state_I,
                 BLOCK_SIZE/2);

// Audio LPF etter FM-demod
arm_fir_init_f32(&audio_lpf,
                 AUDIO_TAPS,
                 audio_lpf_coeffs,
                 audio_state,
                 BLOCK_SIZE/2);
*/


  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, dac_buffer, BLOCK_SIZE, DAC_ALIGN_12B_R);
  HAL_ADC_Start_DMA(&hadc1, adc_buffer, BLOCK_SIZE);

  //Start TIM6
  HAL_TIM_Base_Start(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    if(dac_block_processing == 1)
    {
        nbfm_tx_process_block(&adc_buffer[0], &dac_buffer[0], BLOCK_SIZE/2);
        dac_block_processing = 0;
    }
    else if(dac_block_processing == 2)
    {
        nbfm_tx_process_block(&adc_buffer[BLOCK_SIZE/2], &dac_buffer[BLOCK_SIZE/2], BLOCK_SIZE/2);
        dac_block_processing = 0;
    }
    /*
    if (block_ready == 1)
    {
      process_block(&adc_buffer[0], &dac_buffer[0], BLOCK_SIZE/2);
      block_ready = 0;
    }
    else if (block_ready == 2)
    {
      process_block(&adc_buffer[BLOCK_SIZE/2], &dac_buffer[BLOCK_SIZE/2], BLOCK_SIZE/2);
      block_ready = 0;
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
