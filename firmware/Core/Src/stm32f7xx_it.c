/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_dac1;
extern DMA_HandleTypeDef hdma_dac2;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern I2C_HandleTypeDef hi2c1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M7 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/*
 * All four fault handlers below used to be the stock silent "while(1){}" -
 * indistinguishable from any other hang. Added 2026-08-05 while chasing a
 * hang that reproduces on completely unrelated code paths (FreeDV 700D
 * decode, plain analog SSB) with nothing in common except "hangs only with
 * a real signal present, console completely dead" - a genuine CPU fault
 * (bad pointer, stack overflow) fits both far better than a mode-specific
 * bug does, and there was no way to tell without this. Dumps the stacked
 * registers and SCB fault-status registers over the same blocking UART
 * path freedv_chain.c's codec2 allocator instrumentation uses, since
 * uart_puts() alone would just queue bytes nothing is left running to
 * drain. PC is the important one - it's the address of the instruction
 * that actually faulted.
 */
extern void uart_puts(const char *s);
extern void uart_flush_blocking(void);

static void uart_put_hex32(uint32_t v)
{
  char buf[9];
  buf[8] = '\0';
  for (int i = 7; i >= 0; i--)
  {
    uint32_t nib = v & 0xFu;
    buf[i] = (char)(nib < 10 ? ('0' + nib) : ('A' + nib - 10));
    v >>= 4;
  }
  uart_puts(buf);
}

static void uart_put_hex32_kv(const char *key, uint32_t v)
{
  uart_puts(key);
  uart_puts("=0x");
  uart_put_hex32(v);
  uart_puts("\r\n");
}

void hard_fault_handler_c(uint32_t *stacked)
{
  uart_puts("\r\n\r\n!!! HARD FAULT !!!\r\n");
  uart_put_hex32_kv("R0",  stacked[0]);
  uart_put_hex32_kv("R1",  stacked[1]);
  uart_put_hex32_kv("R2",  stacked[2]);
  uart_put_hex32_kv("R3",  stacked[3]);
  uart_put_hex32_kv("R12", stacked[4]);
  uart_put_hex32_kv("LR",  stacked[5]);
  uart_put_hex32_kv("PC",  stacked[6]);
  uart_put_hex32_kv("PSR", stacked[7]);
  uart_put_hex32_kv("CFSR",  SCB->CFSR);
  uart_put_hex32_kv("HFSR",  SCB->HFSR);
  uart_put_hex32_kv("MMFAR", SCB->MMFAR);
  uart_put_hex32_kv("BFAR",  SCB->BFAR);
  uart_puts("!!! end fault dump !!!\r\n");
  uart_flush_blocking();
  while (1) { }
}

/* USER CODE END 1 */

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  __asm volatile
  (
    " tst lr, #4                \n"
    " ite eq                    \n"
    " mrseq r0, msp              \n"
    " mrsne r0, psp              \n"
    " ldr r1, =hard_fault_handler_c \n"
    " bx r1                      \n"
  );
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  uart_puts("\r\n!!! MemManage fault !!!\r\n");
  uart_put_hex32_kv("CFSR", SCB->CFSR);
  uart_put_hex32_kv("MMFAR", SCB->MMFAR);
  uart_flush_blocking();
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  uart_puts("\r\n!!! Bus fault !!!\r\n");
  uart_put_hex32_kv("CFSR", SCB->CFSR);
  uart_put_hex32_kv("BFAR", SCB->BFAR);
  uart_flush_blocking();
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */
  uart_puts("\r\n!!! Usage fault !!!\r\n");
  uart_put_hex32_kv("CFSR", SCB->CFSR);
  uart_flush_blocking();
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_dac1);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt (DAC ch2, TX IF out).
  */
void DMA1_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_dac2);
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt (ADC2, TX mic in).
  */
void DMA2_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc2);
}

/**
  * @brief This function handles USB OTG FS global interrupt (USB Audio).
  */
void OTG_FS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

/**
  * @brief This function handles I2C1 event interrupt (OLED, non-blocking).
  */
void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c1);
}

/**
  * @brief This function handles I2C1 error interrupt (OLED, non-blocking).
  */
void I2C1_ER_IRQHandler(void)
{
  HAL_I2C_ER_IRQHandler(&hi2c1);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
