/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "hrtim.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HRTIM_FREQ    5440000000

#define CONST_ADJ     4

#define FREQ_IDLE     HRTIM_FREQ / (100000 * CONST_ADJ)
#define MODULATE_ZERO HRTIM_FREQ / (103000 * CONST_ADJ)
#define MODULATE_ONE  HRTIM_FREQ / (105000 * CONST_ADJ)

uint32_t idle = 101000;
uint32_t low = 103000;
uint32_t high = 105000;

#define B(x)          S_to_binary_(#x)

static inline unsigned long long S_to_binary_(const char *s)
{
    unsigned long long i = 0;
    while (*s) {
        i <<= 1;
        i += *s++ - '0';
    }
    return i;
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t PERIOD                  = 0x308C;
uint16_t adc_value               = 0;
volatile uint16_t adc_value_prev = 0;
static uint16_t dutyCycle        = 50;
static uint16_t test_value       = 0;

typedef enum STATE {
    STATE_STOP,
    STATE_START,
} STATES;

struct ADC_t {
    uint16_t value;
    uint16_t low_threshold;
    uint16_t high_threshold;
    uint16_t degrees_conv;
};

struct ADC_t adc = {.value = 0, .low_threshold = 2000, .high_threshold = 3900, .degrees_conv = 22};

uint16_t flag     = 0;
uint8_t freq_flag = 0;

uint8_t max_bits_count         = 8;
volatile uint8_t bit_send_flag = 0;

enum STATE state = STATE_START;

char word_to_send[20] = {
    0,
};

uint64_t hrtim_freq = 5440000000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int dec2bin(int num);
void hrtim_rebase_freq(uint32_t frequency_to_rebase);
void send_digits(int num, int digits);
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

  /* USER CODE END 1 */

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
  MX_LPUART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_HRTIM1_Init();
  /* USER CODE BEGIN 2 */

    HAL_ADC_Start(&hadc1);
    /* TIMA counter operating in continuous mode with prescaler = 010b (div.
 by 4) */
    /* Preload enabled on REP event*/
    PERIOD = FREQ_IDLE; // BASE;//0x3B5E;//(uint16_t)(hrtim_freq / base_freq);
    // Real phase shift must be between PI/2 and PI
    // TODO: Saturate low phase shift at PI/2
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].TIMxCR = HRTIM_TIMCR_CONT +
                                                           HRTIM_TIMCR_PREEN + HRTIM_TIMCR_TREPU + HRTIM_TIMCR_CK_PSC_1;
    /* Set period to 33kHz and duty cycles to 25% */
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].PERxR  = PERIOD; //;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = PERIOD / 2;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP2xR = PERIOD / 8;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP3xR =
        PERIOD / 2 + PERIOD / 8;
    /* TA1 output set on TIMA period and reset on TIMA CMP1 event*/
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].SETx1R = HRTIM_SET1R_PER;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].RSTx1R = HRTIM_RST1R_CMP1;

    /* TA2 output set on TIMA CMP2 and reset on TIMA period event*/
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].SETx2R = HRTIM_SET2R_CMP2;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].RSTx2R = HRTIM_RST2R_CMP3;

    /* Enable TA1, TA2, TD1 and TD2 outputs */
    HRTIM1->sCommonRegs.OENR = HRTIM_OENR_TA1OEN + HRTIM_OENR_TA2OEN +
                               HRTIM_OENR_TD1OEN + HRTIM_OENR_TD2OEN;
    /* Start Timer A and Timer D */
    HRTIM1->sMasterRegs.MCR = HRTIM_MCR_TACEN + HRTIM_MCR_TDCEN;

    HAL_UART_Receive_IT(&hlpuart1, (uint8_t *)&freq_flag, 1);
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Start\n", 8, 1000);
    //HAL_TIM_Base_Start_IT(&htim1);
    // HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_2);
    // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        if (state == STATE_START) {
            /* set new tim CCRx registers based on ADC value */
            HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP2xR =
                (PERIOD / 360 * (adc.value / adc.degrees_conv));
            HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP3xR =
                (PERIOD / 2 + PERIOD / 360 * (adc.value / adc.degrees_conv));
            adc.value = HAL_ADC_GetValue(&hadc1);
            if (adc.value > adc.high_threshold) {
                adc.value = adc.high_threshold;
            } else if (adc.value < adc.low_threshold) {
                adc.value = adc.low_threshold;
            }
            send_digits(dec2bin(10), 4);
            HAL_Delay(100);
            hrtim_rebase_freq(100000);
            HAL_Delay(500);
            send_digits(dec2bin(15), 4);
            HAL_Delay(100);
            hrtim_rebase_freq(100000);
            HAL_Delay(500);
            HAL_ADC_Start(&hadc1);
        }

        if (flag == 1) {
            /* push button pressed*/
            flag  = 0;
            state = !state; // toggle state
            if (state == STATE_STOP) {
                /* set HRTIM CCRx to 0 */
                HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = 0;
                HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP2xR = 0;
                HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP3xR = 0;
                HAL_ADC_Stop(&hadc1);
                CLEAR_BIT(GPIOA->ODR, GPIO_PIN_5);
            } else if (state == STATE_START) {
                /* set HRTIM CCRx to half period to reinit phase shift*/
                HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = PERIOD / 2;
                HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP2xR = 0;
                HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP3xR = 0;
                SET_BIT(GPIOA->ODR, GPIO_PIN_5);
                HAL_ADC_Start(&hadc1);
            }
        }
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// add push button callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13) {
        flag = 1;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        if(freq_flag == 1)
        {
          freq_flag = 0;
          hrtim_rebase_freq(105000);
          GPIOA->ODR ^= (1 << 5);
        }
        else if(freq_flag == 0)
        {
          freq_flag = 1;
          hrtim_rebase_freq(100000);
         GPIOA->ODR ^= (1 << 5);
        }
        
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1) {
        if (freq_flag == 79) // O is for over (frequency is equal to 110kHz)
        {
            freq_flag                                            = 0;
            PERIOD                                               = MODULATE_ONE;
            HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].PERxR  = PERIOD; //;
            HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = PERIOD / 2;
            HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP2xR = 0;
            HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP3xR = 0;
            HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Overdrive\n", 10, 100);
        }

        else if (freq_flag == 66) // B is for base (frequency is equal to 85kHz)
        {
            freq_flag                                            = 0;
            PERIOD                                               = MODULATE_ZERO;
            HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].PERxR  = PERIOD; //;
            HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = PERIOD / 2;
            HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP2xR = 0;
            HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP3xR = 0;
            HAL_UART_Transmit(&hlpuart1, (uint8_t *)"Base\n", 4, 100);
        }
        /* Start Timer A and Timer D */
        HRTIM1->sMasterRegs.MCR = HRTIM_MCR_TACEN;
    }
    HAL_UART_Receive_IT(&hlpuart1, (uint8_t *)&freq_flag, 1);
}

/**
 * @brief Takes decimal number and converts it to binary
 *
 * @param num
 * @return int
 */
int dec2bin(int num)
{
    int bin = 0, k = 1;

    while (num) {
        bin += (num % 2) * k;
        k *= 10;
        num /= 2;
    }

    return bin;
}

/**
 * @brief takes binary number and sends it to UART in polling mode
 * 
 *
 * @param num
 * @param digits
 */
void send_digits(int num, int digits)
{
    bit_send_flag = 0;
    if (digits > 0) {

        send_digits(num / 10, digits - 1);
        sprintf(word_to_send, "%d\n", num % 10);
        if (num % 10 == 0) {
            hrtim_rebase_freq(low);
            GPIOA -> ODR &= ~(1 << 5);
            for (uint32_t i = 0; i < 1000000; i++) {
                __NOP();
            }
        } else if (num % 10 == 1) {
            GPIOA -> ODR |= (1 << 5);
            hrtim_rebase_freq(high);
            for (uint32_t i = 0; i < 1000000; i++) {
                __NOP();
            }

            /* Start Timer A and Timer D */
            // __HAL_TIM_CLEAR_FLAG(&htim1, TIM_SR_UIF); // очищаем флаг
        }
        HAL_UART_Transmit(&hlpuart1, (uint8_t *)word_to_send, strlen(word_to_send), 100);
    } 
    else if (digits == 0) {
      hrtim_rebase_freq(idle);
    }
    
}

void hrtim_rebase_freq(uint32_t frequency_to_rebase)
{
    uint16_t local_PERIOD = HRTIM_FREQ / (frequency_to_rebase * CONST_ADJ);
    PERIOD = local_PERIOD;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].PERxR  = local_PERIOD; //;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = local_PERIOD / 2;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP2xR = 0;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP3xR = 0;

    //HRTIM1->sMasterRegs.MCR = HRTIM_MCR_TACEN;
}
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
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
