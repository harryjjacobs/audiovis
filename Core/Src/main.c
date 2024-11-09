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

#include "utils.h"

#include "colour.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>

#define ARM_MATH_CM4 1
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// Make printf work:
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

// how many LEDs are there in the strip
#define LED_COUNT 30

#define ADC_BUF_LEN 1024

#define FFT_BUF_LEN 512 // TODO: try increasing this

// visualisation
#define FFT_FREQUENCY_MIN 50.0     // the lower frequency bound we care about
#define FFT_FREQUENCY_MAX 1500.0    // the upper frequency bound we care about

#define SPECTRUM_LEN FFT_BUF_LEN / 2 - 1  // -1 because we ignore the first frequency bin
#define SPECTRUM_MAGNITUDE_MIN 0.001
#define SPECTRUM_MAGNITUDE_MAX 1.0
#define LED_FADE_RATE 2.5 // rate at which to reduce the magnitude to zero when there's no new signal (1/s)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// ==== LED stuff ====
uint8_t led_data[LED_COUNT][4];
// 24 = 1 byte each for R, G, and B, 50 is empty space at end to indicate end of data
uint16_t pwm_data[(24 * LED_COUNT) + 50];
volatile uint8_t pwm_dma_sent_flag = 0;

// ==== Audio sampling ====
uint16_t adc_buffer[ADC_BUF_LEN];
volatile uint8_t adc_half_complete_flag = 0;

// ==== FFT calculation ====

arm_rfft_fast_instance_f32 fft_handler;

// array of FFT results, in the form [real, imaginary, real, imaginary, ...]
float32_t fft_result[FFT_BUF_LEN];

// array of the the real components to perform the fft on
float32_t fft_buffer[FFT_BUF_LEN];
uint32_t fft_buffer_index = 0;

// indicates that an FFT has been performed on the fft_buffer and fft_results is now populated
volatile uint8_t fft_complete_flag = 0;

typedef struct spectrum_t {
  float32_t power[SPECTRUM_LEN];
  float32_t frequency[SPECTRUM_LEN];
  float32_t frequency_bin_width;
  float32_t power_sum;
  size_t max_index;
} spectrum_t;

typedef void (*spectrum_colour_palette_t) (int, float32_t, int *, int *, int *);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

void ws2812_send(void) {
  uint32_t indx = 0;
  uint32_t color;

  for (int i = 0; i < LED_COUNT; i++) {
    color = ((led_data[i][1] << 16) | (led_data[i][2] << 8) | (led_data[i][3]));
    for (int i = 23; i >= 0; i--) {
      if (color & (1 << i)) { // if current bit is set, set the pwm duty LONG, else SHORT
        pwm_data[indx] = 48; // 0.8us. 0.64. Full CCR == 800 KHz == 1.25us (time period of led pwm signal)
      } else {
        pwm_data[indx] = 24;  // 0.4us. 0.32.
      }
      indx++;
    }
  }

  for (int i = 0; i < 50; i++) {
    pwm_data[indx] = 0;
    indx++;
  }

  if (HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t*) pwm_data, 24 * LED_COUNT + 50) != HAL_OK) {
    Error_Handler();
  }
  while (!pwm_dma_sent_flag) {
  };
  pwm_dma_sent_flag = 0;
}

// set the led colour channel values
void set_led(uint8_t i, int red, int green, int blue) {
  led_data[i][0] = i;
  led_data[i][1] = green;
  led_data[i][2] = red;
  led_data[i][3] = blue;
}

// increase the led colour channel values
void add_led(uint8_t i, int red, int green, int blue) {
  led_data[i][0] = i;
  led_data[i][1] = max(0, min(255, led_data[i][1] + green));
  led_data[i][2] = max(0, min(255, led_data[i][2] + red));
  led_data[i][3] = max(0, min(255, led_data[i][3] + blue));
}

// turn all the leds off
void reset_leds() {
  for (uint8_t i = 0; i < LED_COUNT; i++) {
    set_led(i, 0, 0, 0);
  }
}

// performs a groovy initialisation sequence for the led strip
//void led_init_sequence_1() {
//  for (int i = 0; i < LED_COUNT; i++) {
//    set_led(i, (i % 1) == 0 ? 255 : 0, (i % 2) == 0 ? 255 : 0, (i % 3) == 0 ? 255 : 0);
//    HAL_Delay(10);
//    ws2812_send();
//  }
//  ws2812_send();
//  for (int i = 0; i < LED_COUNT; i++) {
//    set_led(i, 0, 0, 0);
//    HAL_Delay(10);
//    ws2812_send();
//  }
//}

void spectrum_colour_palette_1(int led_index, float32_t magnitude, int *red, int *green, int *blue) {
  float brightness = min(1.0f, (magnitude / SPECTRUM_MAGNITUDE_MAX)) * 0.5f; // brightness 0..0.5
  float distance = ((double)led_index / LED_COUNT); // how far along the strip are we? 0..1
  distance = distance * 0.25f; // scale

  // this shifts the colour spectrum along over time at the given speed
  static float colour_offset = 0.0f;
  const float speed = 0.1f;
  colour_offset = (HAL_GetTick() / 1000.0f) * speed;

  float h = (arm_cos_f32(2 * PI * distance + colour_offset) + 1.0) / 2.0;
  float s = 1.0;
  float l = brightness;

  RGB rgb = hsl2rgb(h, s, l);

  *red = rgb.r;
  *green = rgb.g;
  *blue = rgb.b;
}

double linear(double v) {
  return v;
}

/**
 * Maps the frequency from the FFT to the closest LEDs in the strip
 */
void map_frequency_to_leds(spectrum_t *spectrum, size_t index, uint8_t **leds, uint8_t *count) {
  float32_t f = spectrum->frequency[index];
  if (f < FFT_FREQUENCY_MIN || f > FFT_FREQUENCY_MAX) {
    *leds = NULL;
    return;
  }
  const double (*mapping)(double) = &linear;
  const double frequency_range = (mapping(FFT_FREQUENCY_MAX) - mapping(FFT_FREQUENCY_MIN));
  // how far along the strip is this frequency band
  double f_proportion = (mapping(f) - mapping(FFT_FREQUENCY_MIN)) / frequency_range;
  // how far along the strip is the next frequency band
  double next_f_proportion = (mapping(f + spectrum->frequency_bin_width) - mapping(FFT_FREQUENCY_MIN)) / frequency_range;

  uint8_t start = (int8_t)((LED_COUNT - 1) * f_proportion);
  uint8_t end = min(LED_COUNT - 1, max(start + 1, (int8_t)((LED_COUNT - 1) * next_f_proportion)));
  if (start >= end) {
    *leds = NULL;
    return;
  }
  *count = end - start;
  *leds = malloc((size_t)(*count * sizeof(uint8_t)));
  for (uint8_t i = start, j = 0; i < end; i++, j++) {
    (*leds)[j] = i;
  }
}

/**
 * Maps the frequency from the FFT to the closest LEDs in the strip going outwards from the central LED
 * symmetrically.
 */
void map_frequency_to_leds_mirrored(spectrum_t *spectrum, size_t index, uint8_t **leds, uint8_t *count) {
  float32_t f = spectrum->frequency[index];
  f = min(FFT_FREQUENCY_MAX, f);
  f = max(FFT_FREQUENCY_MIN, f);
  const double (*mapping)(double) = &linear;
  const double frequency_range = (mapping(FFT_FREQUENCY_MAX) - mapping(FFT_FREQUENCY_MIN));
  // how far along the strip is this frequency band
  double f_proportion = (mapping(f) - mapping(FFT_FREQUENCY_MIN)) / frequency_range;
  // how far along the strip is the next frequency band
  double next_f_proportion = (mapping(f + spectrum->frequency_bin_width) - mapping(FFT_FREQUENCY_MIN)) / frequency_range;

  uint8_t start = (int8_t)((LED_COUNT / 2 - 1) * f_proportion);
  uint8_t end = min(LED_COUNT / 2 - 1, max(start + 1, (int8_t)((LED_COUNT / 2 - 1) * next_f_proportion)));
  if (start >= end) {
    *leds = NULL;
    return;
  }
  *count = (end - start) * 2;
  *leds = malloc((size_t)(*count * sizeof(uint8_t)));
  for (uint8_t i = start, j = 0; i < end; i++, j+=2) {
    (*leds)[j] = LED_COUNT / 2 - i;
    (*leds)[j + 1] = LED_COUNT / 2 - 2 + i;
  }
}

void perform_fft() {
//  int before = HAL_GetTick();
  arm_rfft_fast_f32(&fft_handler, fft_buffer, fft_result, 0);
//  int elapsed = HAL_GetTick() - before;
//  printf("elapsed: %d: ", elapsed);
  fft_complete_flag = 1;
}

void process_adc(uint8_t half_complete_flag) {
  if (half_complete_flag == adc_half_complete_flag) {
    // buffer overrun - we didn't process quickly enough
    // Error_Handler();
    return; // ignore this for now for debugging purposes
  }
  uint32_t offset = 0;
  if (half_complete_flag == 0) {
    // second half of the data has finished
    offset = ADC_BUF_LEN / 2;
  }
  // copy into the fft buffer
  for (uint32_t i = offset; i < offset + ADC_BUF_LEN / 2; i++) {
    fft_buffer[fft_buffer_index] = (float)adc_buffer[i] / 65535;
    fft_buffer_index++;
    if (fft_buffer_index == FFT_BUF_LEN) {
      fft_buffer_index = 0;
      perform_fft();
    }
  }
  adc_half_complete_flag = half_complete_flag;
}

void collect_fft_result(spectrum_t *spectrum) {
  float peak_magnitude = 0.0f;
  spectrum->power_sum = 0.0f;
  for (uint32_t i = 1; i < FFT_BUF_LEN - 1; i+=2) { // skip band 1
    float magnitude_sqrd = fft_result[i] * fft_result[i] + fft_result[i + 1] * fft_result[i + 1];
    spectrum->power[i / 2 - 1] = magnitude_sqrd;
    spectrum->power_sum += magnitude_sqrd;
    if (magnitude_sqrd > peak_magnitude) {
      peak_magnitude = magnitude_sqrd;
      spectrum->max_index = i / 2 - 1;
    }
  }
}

// HAL callbacks

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim1) {
    if (HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1) != HAL_OK) {
      Error_Handler();
    }
    pwm_dma_sent_flag = 1;
  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc == &hadc2) {
    process_adc(1);
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc == &hadc2) {
    process_adc(0);
  }
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
  MX_USART2_UART_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // Initialise FFT handler
  arm_rfft_fast_init_f32(&fft_handler, FFT_BUF_LEN);

  // ADC timer
  HAL_TIM_Base_Start(&htim2);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*) &adc_buffer, ADC_BUF_LEN);

  double fft_sample_rate = (double)HAL_RCC_GetSysClockFreq() / (htim2.Init.Period + 1);
  printf("...\n\n\n\n\n");
  printf("FFT sample rate: %f\r\n", fft_sample_rate);

  spectrum_t spectrum;
  spectrum.frequency_bin_width = fft_sample_rate / FFT_BUF_LEN;
  // pre-calculate the frequencies
  for (uint32_t i = 1; i < FFT_BUF_LEN / 2; i++) { // skip frequency band 1
    spectrum.frequency[i - 1] = (float)(i) * spectrum.frequency_bin_width;
  }

  spectrum_colour_palette_t colour_palette = &spectrum_colour_palette_1;

  // Disabled for now to save flash storage space
//  led_init_sequence_1();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int last_tick = HAL_GetTick();
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (fft_complete_flag) {
      fft_complete_flag = 0;

      // populate the spectrum struct
      collect_fft_result(&spectrum);

      // dynamic sensitivity (normalise the power)
//      float normalisation_factor = 0.001f * (FFT_BUF_LEN / 2) / (max(0.004, spectrum.power_sum));

      for (uint32_t i = 0; i < SPECTRUM_LEN; i++) {
//        spectrum.power[i] *= normalisation_factor;
        uint8_t* leds = NULL;
        uint8_t count;
        map_frequency_to_leds_mirrored(&spectrum, i, &leds, &count);
        if (spectrum.power[i] >= SPECTRUM_MAGNITUDE_MIN) {
          int r;
          int g;
          int b;
          colour_palette(i, spectrum.power[i], &r, &g, &b);
          for (uint8_t i = 0; i < count; i++) {
            add_led(leds[i], r, g, b);
          }
        }
        if (leds != NULL) {
          free(leds);
        }
      }
//      printf("%.2f, %.2f\r\n", spectrum.frequency[spectrum.max_index], spectrum.power[spectrum.max_index]);
    }

    // fade out
    double dt = HAL_GetTick() / 1000.0 - last_tick / 1000.0;
    for (uint32_t i = 0; i < LED_COUNT; i++) {
      add_led(i, -LED_FADE_RATE * dt * 255, -LED_FADE_RATE * dt * 255, -LED_FADE_RATE * dt * 255);
    }

    // update the leds
    ws2812_send();

    last_tick = HAL_GetTick();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 15;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 74;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1814;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
  char *error_msg = "error\n";
  HAL_UART_Transmit(&huart2, (uint8_t *) error_msg, strlen(error_msg) / sizeof(char), HAL_MAX_DELAY);
  while (1) {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    HAL_Delay(1000); // ms
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000); // ms
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
