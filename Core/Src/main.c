/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : FFT tabanlı frekans bandı tespiti ve LED kontrolü
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "arm_math.h"  // CMSIS DSP Kütüphanesinde bulununa FFT'nin çalışması için gerekli header dosyası
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define FFT_SIZE     1024U           //Fast Fourier Transform örnekleme -- arttırılabilir ancak nu değer idealdir
#define SAMPLE_RATE  8000.0f         // Örnekleme oranı -- 8000 / 1024 =~ 7.8125 çözünürlük
#define SMOOTH_ALPHA 0.20f           // Smooth alfa degeri ( orta) geçmiş değerleri kullanma ağırlıklı %80 - %20
#define PI           3.14159265358979f

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim2;

static uint16_t  adc_buffer[FFT_SIZE];     // DMA ham örnekler
static float32_t time_buf [FFT_SIZE];      // gerçek giriş dizisi
static float32_t fft_out  [FFT_SIZE];      // RFFT çıktısı (packed)
static float32_t mag      [FFT_SIZE/2];    // Genlikler

static float32_t smoothB[4] = {0};    //Smooth dizisi

volatile uint8_t  data_ready   = 0;   //ADC 'nin DMA bufferını doldurduğuna dair flag
volatile float32_t detectedFreq = 0.0f;  // Tespit edilen frekans değerine baslangic atandı

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void); //Timer2 ayarlanmıştır. 1/8000 = 0.125 us de bir timer tetiklemesi gerçekleşir.Bu durum ADC'nin continuous değer çekmesinden daha güvenli bir yöntemdir.
void Error_Handler(void);

/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) data_ready = 1; // DMA dolduruluduğunda flag = 1
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();

  HAL_TIM_Base_Start(&htim2);
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, FFT_SIZE) != HAL_OK)
      Error_Handler();

  //CONTINUOUS LOOP
  while (1)
  {
    if (!data_ready) continue;
    data_ready = 0;
    HAL_TIM_Base_Stop(&htim2);

    /* 1. DC offset ve Hamming pencere ------------------------------------ */
    uint32_t sum = 0;
    for (uint16_t i = 0; i < FFT_SIZE; ++i) sum += adc_buffer[i];
    const float32_t mean = (float32_t)sum / FFT_SIZE;

    for (uint16_t i = 0; i < FFT_SIZE; ++i)
    {
      float32_t x = (float32_t)adc_buffer[i] - mean;               // DC çıkar(Ölçülen - ortalama)
      float32_t w = 0.54f - 0.46f * cosf(2.0f*PI*i/(FFT_SIZE-1));  // Hamming  işlemi
      time_buf[i] = x * w;                                         //  gerçek dizi
    }

   // FFT Dönüşümleri sağlayan dosya girdileri
    arm_rfft_fast_instance_f32 S;
    arm_rfft_fast_init_f32(&S, FFT_SIZE);
    arm_rfft_fast_f32(&S, time_buf, fft_out, 0);

    /* 3. Genlik ----------------------------------------------------------- */
    mag[0] = fabsf(fft_out[0]);               // DC
    for (uint16_t k = 1; k < FFT_SIZE/2; ++k)
    {
      float32_t re = fft_out[2*k];
      float32_t im = fft_out[2*k + 1];
      mag[k] = sqrtf(re*re + im*im);    //Büyüklük = karekök (re^2 + im^2)
    }

    /* 4. En yüksek bin & gerçek frekans ---------------------------------- */
    uint32_t maxIdx = 1; float32_t maxVal = mag[1];
    for (uint16_t k = 2; k < FFT_SIZE/2; ++k)
      if (mag[k] > maxVal) { maxVal = mag[k]; maxIdx = k; }

    detectedFreq = (float32_t)maxIdx * SAMPLE_RATE / FFT_SIZE;

    /* 5. Bant maksimumları (20-250 / 251-500 / 501-750 / 751-1000 Hz) ---- */
    /*250/7.8125 = 32 , 500/7.8125 = 64 , 750/7.8125 = 96 , 1000/7.8125 = 128 */
    const uint16_t binStart[4] = {  3,  33,  65,  97 }; // frequency / ( sapmle rate / fft size) sonucları
    const uint16_t binEnd  [4] = { 32,  64,  96, 128 };
    float32_t maxB[4] = {0};

    for (uint8_t b = 0; b < 4; ++b)
      for (uint16_t k = binStart[b]; k <= binEnd[b]; ++k)
        if (mag[k] > maxB[b]) maxB[b] = mag[k];

    // 6. Smoothing İslemi
    for (uint8_t b = 0; b < 4; ++b)
      smoothB[b] = SMOOTH_ALPHA*maxB[b] + (1.0f-SMOOTH_ALPHA)*smoothB[b];

    uint8_t band = 0; float32_t highest = smoothB[0];
    for (uint8_t b = 1; b < 4; ++b)
      if (smoothB[b] > highest) { highest = smoothB[b]; band = b; }

    // 7. LED KONTROL
    if (highest < 100.0f)   // gürültü eşiği - noise threshold
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_All & 0xF000, GPIO_PIN_RESET);
    else
    {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, (band==0)?GPIO_PIN_SET:GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, (band==1)?GPIO_PIN_SET:GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, (band==2)?GPIO_PIN_SET:GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, (band==3)?GPIO_PIN_SET:GPIO_PIN_RESET);
    }

    // 8. Yeni blok
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, FFT_SIZE) != HAL_OK)
        Error_Handler();
    HAL_TIM_Base_Start(&htim2);
  }
}

/* USER CODE BEGIN 4 */
void Error_Handler(void)
{
  HAL_GPIO_WritePin(GPIOD,
      GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,
      GPIO_PIN_RESET);
  __disable_irq();
  while (1) {}
}
/* USER CODE END 4 */

/* Rest of MX_* and SystemClock_Config() unchanged... */


/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef       RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef       RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM            = 4;
  RCC_OscInitStruct.PLL.PLLN            = 168;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ            = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

  RCC_ClkInitStruct.ClockType           = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK
                                        |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider       = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV4;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin   = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = GPIO_PIN_1;
  GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();
  hdma_adc1.Instance                 = DMA2_Stream0;
  hdma_adc1.Init.Channel             = DMA_CHANNEL_0;
  hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode                = DMA_NORMAL;
  hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void MX_ADC1_Init(void)
{
  __HAL_RCC_ADC1_CLK_ENABLE();
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance                   = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode          = DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
  sConfig.Channel      = ADC_CHANNEL_1;
  sConfig.Rank         = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_ClockConfigTypeDef  sClockConfig  = {0};
  htim2.Instance           = TIM2;
  htim2.Init.Prescaler     = 83;
  htim2.Init.CounterMode   = TIM_COUNTERMODE_UP;
  htim2.Init.Period        = 124;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
  sClockConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockConfig) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}
