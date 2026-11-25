/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
#define BLOCK_SIZE       32u
#define FULL_BUFFER_SIZE (2u * BLOCK_SIZE)
#define NUM_TAPS         31u

static uint16_t adc_buffer[FULL_BUFFER_SIZE];
static uint16_t dac_buffer[FULL_BUFFER_SIZE];
static float32_t input_f32[BLOCK_SIZE];
static float32_t output_f32[BLOCK_SIZE];
static const float32_t fir_coeffs[NUM_TAPS] = {
    3.6331697815e-18f,
    1.2014263759e-03f,
    2.7843278480e-03f,
    4.2273160854e-03f,
    3.9427634128e-03f,
    -1.1162091159e-17f,
    -8.2567772927e-03f,
    -1.8583210130e-02f,
    -2.5389820435e-02f,
    -2.1235308698e-02f,
    2.7725194169e-17f,
    3.9588112571e-02f,
    9.1888877021e-02f,
    1.4509908103e-01f,
    1.8490287856e-01f,
    1.9966066730e-01f,
    1.8490287856e-01f,
    1.4509908103e-01f,
    9.1888877021e-02f,
    3.9588112571e-02f,
    2.7725194169e-17f,
    -2.1235308698e-02f,
    -2.5389820435e-02f,
    -1.8583210130e-02f,
    -8.2567772927e-03f,
    -1.1162091159e-17f,
    3.9427634128e-03f,
    4.2273160854e-03f,
    2.7843278480e-03f,
    1.2014263759e-03f,
    3.6331697815e-18f
};
static float32_t fir_state[NUM_TAPS + BLOCK_SIZE - 1u];
static arm_fir_instance_f32 S_FIR;
static volatile uint16_t *adc_in_ptr;
static volatile uint16_t *dac_out_ptr;
static void process_half_buffer(void);
/* USER CODE END PV */

/* USER CODE BEGIN 2 */
for (uint32_t i = 0; i < FULL_BUFFER_SIZE; ++i) dac_buffer[i] = 2048u;
arm_fir_init_f32(&S_FIR, NUM_TAPS, fir_coeffs, fir_state, BLOCK_SIZE);
if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, FULL_BUFFER_SIZE) != HAL_OK) Error_Handler();
if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t *)dac_buffer, FULL_BUFFER_SIZE, DAC_ALIGN_12B_R) != HAL_OK) Error_Handler();
if (HAL_TIM_Base_Start(&htim2) != HAL_OK) Error_Handler();
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1) return;
    adc_in_ptr = &adc_buffer[0];
    dac_out_ptr = &dac_buffer[0];
    process_half_buffer();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc != &hadc1) return;
    adc_in_ptr = &adc_buffer[BLOCK_SIZE];
    dac_out_ptr = &dac_buffer[BLOCK_SIZE];
    process_half_buffer();
}

static void process_half_buffer(void)
{
    for (uint32_t i = 0; i < BLOCK_SIZE; ++i) {
        input_f32[i] = ((float32_t)adc_in_ptr[i] - 2048.0f) / 2048.0f;
    }

    arm_fir_f32(&S_FIR, input_f32, output_f32, BLOCK_SIZE);

    for (uint32_t i = 0; i < BLOCK_SIZE; ++i) {
        float32_t sample = output_f32[i] * 2048.0f + 2048.0f;
        if (sample > 4095.0f) sample = 4095.0f;
        if (sample < 0.0f) sample = 0.0f;
        dac_out_ptr[i] = (uint16_t)sample;
    }
}
/* USER CODE END 4 */
