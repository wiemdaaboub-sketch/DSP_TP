/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
#define BLOCK_SIZE       32u
#define FULL_BUFFER_SIZE (2u * BLOCK_SIZE)
#define NUM_STAGES       1u

static uint16_t adc_buffer[FULL_BUFFER_SIZE];
static uint16_t dac_buffer[FULL_BUFFER_SIZE];
static float32_t input_f32[BLOCK_SIZE];
static float32_t output_f32[BLOCK_SIZE];
static const float32_t iir_coeffs[5u * NUM_STAGES] = {
    6.7455273889e-02f,
    1.3491054778e-01f,
    6.7455273889e-02f,
    1.1429805025e+00f,
    -4.1280159810e-01f
};
static float32_t iir_state[4u * NUM_STAGES];
static arm_biquad_casd_df1_inst_f32 S_IIR;
static volatile uint16_t *adc_in_ptr;
static volatile uint16_t *dac_out_ptr;
static void process_half_buffer(void);
/* USER CODE END PV */

/* USER CODE BEGIN 2 */
for (uint32_t i = 0; i < FULL_BUFFER_SIZE; ++i) dac_buffer[i] = 2048u;
arm_biquad_cascade_df1_init_f32(&S_IIR, NUM_STAGES, iir_coeffs, iir_state);
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

    arm_biquad_cascade_df1_f32(&S_IIR, input_f32, output_f32, BLOCK_SIZE);

    for (uint32_t i = 0; i < BLOCK_SIZE; ++i) {
        float32_t sample = output_f32[i] * 2048.0f + 2048.0f;
        if (sample > 4095.0f) sample = 4095.0f;
        if (sample < 0.0f) sample = 0.0f;
        dac_out_ptr[i] = (uint16_t)sample;
    }
}
/* USER CODE END 4 */
