
#include "nrf.h"
#include "nrf_drv_pwm.h"
#include "app_util_platform.h"
#include "nordic_common.h"
#include "nrf_delay.h"

#include "motor.h"

#define MOTORA_IA		23
#define MOTORA_IB		24
#define MOTORB_IA		12
#define MOTORB_IB		22

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

static uint16_t const m_top  = 100;		//20 KHz with PWM base clk of 2 MHz

static nrf_pwm_values_individual_t m_seq_values;
static nrf_pwm_sequence_t const m_seq =
{
    .values.p_individual    = &m_seq_values,
    .length             	= NRF_PWM_VALUES_LENGTH(m_seq_values),
    .repeats             	= 0,
    .end_delay           	= 0
};

static bool m_motors_enabled = false;

void motor_enable(bool enable)
{
	if(enable == false)
	{
		//turn off motors;
		set_motors(0,0);
	}
	m_motors_enabled = enable;
}

void motor_init()
{
	uint32_t err_code;
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            MOTORA_IA | NRF_DRV_PWM_PIN_INVERTED, // channel 0
            MOTORA_IB | NRF_DRV_PWM_PIN_INVERTED, // channel 0
            MOTORB_IA | NRF_DRV_PWM_PIN_INVERTED, // channel 0
            MOTORB_IB | NRF_DRV_PWM_PIN_INVERTED, // channel 0
        },
        .irq_priority = APP_IRQ_PRIORITY_LOW,
        .base_clock   = NRF_PWM_CLK_2MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = m_top,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    err_code = nrf_drv_pwm_init(&m_pwm0, &config0, NULL);
    APP_ERROR_CHECK(err_code);
	
	m_seq_values.channel_0 = 50 | 0x8000;
	m_seq_values.channel_1 = 50 | 0x8000;
	m_seq_values.channel_2 = 50 | 0x8000;
	m_seq_values.channel_3 = 50 | 0x8000;
	
	nrf_drv_pwm_simple_playback(&m_pwm0, &m_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
	
}

//value range from -100 to 100 (max backward to max_forward)
void set_motors(int16_t value_motor_a, int16_t value_motor_b)
{
	if(!m_motors_enabled)
	{
		return;
	}
	value_motor_a = MIN(value_motor_a, 100);
	value_motor_a = MAX(value_motor_a, -100);
	
	value_motor_b = MIN(value_motor_b, 100);
	value_motor_b = MAX(value_motor_b, -100);
	
	//motors does not start to move before about 50% duty cycle
	uint8_t motor_start_value = 40;
	
	if(value_motor_a > 0)
	{
		value_motor_a = value_motor_a * (100 - motor_start_value) / 100 + motor_start_value;
	}
	else
	{
		value_motor_a = value_motor_a * (100 - motor_start_value) / 100 - motor_start_value;
	}
	
	if(value_motor_b > 0)
	{
		value_motor_b = value_motor_b * (100 - motor_start_value) / 100 + motor_start_value;
	}
	else
	{
		value_motor_b = value_motor_b * (100 - motor_start_value) / 100 - motor_start_value;
	}
	
	//apply break by turning on the opposite way the rest of the period
	m_seq_values.channel_0 = (50 + value_motor_a/2) | 0x8000;
	m_seq_values.channel_1 = (50 - value_motor_a/2) | 0x8000;
	
	m_seq_values.channel_2 = (50 + value_motor_b/2) | 0x8000;
	m_seq_values.channel_3 = (50 - value_motor_b/2) | 0x8000;
	
	nrf_drv_pwm_simple_playback(&m_pwm0, &m_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}

void motor_test(void)
{
	m_motors_enabled = true;
	
	for(int8_t i = 0; i < 100; i++)
	{
		set_motors(i,i);
		nrf_delay_ms(50);
	}
	for(int8_t i = 100; i > 0; i--)
	{
		set_motors(i,i);
		nrf_delay_ms(50);
	}
	
	for(int8_t i = 0; i < 100; i++)
	{
		set_motors(-i,-i);
		nrf_delay_ms(50);
	}
	
	for(int8_t i = 100; i > 0; i--)
	{
		set_motors(-i,-i);
		nrf_delay_ms(50);
	}
}
