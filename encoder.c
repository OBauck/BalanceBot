
#include <string.h>

#include "nrf.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_gpiote.h"
#include "app_util_platform.h"
#include "SEGGER_RTT.h"

#define MOTOR_A_PHASE_A_PIN		25
#define MOTOR_A_PHASE_B_PIN		2
#define MOTOR_B_PHASE_A_PIN		27
#define MOTOR_B_PHASE_B_PIN		26

#define TIMER_INSTANCE_A	2
#define TIMER_INSTANCE_B	3

const nrf_drv_timer_t encoder_a_timer = NRF_DRV_TIMER_INSTANCE(TIMER_INSTANCE_A);
const nrf_drv_timer_t encoder_b_timer = NRF_DRV_TIMER_INSTANCE(TIMER_INSTANCE_B);

static bool encoder_AA_state = 0;
static bool encoder_AB_state = 0;
static bool encoder_BA_state = 0;
static bool encoder_BB_state = 0;
static int32_t encoder_values[2];

static void timer_dummy_handler(nrf_timer_event_t event_type, void* p_context)
{
}

//setup timers in counter mode
static void timer_setup(void)
{
	uint32_t err_code;
	
	//timer config is defined in nrf_drv_config.h
	nrf_drv_timer_config_t timer_config_a = NRF_DRV_TIMER_DEFAULT_CONFIG(TIMER_INSTANCE_A);
	
	err_code = nrf_drv_timer_init(&encoder_a_timer, &timer_config_a, timer_dummy_handler);
	APP_ERROR_CHECK(err_code);
	
	nrf_drv_timer_enable(&encoder_a_timer);
	
	//timer config is defined in nrf_drv_config.h
	nrf_drv_timer_config_t timer_config_b = NRF_DRV_TIMER_DEFAULT_CONFIG(TIMER_INSTANCE_B);
	
	err_code = nrf_drv_timer_init(&encoder_b_timer, &timer_config_b, timer_dummy_handler);
	APP_ERROR_CHECK(err_code);
	
	nrf_drv_timer_enable(&encoder_b_timer);
}

static void gpiote_event_handlerAA(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	encoder_AA_state = !encoder_AA_state;

	encoder_values[0] -= (encoder_AA_state ^ encoder_AB_state);
	encoder_values[0] += (encoder_AA_state & encoder_AB_state);
}

static void gpiote_event_handlerAB(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	encoder_AB_state = !encoder_AB_state;

	encoder_values[0] += (encoder_AA_state ^ encoder_AB_state);
	encoder_values[0] -= (encoder_AA_state & encoder_AB_state);
}

static void gpiote_event_handlerBA(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	encoder_BA_state = !encoder_BA_state;

	encoder_values[1] -= (encoder_BA_state ^ encoder_BB_state);
	encoder_values[1] += (encoder_BA_state & encoder_BB_state);
}

static void gpiote_event_handlerBB(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
	encoder_BA_state = !encoder_BA_state;

	encoder_values[1] += (encoder_BA_state ^ encoder_BB_state);
	encoder_values[1] -= (encoder_BA_state & encoder_BB_state);
}

static void gpiote_setup(void)
{
	uint32_t err_code;
	
	if(!nrf_drv_gpiote_is_init())
	{
		err_code = nrf_drv_gpiote_init();
		APP_ERROR_CHECK(err_code);
	}
	
	nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);

    err_code = nrf_drv_gpiote_in_init(MOTOR_A_PHASE_A_PIN, &in_config, gpiote_event_handlerAA);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(MOTOR_A_PHASE_A_PIN, true);
	
	err_code = nrf_drv_gpiote_in_init(MOTOR_A_PHASE_B_PIN, &in_config, gpiote_event_handlerAB);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(MOTOR_A_PHASE_B_PIN, true);
	
    err_code = nrf_drv_gpiote_in_init(MOTOR_B_PHASE_A_PIN, &in_config, gpiote_event_handlerBA);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(MOTOR_B_PHASE_A_PIN, true);
	
	err_code = nrf_drv_gpiote_in_init(MOTOR_B_PHASE_B_PIN, &in_config, gpiote_event_handlerBB);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(MOTOR_B_PHASE_B_PIN, true);
}

//setup ppi between gpiote IN event from one of the encoder phases and timer counter task
static void ppi_setup(void)
{
	uint32_t err_code;
	
	nrf_ppi_channel_t ppi_channel1, ppi_channel2;
	
	err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_ppi_channel_alloc(&ppi_channel1);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_assign(ppi_channel1,
                                          nrf_drv_gpiote_in_event_addr_get(MOTOR_A_PHASE_A_PIN),
                                          nrf_drv_timer_task_address_get(&encoder_a_timer, NRF_TIMER_TASK_COUNT));
    APP_ERROR_CHECK(err_code);

	err_code = nrf_drv_ppi_channel_alloc(&ppi_channel2);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_ppi_channel_assign(ppi_channel2,
                                          nrf_drv_gpiote_in_event_addr_get(MOTOR_B_PHASE_A_PIN),
                                          nrf_drv_timer_task_address_get(&encoder_b_timer, NRF_TIMER_TASK_COUNT));
    APP_ERROR_CHECK(err_code);
	
    // Enable both configured PPI channels
    err_code = nrf_drv_ppi_channel_enable(ppi_channel1);
    APP_ERROR_CHECK(err_code);
	
	err_code = nrf_drv_ppi_channel_enable(ppi_channel2);
    APP_ERROR_CHECK(err_code);
}

void encoder_init(void)
{
	memset(encoder_values, 0, sizeof(encoder_values));
	
	timer_setup();
	gpiote_setup();
	ppi_setup();
}

void get_encoders(int32_t values[2])
{
	values[0] = encoder_values[0];
	values[1] = encoder_values[1];
	/*
	values[0] = nrf_drv_timer_capture(&encoder_a_timer, NRF_TIMER_CC_CHANNEL0);
	nrf_drv_timer_clear(&encoder_a_timer);
	
	values[1] = nrf_drv_timer_capture(&encoder_b_timer, NRF_TIMER_CC_CHANNEL0);
	nrf_drv_timer_clear(&encoder_b_timer);
	*/
}
