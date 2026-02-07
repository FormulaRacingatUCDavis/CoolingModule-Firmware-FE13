#include "cooling.h"
#include "adc.h"
#include "pwm.h"
#include "can_manager.h"
#include "main.h"
#include "math.h"

#define VOLTAGE_DIVIDER_RATIO (12.0 / (12.0 + 6.04))
#define PSI_PER_KPA 0.145038
#define HI8(x) ((x>>8)&0xFF)
#define LO8(x) (x&0xFF);

#define NUM_SAMPLES_IN_AVERAGE 5

#define PUMP_THRESH 400
#define FAN_THRESH_1 400
#define FAN_THRESH_2 500
#define FAN_THRESH_3 600
#define HYSTERESIS 30

// HANDLE TYPE DEFS from main
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;

extern CAN_DATA_t can_data;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern uint32_t ADC_RES_BUFFER[4]; // buffer for ADC DMA

// PRIVATE GLOBALS

PWM_Output_t pwm_fan;
PWM_Output_t pwm_pump;
PWM_Output_t pwm_extra;

uint8_t num_samples = 0;

// average ADC readings
uint64_t adc_inlet_temp_average = 0;
uint64_t adc_outlet_temp_average = 0;
uint64_t adc_air_in_temp_average = 0;
uint64_t adc_air_out_temp_average = 0;

uint16_t curr_inlet_temp = 0;
uint16_t curr_outlet_temp = 0;
uint16_t curr_air_in_temp = 0;
uint16_t curr_air_out_temp = 0;

uint16_t s1_buf[5];
uint16_t s2_buf[5];
uint16_t s3_buf[5];
uint16_t s4_buf[5];
uint8_t ready_to_send = 0;


// PRIVATE FUNCTION PROTOTYPES
//uint16_t get_pres(uint16_t adc_val);
int16_t get_temp(uint16_t adc_val);
int16_t get_air_temp(uint16_t adc_val);
void set_fan_speed(uint8_t speed);
void set_pump_speed(uint8_t speed);
void update_pwm(int16_t inlet_temp);

void Cooling_Init(){
	PWM_Init(&pwm_fan, &htim1, TIM_CHANNEL_1);
	PWM_Init(&pwm_pump, &htim1, TIM_CHANNEL_2);
	PWM_Init(&pwm_extra, &htim1, TIM_CHANNEL_3);

	adc_inlet_temp_average = 0;
	adc_outlet_temp_average = 0;
	adc_air_in_temp_average = 0;
	adc_air_out_temp_average = 0;

	curr_inlet_temp = 0;
	curr_outlet_temp = 0;
	curr_air_in_temp = 0;
	curr_air_out_temp = 0;

	set_pump_speed(255);
	set_fan_speed(128);
}

void Cooling_Update()
{
	update_pwm(curr_inlet_temp);
}

void Process_Average(){
	static uint8_t tx_data[8];
	if(ready_to_send){
		adc_inlet_temp_average = (s1_buf[0] + s1_buf[1] + s1_buf[2] + s1_buf[3] + s1_buf[4]) / 5;
		adc_outlet_temp_average = (s2_buf[0] + s2_buf[1] + s2_buf[2] + s2_buf[3] + s2_buf[4]) / 5;
		adc_air_in_temp_average = (s3_buf[0] + s3_buf[1] + s3_buf[2] + s3_buf[3] + s3_buf[4]) / 5;
		adc_air_out_temp_average = (s4_buf[0] + s4_buf[1] + s4_buf[2] + s4_buf[3] + s4_buf[4]) / 5;

		// send over can
		int16_t inlet_temp = get_temp(adc_inlet_temp_average);
		int16_t outlet_temp = get_temp(adc_outlet_temp_average);
		int16_t air_in_temp = get_air_temp(adc_air_in_temp_average);
		int16_t air_out_temp = get_air_temp(adc_air_out_temp_average);

		tx_data[0] = HI8(inlet_temp);
		tx_data[1] = LO8(inlet_temp);
		tx_data[2] = HI8(outlet_temp);
		tx_data[3] = LO8(outlet_temp);
		tx_data[4] = HI8(air_in_temp);
		tx_data[5] = LO8(air_in_temp);
		tx_data[6] = HI8(air_out_temp);
		tx_data[7] = LO8(air_out_temp);
		CAN_Send(&hcan2, COOLING_LOOP_TEMPS, tx_data, 8);

		// save most recent value
		curr_inlet_temp = inlet_temp;
		curr_outlet_temp = outlet_temp;
		curr_air_in_temp = air_in_temp;
		curr_air_out_temp = air_out_temp;

		// reset averages and num_samples
		adc_inlet_temp_average = 0;
		adc_outlet_temp_average = 0;
		adc_air_in_temp_average = 0;
		adc_air_out_temp_average = 0;
		ready_to_send = 0;
	}
}

// ISR called when ADC finishes conversion and DMA has written to ADC_RES_BUFFER
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	s1_buf[num_samples] = ADC_RES_BUFFER[0];
	s2_buf[num_samples] = ADC_RES_BUFFER[1];
	s3_buf[num_samples] = ADC_RES_BUFFER[2];
	s4_buf[num_samples] = ADC_RES_BUFFER[3];
	num_samples++;
	if(num_samples == 5){
		ready_to_send = 1;
		num_samples = 0;
	}
}

void update_pwm(int16_t inlet_temp)
{
	// TODO REMOVE THIS, THIS IS FOR COOLING TESTING
	set_pump_speed(229);
	set_fan_speed(229);
	return; // TODO DEFINITELY REMOVE THIS YO

	// allow cooling override: use hardcoded values for pump and fan speed
	if (can_data.PWM_requested) {
		set_pump_speed(128);
		set_fan_speed(128);
		return;
	}

	// threshold variables to add hysteresis
	static int fan_t1 = FAN_THRESH_1 + HYSTERESIS;
	static int fan_t2 = FAN_THRESH_2 + HYSTERESIS;
	static int fan_t3 = FAN_THRESH_2 + HYSTERESIS;
	static int pump_t = PUMP_THRESH + HYSTERESIS;

	//TODO: update these values to consider ambient air temp, vehicle speed, etc?
	if(can_data.inverter_enable || (can_data.mc_temp_max > pump_t) || (can_data.motor_temp > pump_t)){
		set_pump_speed(255);
		pump_t = PUMP_THRESH;
	} else {
		set_pump_speed(0);
		pump_t = PUMP_THRESH + HYSTERESIS;
	}

	if(inlet_temp > fan_t3){
		set_fan_speed(255);
		fan_t1 = FAN_THRESH_1;
		fan_t2 = FAN_THRESH_2;
		fan_t3 = FAN_THRESH_3;
	} else if(inlet_temp > fan_t2){
		set_fan_speed(180);
		fan_t1 = FAN_THRESH_1;
		fan_t2 = FAN_THRESH_2;
		fan_t3 = FAN_THRESH_3 + HYSTERESIS;
	} else if(inlet_temp > fan_t1){
		fan_t1 = FAN_THRESH_1;
		fan_t2 = FAN_THRESH_2 + HYSTERESIS;
		fan_t3 = FAN_THRESH_3 + HYSTERESIS;
		set_fan_speed(100);
	} else {
		fan_t1 = FAN_THRESH_1 + HYSTERESIS;
		fan_t2 = FAN_THRESH_2 + HYSTERESIS;
		fan_t3 = FAN_THRESH_3 + HYSTERESIS;
		set_fan_speed(0);
	}
}

int16_t get_temp(uint16_t adc_val)
{
	// need to recalibrate these sensors with new GE2098(Already calibrated)
	float temp = (99.2596*exp((-3.22926) * adc_val / 4095) - 21.4981-3.17)/1.01085;
	return (int16_t) (temp*10);
}

int16_t get_air_temp(uint16_t adc_val)
{
	float temp = 83.35412 - 0.03634221 * adc_val +0.0000034466 * adc_val * adc_val;
	return (int16_t) (temp *10);
}


void set_pump_speed(uint8_t speed)
{
	PWM_SetDutyCycle(&pwm_pump, speed);
}

void set_fan_speed(uint8_t speed)
{
	PWM_SetDutyCycle(&pwm_fan, speed);
}


