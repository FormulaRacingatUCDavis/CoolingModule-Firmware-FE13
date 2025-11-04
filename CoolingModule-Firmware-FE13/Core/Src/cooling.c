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

uint8_t num_samples;

// average ADC readings
uint64_t adc_inlet_temp_average = 0;
uint64_t adc_outlet_temp_average = 0;
uint64_t adc_air_in_temp_average = 0;
uint64_t adc_air_out_temp_average = 0;

uint16_t curr_inlet_temp = 0;
uint16_t curr_outlet_temp = 0;
uint16_t curr_air_in_temp = 0;
uint16_t curr_air_out_temp = 0;



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

// ISR called when ADC finishes conversion and DMA has written to ADC_RES_BUFFER
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	static uint8_t tx_data[8];
	if (num_samples == 0) { // first sample of this average
		adc_inlet_temp_average = ADC_RES_BUFFER[0];
		adc_outlet_temp_average = ADC_RES_BUFFER[1];
		adc_air_in_temp_average = ADC_RES_BUFFER[2];
		adc_air_out_temp_average = ADC_RES_BUFFER[3];
	} else if (num_samples < NUM_SAMPLES_IN_AVERAGE) {
		// calculate running average
		// NewAverage = OldAverage * (n-1) / n + NewValue / n, where n is num elements AFTER new element included
		num_samples++;
		adc_inlet_temp_average = adc_inlet_temp_average * (num_samples-1) / num_samples + ADC_RES_BUFFER[0] / num_samples;
		adc_outlet_temp_average = adc_outlet_temp_average * (num_samples-1) / num_samples + ADC_RES_BUFFER[1] / num_samples;
		adc_air_in_temp_average = adc_air_in_temp_average * (num_samples-1) / num_samples + ADC_RES_BUFFER[2] / num_samples;
		adc_air_out_temp_average = adc_air_out_temp_average * (num_samples-1) / num_samples + ADC_RES_BUFFER[3] / num_samples;
	} else {
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
		num_samples = 0;
	}
}

void update_pwm(int16_t inlet_temp)
{
	//set_pump_speed(40);

	// allow cooling override: use hardcoded values for pump and fan speed
	if (can_data.PWM_requested) {
		set_pump_speed(255);
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


