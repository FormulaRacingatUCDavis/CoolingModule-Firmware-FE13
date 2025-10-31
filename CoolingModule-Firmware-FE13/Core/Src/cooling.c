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
#define BUZZ_TIME_MS 1500

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

// PRIVATE GLOBALS
ADC_Input_t adc_inlet_temp;
ADC_Input_t adc_outlet_temp;
ADC_Input_t adc_air_in_temp;
ADC_Input_t adc_air_out_temp;
ADC_Input_t adc_extra_1;
ADC_Input_t adc_extra_2;
ADC_Input_t adc_extra_3;
ADC_Input_t adc_extra_4;
// no longer needed:
//ADC_Input_t adc_inlet_pres;
//ADC_Input_t adc_outlet_pres;

PWM_Output_t pwm_fan;
PWM_Output_t pwm_pump;
PWM_Output_t pwm_extra;


// PRIVATE FUNCTION PROTOTYPES
//uint16_t get_pres(uint16_t adc_val);
int16_t get_temp(uint16_t adc_val);
int16_t get_air_temp(uint16_t adc_val);
void set_fan_speed(uint8_t speed);
void set_pump_speed(uint8_t speed);
void update_pwm(int16_t inlet_temp);

void Cooling_Init(){
	// TODO check channels
	ADC_Input_Init(&adc_inlet_temp, &hadc1, ADC_CHANNEL_0, 1, ADC_SAMPLETIME_480CYCLES);
	ADC_Input_Init(&adc_outlet_temp, &hadc1, ADC_CHANNEL_1, 1, ADC_SAMPLETIME_480CYCLES);
	ADC_Input_Init(&adc_air_in_temp, &hadc1, ADC_CHANNEL_2, 1, ADC_SAMPLETIME_480CYCLES);
	ADC_Input_Init(&adc_air_out_temp, &hadc1, ADC_CHANNEL_3, 1, ADC_SAMPLETIME_480CYCLES);
	ADC_Input_Init(&adc_extra_1, &hadc1, ADC_CHANNEL_14, 1, ADC_SAMPLETIME_480CYCLES);
	ADC_Input_Init(&adc_extra_2, &hadc1, ADC_CHANNEL_15, 1, ADC_SAMPLETIME_480CYCLES);
	ADC_Input_Init(&adc_extra_3, &hadc1, ADC_CHANNEL_8, 1, ADC_SAMPLETIME_480CYCLES);
	ADC_Input_Init(&adc_extra_4, &hadc1, ADC_CHANNEL_9, 1, ADC_SAMPLETIME_480CYCLES);
	//  ADC_Input_Init(&adc_inlet_pres, &hadc1, ADC_CHANNEL_4, 1, ADC_SAMPLETIME_480CYCLES);
	//	ADC_Input_Init(&adc_outlet_pres, &hadc1, ADC_CHANNEL_5, 1, ADC_SAMPLETIME_480CYCLES);

	PWM_Init(&pwm_fan, &htim1, TIM_CHANNEL_1);
	PWM_Init(&pwm_pump, &htim1, TIM_CHANNEL_2);
	PWM_Init(&pwm_extra, &htim1, TIM_CHANNEL_3);

	set_pump_speed(255);
	set_fan_speed(128);
}

void Cooling_Update()
{
	static uint8_t loop_count = 0;
	static uint8_t tx_data[8];


	loop_count++;
	if(loop_count <= SLOW_DIVIDER) return;
	loop_count = 0;

	// NOTE: can send temps and pressures at reduced frequency if CAN traffic too high

	ADC_Measure(&adc_inlet_temp, 1000);
	ADC_Measure(&adc_outlet_temp, 1000);
	ADC_Measure(&adc_air_in_temp, 1000);
	ADC_Measure(&adc_air_out_temp, 1000);

	int16_t inlet_temp = get_temp(adc_inlet_temp.value);
	int16_t outlet_temp = get_temp(adc_outlet_temp.value);
	int16_t temp_air_in = get_air_temp(adc_air_in_temp.value);
	int16_t temp_air_out = get_air_temp(adc_air_out_temp.value);

	tx_data[0] = HI8(inlet_temp);
	tx_data[1] = LO8(inlet_temp);
	tx_data[2] = HI8(outlet_temp);
	tx_data[3] = LO8(outlet_temp);
	tx_data[4] = HI8(temp_air_in);
	tx_data[5] = LO8(temp_air_in);
	tx_data[6] = HI8(temp_air_out);
	tx_data[7] = LO8(temp_air_out);
	CAN_Send(&hcan2, COOLING_LOOP_TEMPS, tx_data, 8);

//	ADC_Measure(&adc_inlet_pres, 1000);
//	ADC_Measure(&adc_outlet_pres, 1000);
//
//	uint16_t inlet_pres = get_pres(adc_inlet_pres.value);
//	uint16_t outlet_pres = get_pres(adc_outlet_pres.value);
//
//	tx_data[0] = HI8(inlet_pres);
//	tx_data[1] = LO8(inlet_pres);
//	tx_data[2] = HI8(outlet_pres);
//	tx_data[3] = LO8(outlet_pres);
//	CAN_Send(&hcan2, COOLING_LOOP_PRESSURES, tx_data, 4);



	update_pwm(inlet_temp);
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

//uint16_t get_pres(uint16_t adc_val)
//{
//	// GE2098 sensor
//	// equation from datasheet
//	float v = (float)adc_val * (3.3/4095.0) / VOLTAGE_DIVIDER_RATIO;
//	float pres = (((v / 5.0) + 0.011453) / 0.0045726) * PSI_PER_KPA;
//	return (uint16_t)(pres*100);
//}


