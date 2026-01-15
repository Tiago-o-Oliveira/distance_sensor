#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "acc_definitions_a121.h"
#include "acc_definitions_common.h"
#include "acc_detector_distance.h"
#include "acc_hal_definitions_a121.h"
#include "acc_hal_integration_a121.h"
#include "acc_integration.h"
#include "acc_integration_log.h"
#include "acc_rss_a121.h"
#include "acc_sensor.h"
#include "acc_version.h"

#include "ring_buffer.h"
/** \example example_detector_distance_low_power_off.c
 * @brief This is an example on how to disable the sensor and put the system in a low power state between measurements
 * @n
 * The example executes as follows:
 *   - Create a distance configuration
 *   - Create a sensor instance
 *   - Enable sensor
 *   - Create a detector instance
 *   - Calibrate the sensor
 *   - Calibrate the detector
 *    - Disable sensor
 *   - Loop forever:
 *     - Enable sensor
 *     - Prepare the detector
 *     - Perform a sensor measurement and read out the data
 *     - Disable sensor
 *     - Process the measurement and get detector result
 *     - Put the system in deep sleep for a specified amount of time
 *   - Destroy the configuration
 *   - Destroy the detector instance
 *   - Destroy the sensor instance
 */

/** Typedef **/
typedef enum{
	DISTANCE_PRESET_CONFIG_NONE = 0,
	DISTANCE_PRESET_CONFIG_BALANCED,
	DISTANCE_PRESET_CONFIG_HIGH_ACCURACY,
} distance_preset_config_t;

typedef struct{
	acc_detector_distance_config_t   *distance_config;
	acc_detector_distance_handle_t   *distance_handle;
	acc_sensor_t                     *sensor;
	void                             *buffer;
	uint8_t                          *detector_cal_result_static;
	uint32_t                          buffer_size;
	uint32_t                          detector_cal_result_static_size;
	acc_detector_cal_result_dynamic_t detector_cal_result_dynamic;
	acc_cal_result_t                  sensor_cal_result;
} a121_sensor_t;

typedef enum{
	STATE_MEASURE = 0x00,
	STATE_TEST_AT,
	STATE_SETUP_CONNECTION,
	STATE_CONNECT,
	STATE_MQTT_CONNECT,
	STATE_SEND_DATA,
	STATE_RECEIVE_CONFIG,
	STATE_TURN_OFF,
	STATE_TEST_RADAR,
	STATE_TEST_MODEM,
} sensor_states_t;

/** Definitions **/
#define SENSOR_ID           (1U)
#define SENSOR_TIMEOUT_MS   (1000U)
#define DEFAULT_UPDATE_RATE (1.0f)
#define FIRMWARE_VERSION 0x01

/** Globals **/
extern UART_HandleTypeDef huart2;
static uint8_t state = STATE_MEASURE;
/** Prototypes **/
static void cleanup(acc_detector_distance_handle_t *distance_handle,
                    acc_detector_distance_config_t *distance_config,
                    acc_sensor_t                   *sensor,
                    void                           *buffer,
                    uint8_t                        *detector_cal_result_static);

static void set_config(acc_detector_distance_config_t *detector_config, distance_preset_config_t preset);
int acconeer_main(int argc, char *argv[]);

uint8_t nrf9151_setup(void);
float calculate_result(acc_detector_distance_result_t *result);
void u64_to_str(uint64_t val, char *buf);

static uint8_t a121_config_and_calibrate(a121_sensor_t *a121Sensor);
static uint8_t a121_measure(a121_sensor_t *a121Sensor,acc_detector_distance_result_t *result);
/** Functions **/



static uint8_t a121_config_and_calibrate(a121_sensor_t *a121Sensor){
	a121Sensor->distance_config                 = NULL;
	a121Sensor->distance_handle                 = NULL;
	a121Sensor->sensor                          = NULL;
	a121Sensor->buffer                          = NULL;
	a121Sensor->detector_cal_result_static      = NULL;
	a121Sensor->buffer_size                     = 0U;
	a121Sensor->detector_cal_result_static_size = 0U;
	a121Sensor->detector_cal_result_dynamic     = (acc_detector_cal_result_dynamic_t){0};

	const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation();

	if (!acc_rss_hal_register(hal)){
		return EXIT_FAILURE;
	}

	a121Sensor->distance_config = acc_detector_distance_config_create();

	if (a121Sensor->distance_config == NULL){
		printf("acc_detector_distance_config_create() failed\n");
		cleanup(a121Sensor->distance_handle, a121Sensor->distance_config, a121Sensor->sensor, a121Sensor->buffer, a121Sensor->detector_cal_result_static);
		return EXIT_FAILURE;
	}

	set_config(a121Sensor->distance_config, DISTANCE_PRESET_CONFIG_BALANCED);

	uint32_t sleep_time_ms = (uint32_t)(300000.0f / DEFAULT_UPDATE_RATE);

	acc_integration_set_periodic_wakeup(sleep_time_ms);

	a121Sensor->distance_handle = acc_detector_distance_create(a121Sensor->distance_config);

	if (a121Sensor->distance_handle == NULL){
		printf("acc_detector_distance_create() failed\n");
		cleanup(a121Sensor->distance_handle, a121Sensor->distance_config, a121Sensor->sensor, a121Sensor->buffer, a121Sensor->detector_cal_result_static);
		return EXIT_FAILURE;
	}

	if (!acc_detector_distance_get_sizes(a121Sensor->distance_handle, &a121Sensor->buffer_size, &a121Sensor->detector_cal_result_static_size)){
		printf("acc_detector_distance_get_buffer_size() failed\n");
		cleanup(a121Sensor->distance_handle, a121Sensor->distance_config, a121Sensor->sensor, a121Sensor->buffer, a121Sensor->detector_cal_result_static);
		return EXIT_FAILURE;
	}

	a121Sensor->buffer = acc_integration_mem_alloc(a121Sensor->buffer_size);
	if (a121Sensor->buffer == NULL){
		printf("buffer allocation failed\n");
		cleanup(a121Sensor->distance_handle, a121Sensor->distance_config, a121Sensor->sensor, a121Sensor->buffer, a121Sensor->detector_cal_result_static);
		return EXIT_FAILURE;
	}

	a121Sensor->detector_cal_result_static = acc_integration_mem_alloc(a121Sensor->detector_cal_result_static_size);
	if (a121Sensor->detector_cal_result_static == NULL){
		printf("buffer allocation failed\n");
		cleanup(a121Sensor->distance_handle, a121Sensor->distance_config, a121Sensor->sensor, a121Sensor->buffer, a121Sensor->detector_cal_result_static);
		return EXIT_FAILURE;
	}

	acc_hal_integration_sensor_supply_on(SENSOR_ID);
	acc_hal_integration_sensor_enable(SENSOR_ID);

	a121Sensor->sensor = acc_sensor_create(SENSOR_ID);
	if (a121Sensor->sensor == NULL){
		printf("acc_sensor_create() failed\n");
		cleanup(a121Sensor->distance_handle, a121Sensor->distance_config, a121Sensor->sensor, a121Sensor->buffer, a121Sensor->detector_cal_result_static);
		return EXIT_FAILURE;
	}

	// Calibrate sensor
	bool             status;
	bool             cal_complete = false;
	acc_cal_result_t sensor_cal_result;

	do{
		status = acc_sensor_calibrate(a121Sensor->sensor, &cal_complete, &sensor_cal_result, a121Sensor->buffer, a121Sensor->buffer_size);

		if (cal_complete){
			break;
		}

		if (status){
			status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
		}
	} while (status);

	if (!status){
		printf("acc_sensor_calibrate() failed\n");
		cleanup(a121Sensor->distance_handle, a121Sensor->distance_config, a121Sensor->sensor, a121Sensor->buffer, a121Sensor->detector_cal_result_static);
		return EXIT_FAILURE;
	}

	/* Reset sensor after calibration by disabling/enabling it */
	acc_hal_integration_sensor_disable(SENSOR_ID);
	acc_hal_integration_sensor_enable(SENSOR_ID);

	// Calibrate detector
	bool done = false;

	do{
		status = acc_detector_distance_calibrate(a121Sensor->sensor,
		                                         a121Sensor->distance_handle,
		                                         &sensor_cal_result,
		                                         a121Sensor->buffer,
		                                         a121Sensor->buffer_size,
		                                         a121Sensor->detector_cal_result_static,
		                                         a121Sensor->detector_cal_result_static_size,
		                                         &a121Sensor->detector_cal_result_dynamic,
		                                         &done);

		if (done){
			break;
		}

		if (status){
			status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
		}
	} while (status);

	if (!status){
		printf("acc_detector_distance_calibrate() failed\n");
		cleanup(a121Sensor->distance_handle, a121Sensor->distance_config, a121Sensor->sensor, a121Sensor->buffer, a121Sensor->detector_cal_result_static);
		return EXIT_FAILURE;
	}

	acc_hal_integration_sensor_disable(SENSOR_ID);

	/* Preserve variable name and value exactly as used later */
	a121Sensor->sensor_cal_result = sensor_cal_result;

	return EXIT_SUCCESS;
}

static uint8_t a121_measure(a121_sensor_t *a121Sensor, acc_detector_distance_result_t *result){

	acc_hal_integration_sensor_enable(SENSOR_ID);

	bool result_available = false;

	do {
		if (!acc_detector_distance_prepare(a121Sensor->distance_handle,
				a121Sensor->distance_config, a121Sensor->sensor,
				&a121Sensor->sensor_cal_result, a121Sensor->buffer,
				a121Sensor->buffer_size)) {
			printf("acc_detector_distance_prepare() failed\n");
			cleanup(a121Sensor->distance_handle, a121Sensor->distance_config,
					a121Sensor->sensor, a121Sensor->buffer,
					a121Sensor->detector_cal_result_static);
			return EXIT_FAILURE;
		}

		if (!acc_sensor_measure(a121Sensor->sensor)) {
			printf("acc_sensor_measure failed\n");
			cleanup(a121Sensor->distance_handle, a121Sensor->distance_config,
					a121Sensor->sensor, a121Sensor->buffer,
					a121Sensor->detector_cal_result_static);
			return EXIT_FAILURE;
		}

		if (!acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID,
				SENSOR_TIMEOUT_MS)) {
			printf("Sensor interrupt timeout\n");
			cleanup(a121Sensor->distance_handle, a121Sensor->distance_config,
					a121Sensor->sensor, a121Sensor->buffer,
					a121Sensor->detector_cal_result_static);
			return EXIT_FAILURE;
		}

		if (!acc_sensor_read(a121Sensor->sensor, a121Sensor->buffer,
				a121Sensor->buffer_size)) {
			printf("acc_sensor_read failed\n");
			cleanup(a121Sensor->distance_handle, a121Sensor->distance_config,
					a121Sensor->sensor, a121Sensor->buffer,
					a121Sensor->detector_cal_result_static);
			return EXIT_FAILURE;
		}

		if (!acc_detector_distance_process(a121Sensor->distance_handle,
				a121Sensor->buffer, a121Sensor->detector_cal_result_static,
				&a121Sensor->detector_cal_result_dynamic, &result_available,
				result)) {
			printf("acc_detector_distance_process failed\n");
			cleanup(a121Sensor->distance_handle, a121Sensor->distance_config,
					a121Sensor->sensor, a121Sensor->buffer,
					a121Sensor->detector_cal_result_static);
			return EXIT_FAILURE;
		}
	} while (!result_available);
	acc_hal_integration_sensor_disable(SENSOR_ID);

	return EXIT_SUCCESS;
}


int acconeer_main(int argc, char *argv[]){
	(void)argc;
	(void)argv;

	a121_sensor_t a121Sensor;
	acc_detector_distance_result_t result;


	if (a121_config_and_calibrate(&a121Sensor) != EXIT_SUCCESS){
		return EXIT_FAILURE;
	}

	while (true){

		switch (state){
			case STATE_MEASURE:

				break;
			case STATE_TEST_AT:

				break;

		}









		char str[30];
		char command[256];

		nrf9151_setup();


		ring_buffer_flush_buffer();
		uint8_t Command[] = "AT%XVBAT";
				HAL_UART_Transmit_DMA(&huart2, (uint8_t*) Command,
						sizeof(Command) - 1);
				HAL_Delay(1000);
				uart_wait_for_string((const uint8_t *)"%XVBAT:",1000);


		uint8_t Command1[] = "AT+CFUN=1\r\n";
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*) Command1,
				sizeof(Command1) - 1);
		HAL_Delay(60000);
		//while (!(uart_wait_for_line("CEREG: 1",60000)));

		uint8_t Command2[] = "AT#XMQTTCON=1,\"ecomfort-gateway\",\"ecomfort*2018\",\"devmqtt.ecomfort.com.br\",1883\r\n";
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*) Command2,
				sizeof(Command2) - 1);
		HAL_Delay(60000);

		uint16_t distance_result = (uint16_t) calculate_result(&result);

		uint64_t data = (0x06A0000300010000 | distance_result);
		data |= ((uint64_t) (result.temperature & 0xFF)) << 40;

		u64_to_str(data, str);

		//HAL_UART_Transmit(&huart1,(uint8_t *)str,10,HAL_MAX_DELAY);

		int len = snprintf(command, sizeof(command),"AT#XMQTTPUB=\"ecomfort/iot/v1/s2g/gateway/LTE25082800003/device/0x000d6f000ca67637/event\",\"%s\"\r\n",str);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*) command, len);
		HAL_Delay(60000);

		acc_integration_sleep_until_periodic_wakeup();


	}


	cleanup(a121Sensor.distance_handle, a121Sensor.distance_config, a121Sensor.sensor, a121Sensor.buffer, a121Sensor.detector_cal_result_static);

	printf("Application finished OK\n");

	return EXIT_SUCCESS;
}





static void cleanup(acc_detector_distance_handle_t *distance_handle,
                    acc_detector_distance_config_t *distance_config,
                    acc_sensor_t                   *sensor,
                    void                           *buffer,
                    uint8_t                        *detector_cal_result_static){
	acc_hal_integration_sensor_disable(SENSOR_ID);
	acc_hal_integration_sensor_supply_off(SENSOR_ID);

	if (distance_config != NULL){
		acc_detector_distance_config_destroy(distance_config);
	}

	if (distance_handle != NULL){
		acc_detector_distance_destroy(distance_handle);
	}

	if (sensor != NULL){
		acc_sensor_destroy(sensor);
	}

	if (buffer != NULL){
		acc_integration_mem_free(buffer);
	}

	if (detector_cal_result_static != NULL){
		acc_integration_mem_free(detector_cal_result_static);
	}
}

static void set_config(acc_detector_distance_config_t *detector_config, distance_preset_config_t preset){
	// Add configuration of the detector here
	switch (preset){
		case DISTANCE_PRESET_CONFIG_NONE:
			// Add configuration of the detector here
			break;

		case DISTANCE_PRESET_CONFIG_BALANCED:
			acc_detector_distance_config_start_set(detector_config, 0.25f);
			acc_detector_distance_config_end_set(detector_config, 3.0f);
			acc_detector_distance_config_max_step_length_set(detector_config, 0U);
			acc_detector_distance_config_max_profile_set(detector_config, ACC_CONFIG_PROFILE_5);
			acc_detector_distance_config_reflector_shape_set(detector_config, ACC_DETECTOR_DISTANCE_REFLECTOR_SHAPE_GENERIC);
			acc_detector_distance_config_peak_sorting_set(detector_config, ACC_DETECTOR_DISTANCE_PEAK_SORTING_STRONGEST);
			acc_detector_distance_config_threshold_method_set(detector_config, ACC_DETECTOR_DISTANCE_THRESHOLD_METHOD_CFAR);
			acc_detector_distance_config_threshold_sensitivity_set(detector_config, 0.5f);
			acc_detector_distance_config_signal_quality_set(detector_config, 15.0f);
			acc_detector_distance_config_close_range_leakage_cancellation_set(detector_config, false);
			break;

		case DISTANCE_PRESET_CONFIG_HIGH_ACCURACY:
			acc_detector_distance_config_start_set(detector_config, 0.25f);
			acc_detector_distance_config_end_set(detector_config, 3.0f);
			acc_detector_distance_config_max_step_length_set(detector_config, 2U);
			acc_detector_distance_config_max_profile_set(detector_config, ACC_CONFIG_PROFILE_3);
			acc_detector_distance_config_reflector_shape_set(detector_config, ACC_DETECTOR_DISTANCE_REFLECTOR_SHAPE_GENERIC);
			acc_detector_distance_config_peak_sorting_set(detector_config, ACC_DETECTOR_DISTANCE_PEAK_SORTING_STRONGEST);
			acc_detector_distance_config_threshold_method_set(detector_config, ACC_DETECTOR_DISTANCE_THRESHOLD_METHOD_CFAR);
			acc_detector_distance_config_threshold_sensitivity_set(detector_config, 0.5f);
			acc_detector_distance_config_signal_quality_set(detector_config, 20.0f);
			acc_detector_distance_config_close_range_leakage_cancellation_set(detector_config, false);
			break;
	}
}


void fsm_state_measure(void){
	static uint8_t retryTimes;

	if(a121_measure(&a121Sensor, &result)){

	}else{

	}
}

uint8_t nrf9151_setup(void){

	ring_buffer_flush_buffer();

	uint8_t Command1 [] ="AT\r\n";
	HAL_UART_Transmit_DMA(&huart2,(uint8_t *)Command1,sizeof(Command1) - 1);

	if(uart_wait_for_string((const uint8_t *)"OK\r\n",1000)){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	}else{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
	}


	uint8_t Command2 [] ="AT+CMEE=1\r\n";
	HAL_UART_Transmit_DMA(&huart2,(uint8_t *)Command2,sizeof(Command2) - 1);
	HAL_Delay(1000);

	uint8_t Command3 [] ="AT+CFUN=0\r\n";
	HAL_UART_Transmit_DMA(&huart2,(uint8_t *)Command3,sizeof(Command3) - 1);
	HAL_Delay(1000);

	uint8_t Command4 [] ="AT+CEREG=5\r\n";
	HAL_UART_Transmit_DMA(&huart2,(uint8_t *)Command4,sizeof(Command4) - 1);
	HAL_Delay(1000);

	uint8_t Command5 [] ="AT+CGDCONT=0,\"IP\",\"kiteiot.vivo.com.br\"\r\n";
	HAL_UART_Transmit_DMA(&huart2,(uint8_t *)Command5,sizeof(Command5) - 1);
	HAL_Delay(1000);

	uint8_t Command6 [] ="AT+CGAUTH=0,1,\"vivo\",\"vivo\"\r\n";
	HAL_UART_Transmit_DMA(&huart2,(uint8_t *)Command6,sizeof(Command6) - 1);
	HAL_Delay(1000);

	return 0x00;

}


float calculate_result(acc_detector_distance_result_t *result){
	float smallerDistance;

	if (result->num_distances > 0){
		smallerDistance = result->distances[0];
	}else{
		smallerDistance = 0;
	}

	return (smallerDistance*1000.0);
}

void u64_to_str(uint64_t val, char *buf) {
    char temporary[21];
    int i = 0;

    do {
    	temporary[i++] = '0' + (val % 10);
        val /= 10;
    } while (val);

    int j = 0;

    while (i--) buf[j++] = temporary[i];
    buf[j] = '\0';
}





