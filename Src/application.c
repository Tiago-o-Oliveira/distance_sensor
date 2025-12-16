// Copyright (c) Acconeer AB, 2023-2024
// All rights reserved
// This file is subject to the terms and conditions defined in the file
// 'LICENSES/license_acconeer.txt', (BSD 3-Clause License) which is part
// of this source code package.

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

typedef enum{
	DISTANCE_PRESET_CONFIG_NONE = 0,
	DISTANCE_PRESET_CONFIG_BALANCED,
	DISTANCE_PRESET_CONFIG_HIGH_ACCURACY,
} distance_preset_config_t;

#define SENSOR_ID           (1U)
#define SENSOR_TIMEOUT_MS   (1000U)
#define DEFAULT_UPDATE_RATE (1.0f)
#define FIRMWARE_VERSION 0x01

extern UART_HandleTypeDef huart1;

static void cleanup(acc_detector_distance_handle_t *distance_handle,
                    acc_detector_distance_config_t *distance_config,
                    acc_sensor_t                   *sensor,
                    void                           *buffer,
                    uint8_t                        *detector_cal_result_static);

static void set_config(acc_detector_distance_config_t *detector_config, distance_preset_config_t preset);

int acconeer_main(int argc, char *argv[]);
void led_pattern_success(void);
void led_pattern_fail(void);
uint8_t nrf9151_setup(void);
float calculate_result(acc_detector_distance_result_t *result);
void u64_to_str(uint64_t val, char *buf);

int acconeer_main(int argc, char *argv[]){
	(void)argc;
	(void)argv;
	acc_detector_distance_config_t   *distance_config                 = NULL;
	acc_detector_distance_handle_t   *distance_handle                 = NULL;
	acc_sensor_t                     *sensor                          = NULL;
	void                             *buffer                          = NULL;
	uint8_t                          *detector_cal_result_static      = NULL;
	uint32_t                          buffer_size                     = 0U;
	uint32_t                          detector_cal_result_static_size = 0U;
	acc_detector_cal_result_dynamic_t detector_cal_result_dynamic     = {0};


	const acc_hal_a121_t *hal = acc_hal_rss_integration_get_implementation();

	if (!acc_rss_hal_register(hal)){
		return EXIT_FAILURE;
	}

	distance_config = acc_detector_distance_config_create();

	if (distance_config == NULL){
		printf("acc_detector_distance_config_create() failed\n");
		cleanup(distance_handle, distance_config, sensor, buffer, detector_cal_result_static);
		return EXIT_FAILURE;
	}

	set_config(distance_config, DISTANCE_PRESET_CONFIG_BALANCED);

	uint32_t sleep_time_ms = (uint32_t)(300000.0f / DEFAULT_UPDATE_RATE);

	acc_integration_set_periodic_wakeup(sleep_time_ms);

	distance_handle = acc_detector_distance_create(distance_config);
	if (distance_handle == NULL){
		printf("acc_detector_distance_create() failed\n");
		cleanup(distance_handle, distance_config, sensor, buffer, detector_cal_result_static);
		return EXIT_FAILURE;
	}

	if (!acc_detector_distance_get_sizes(distance_handle, &buffer_size, &detector_cal_result_static_size)){
		printf("acc_detector_distance_get_buffer_size() failed\n");
		cleanup(distance_handle, distance_config, sensor, buffer, detector_cal_result_static);
		return EXIT_FAILURE;
	}

	buffer = acc_integration_mem_alloc(buffer_size);
	if (buffer == NULL){
		printf("buffer allocation failed\n");
		cleanup(distance_handle, distance_config, sensor, buffer, detector_cal_result_static);
		return EXIT_FAILURE;
	}

	detector_cal_result_static = acc_integration_mem_alloc(detector_cal_result_static_size);
	if (detector_cal_result_static == NULL){
		printf("buffer allocation failed\n");
		cleanup(distance_handle, distance_config, sensor, buffer, detector_cal_result_static);
		return EXIT_FAILURE;
	}

	acc_hal_integration_sensor_supply_on(SENSOR_ID);
	acc_hal_integration_sensor_enable(SENSOR_ID);

	sensor = acc_sensor_create(SENSOR_ID);
	if (sensor == NULL){
		printf("acc_sensor_create() failed\n");
		cleanup(distance_handle, distance_config, sensor, buffer, detector_cal_result_static);
		return EXIT_FAILURE;
	}

	// Calibrate sensor
	bool             status;
	bool             cal_complete = false;
	acc_cal_result_t sensor_cal_result;

	do{
		status = acc_sensor_calibrate(sensor, &cal_complete, &sensor_cal_result, buffer, buffer_size);

		if (cal_complete){
			break;
		}

		if (status){
			status = acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS);
		}
	} while (status);

	if (!status){
		printf("acc_sensor_calibrate() failed\n");
		cleanup(distance_handle, distance_config, sensor, buffer, detector_cal_result_static);
		return EXIT_FAILURE;
	}

	/* Reset sensor after calibration by disabling/enabling it */
	acc_hal_integration_sensor_disable(SENSOR_ID);
	acc_hal_integration_sensor_enable(SENSOR_ID);

	// Calibrate detector
	bool done = false;

	do{
		status = acc_detector_distance_calibrate(sensor,
		                                         distance_handle,
		                                         &sensor_cal_result,
		                                         buffer,
		                                         buffer_size,
		                                         detector_cal_result_static,
		                                         detector_cal_result_static_size,
		                                         &detector_cal_result_dynamic,
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
		cleanup(distance_handle, distance_config, sensor, buffer, detector_cal_result_static);
		return EXIT_FAILURE;
	}

	acc_hal_integration_sensor_disable(SENSOR_ID);

	while (true){
		acc_detector_distance_result_t result;

		acc_hal_integration_sensor_enable(SENSOR_ID);

		bool result_available = false;

		do{
			if (!acc_detector_distance_prepare(distance_handle, distance_config, sensor, &sensor_cal_result, buffer, buffer_size)){
				printf("acc_detector_distance_prepare() failed\n");
				cleanup(distance_handle, distance_config, sensor, buffer, detector_cal_result_static);
				return EXIT_FAILURE;
			}

			if (!acc_sensor_measure(sensor)){
				printf("acc_sensor_measure failed\n");
				cleanup(distance_handle, distance_config, sensor, buffer, detector_cal_result_static);
				return EXIT_FAILURE;
			}

			if (!acc_hal_integration_wait_for_sensor_interrupt(SENSOR_ID, SENSOR_TIMEOUT_MS)){
				printf("Sensor interrupt timeout\n");
				cleanup(distance_handle, distance_config, sensor, buffer, detector_cal_result_static);
				return EXIT_FAILURE;
			}

			if (!acc_sensor_read(sensor, buffer, buffer_size)){
				printf("acc_sensor_read failed\n");
				cleanup(distance_handle, distance_config, sensor, buffer, detector_cal_result_static);
				return EXIT_FAILURE;
			}

			if (!acc_detector_distance_process(distance_handle,
			                                   buffer,
			                                   detector_cal_result_static,
			                                   &detector_cal_result_dynamic,
			                                   &result_available,
			                                   &result)){
				printf("acc_detector_distance_process failed\n");
				cleanup(distance_handle, distance_config, sensor, buffer, detector_cal_result_static);
				return EXIT_FAILURE;
			}
		} while (!result_available);

		char str[30];
		char command[256];

		acc_hal_integration_sensor_disable(SENSOR_ID);
		nrf9151_setup();
		uart_send_string("AT+CFUN=1\r\n");

		HAL_Delay(60000);
		//while (!(uart_wait_for_line("CEREG: 1",60000)));



		uart_send_string("AT#XMQTTCON=1,\"ecomfort-gateway\",\"ecomfort*2018\",\"devmqtt.ecomfort.com.br\",1883\r\n");

		HAL_Delay(60000);

		uint16_t distance_result = (uint16_t )calculate_result(&result);

		uint64_t data = (0x06A000F000010000 | distance_result);
		data |= ((uint64_t)(result.temperature & 0xFF)) << 40;


		u64_to_str(data, str);


		HAL_UART_Transmit(&huart1,(uint8_t *)str,10,HAL_MAX_DELAY);

		snprintf(command,sizeof(command),"AT#XMQTTPUB=\"ecomfort/iot/v1/s2g/gateway/LTE25082800003/device/0x000d6f000ca67637/event\",\"%s\"\r\n",str);

		//uart_send_string("AT#XMQTTPUB=\"ecomfort/iot/v1/s2g/gateway/LTE25082800003/device/0x000d6f000ca67637/event\",\"06A0CFF000011000\"\r\n");
		uart_send_string(command);



		HAL_Delay(60000);


		acc_integration_sleep_until_periodic_wakeup();
	}

	cleanup(distance_handle, distance_config, sensor, buffer, detector_cal_result_static);

	printf("Application finished OK\n");

	return EXIT_SUCCESS;
}

static void cleanup(acc_detector_distance_handle_t *distance_handle,
                    acc_detector_distance_config_t *distance_config,
                    acc_sensor_t                   *sensor,
                    void                           *buffer,
                    uint8_t                        *detector_cal_result_static)
{
	acc_hal_integration_sensor_disable(SENSOR_ID);
	acc_hal_integration_sensor_supply_off(SENSOR_ID);

	if (distance_config != NULL)
	{
		acc_detector_distance_config_destroy(distance_config);
	}

	if (distance_handle != NULL)
	{
		acc_detector_distance_destroy(distance_handle);
	}

	if (sensor != NULL)
	{
		acc_sensor_destroy(sensor);
	}

	if (buffer != NULL)
	{
		acc_integration_mem_free(buffer);
	}

	if (detector_cal_result_static != NULL)
	{
		acc_integration_mem_free(detector_cal_result_static);
	}
}

static void set_config(acc_detector_distance_config_t *detector_config, distance_preset_config_t preset)
{
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


void led_pattern_success(void){

    HAL_Delay(2000);
}

void led_pattern_fail(void){

    HAL_Delay(500);
}

uint8_t nrf9151_setup(void){

  uart_send_string("AT\r\n");
  HAL_Delay(1000);

  uart_send_string("AT+CMEE=1\r\n");
  HAL_Delay(1000);

  uart_send_string("AT+CFUN=0\r\n");
  HAL_Delay(1000);

  uart_send_string("AT+CEREG=5\r\n");
  HAL_Delay(1000);

  uart_send_string("AT+CGDCONT=0,\"IP\",\"kiteiot.vivo.com.br\"\r\n");
  HAL_Delay(1000);

  uart_send_string("AT+CGAUTH=0,1,\"vivo\",\"vivo\"\r\n");
  HAL_Delay(1000);
  return 0x00;

}

bool nrf9151_connect_to_network(void){
	uint8_t attempts = 0;

	do{
		uart_send_string("AT+CFUN=1\r\n");

		uint32_t startTime = HAL_GetTick();

		while(((HAL_GetTick() - startTime) < 60000)){
			if(uart_wait_for_line("CEREG: 1",1000) == 0x00){
				return true;
			}
		}

		attempts++;
		acc_integration_sleep_ms(1000);
	}while(attempts < 3);

	return false;
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

