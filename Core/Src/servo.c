/*
 * servo.c
 *
 *  Created on: 07.07.2026
 *      Author: marcp
 */

#include"servo.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define REG_TORQUE_ENABLE 0x28
#define TERMINAL_PRINT_CYCLE_COUNT 25

uint8_t terminal_print_count = 0;

void terminal_print_servo_data(void) {
	//terminal print pv
	char msg[300];
	uint16_t Location[7];
	uint16_t Speed[7];
	float Load[7];

	uint8_t Temp[7];
	uint8_t Status[7];
	uint8_t Motion[7];
	float Voltage[7];
	float Current[7];
	terminal_print_count++;
	if (terminal_print_count == TERMINAL_PRINT_CYCLE_COUNT) {
		terminal_print_count = 0;
		for (int id = 1; id < 7; id++) {
			Location[id] = (motor_buffer[id][6] << 8) | motor_buffer[id][5];
			Speed[id] = (motor_buffer[id][8] << 8) | motor_buffer[id][7];
			uint16_t raw_load = (motor_buffer[id][10] << 8)
					| motor_buffer[id][9];
			Load[id] = (float) raw_load / 10;
			Temp[id] = motor_buffer[id][12];
			Status[id] = motor_buffer[id][14];
			Motion[id] = motor_buffer[id][15];
			uint8_t raw_voltage = motor_buffer[id][11];
			Voltage[id] = (float) raw_voltage / 10;
			uint8_t raw_current = motor_buffer[id][18];
			Current[id] = (float) raw_current * 6.5 / 1000;
			int text_len = snprintf(msg, sizeof(msg),
							"ID: %1d Location: %4d stp Speed: %4d stp/s Load: %5.1f%% Temp: %2d°C Status: %1d Motion %1d Voltage: %4.1fV Current: %5.3fA \r\n",
							id, Location[id], Speed[id], Load[id], Temp[id], Status[id], Motion[id], Voltage[id], Current[id]);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, text_len,
			HAL_MAX_DELAY);
		}
	}
}

void servo_write_byte(uint8_t id, uint8_t start_addr, uint8_t num_bytes, ...) {
	uint8_t data_buffer[32];
	if(num_bytes > 32) return;

	va_list args;
	va_start(args, num_bytes);

	for (uint8_t i = 0; i < num_bytes; i++) {
		data_buffer[i] = (uint8_t)va_arg(args, int);
	}

	va_end(args);

	uint8_t servo_cmd[39] = {0XFF, 0XFF, id, num_bytes + 3, 0X03, start_addr};

	uint8_t checksum = servo_cmd[2] + servo_cmd[3] + servo_cmd[4] + servo_cmd[5];

	for(uint8_t i = 6; i < num_bytes + 6; i++){
		servo_cmd[i] = data_buffer[i-6];
		checksum += data_buffer[i - 6];
	}

	servo_cmd[num_bytes + 6] = ~checksum;

	HAL_UART_Transmit(&huart1, servo_cmd, num_bytes + 7, HAL_MAX_DELAY);
}

void servo_read_all(void){
	//set motor buffer ready flag to zero
	memset((void*)motor_buffer_ready, 0, sizeof(motor_buffer_ready));

	//send command to read the data
	uint8_t servo_cmd[] = {0XFF, 0XFF, 0XFE, 0X0A, 0X82, 0X38, 0X0E, 0X01, 0X02, 0X03, 0X04, 0X05, 0X06, 0X00};

	uint8_t checksum = 0;
	for(int i = 2; i < (sizeof(servo_cmd) - 1); i++){
		checksum += servo_cmd[i];
	}
	servo_cmd[sizeof(servo_cmd) - 1] = ~checksum;

	HAL_UART_Transmit(&huart1, servo_cmd, sizeof(servo_cmd), HAL_MAX_DELAY);

	//wait till all motors have answered
	uint32_t start_time = HAL_GetTick();
	while(1){
		uint8_t all_ready = 1;

		for(int id = 1; id < 7; id++){
			if(motor_buffer_ready[id] == 0){
				all_ready = 0;
				break;
			}
		}

		if(all_ready == 1){
			break;
		}

		if((HAL_GetTick() - start_time) > SERVO_MAX_WAIT_TIME){
			HAL_UART_Transmit(&huart2, (uint8_t*)"Error: mindestens einer der Servos antwortet nicht\r\n", 52, HAL_MAX_DELAY);
			Error_Handler();
		}
	}
}

void servo_read_id(uint8_t id){
	//set motor buffer ready flag to zero
	memset((void*)motor_buffer_ready, 0, sizeof(motor_buffer_ready));

	//send command to read the data
	uint8_t servo_cmd[] = {0XFF, 0XFF, id, 0X04, 0X02, 0X38, 0X0E, 0X00};

	uint8_t checksum = 0;
	for(int i = 2; i < (sizeof(servo_cmd) - 1); i++){
		checksum += servo_cmd[i];
	}
	servo_cmd[sizeof(servo_cmd) - 1] = ~checksum;

	HAL_UART_Transmit(&huart1, servo_cmd, sizeof(servo_cmd), HAL_MAX_DELAY);

	//wait till all motors have answered
		uint32_t start_time = HAL_GetTick();
		while(motor_buffer_ready[id] == 0){
			if((HAL_GetTick() - start_time) > SERVO_MAX_WAIT_TIME){
				HAL_UART_Transmit(&huart2, (uint8_t*)"Error: der einzeln angefragte Servos antwortet nicht\r\n", 54, HAL_MAX_DELAY);
				Error_Handler();
			}
		}
}

void servo_set_angle(uint8_t id, float angle){
	uint16_t steps = (((angle + 180) * 4095) / 360);
	uint16_t speed = (32766 / ((7 - id) * (7 - id)));

	servo_write_byte(id, 0X2A, 6, steps & 0xFF, (steps >> 8) & 0xFF, 0X00, 0X00, speed & 0xFF, (speed >> 8) & 0xFF);
}

void servo_zero_point_calibration(void){
	char msg[300];
	HAL_UART_Transmit(&huart2, (uint8_t*)"start zero point Calibration completed\r\n", 40, HAL_MAX_DELAY);
	HAL_Delay(50);

	//unlock servos
	servo_write_byte(0xFE, 0x37, 1, 0);
	servo_write_byte(0xFE, 0x28, 1, 0);

	//calibrate servos
	for(uint8_t id = 1; id < 7; id++){
		int len = sprintf(msg,"Press the blue button on the stm32 to calibrate servo Nr.%d\r\n", id);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, len, HAL_MAX_DELAY);
		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET){}
		HAL_Delay(20);
		servo_write_byte(id, 0x28, 1, 0x80);
		HAL_UART_Transmit(&huart2, (uint8_t*)"Calibrated\r\n", 12, HAL_MAX_DELAY);
		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){}
		HAL_Delay(20);
	}

	//lock servos
	servo_write_byte(0xFE, 0x37, 1, 1);
	servo_write_byte(0xFE, 0x28, 1, 1);

	HAL_UART_Transmit(&huart2, (uint8_t*)"zero point Calibration completed\r\n", 34, HAL_MAX_DELAY);
}

void servo_min_max_calibration(uint8_t motor_buffer[][30], uint8_t *motor_buffer_ready){
	char msg[300];
	uint16_t min_angle = 4095;
	uint16_t max_angle = 0;
	uint16_t Location;

	//unlock servos
	servo_write_byte(0xFE, 0x37, 1, 0);
	servo_write_byte(0xFE, 0x28, 1, 0);

	//calibrate servos
	for(uint8_t id = 1; id < 7; id++){
		HAL_UART_Transmit(&huart2, (uint8_t*)"start min max Calibration completed\r\n", 37, HAL_MAX_DELAY);
		HAL_Delay(50);
		int len = sprintf(msg,"Move servo Nr.%d to its absolute maximum and minimum angle then press the blue button on the stm32\r\n", id);
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, len, HAL_MAX_DELAY);
		HAL_Delay(50);

		//Werte sammeln solange der Knopf nicht gedrückt ist
		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET){
			servo_read_id(id);
			Location = (motor_buffer[id][6] << 8) | motor_buffer[id][5];
			if (Location < min_angle) {
				min_angle = Location;
			}
			if (Location > max_angle) {
				max_angle = Location;
			}
			len = sprintf(msg,
							"minimum angle: %4d maximum angle: %4d current angle:%4d\r\n",
							min_angle, max_angle, Location);
			HAL_UART_Transmit(&huart2, (uint8_t*) msg, len, HAL_MAX_DELAY);
			HAL_Delay(250);
		}

		//Werte speichern sobald der knopf gedrückt wird
		uint16_t final_min_angle = min_angle + 25;
		uint16_t final_max_angle = max_angle - 25;
		min_angle = 4095;
		max_angle = 0;
		servo_write_byte(id, 0x09, 2, (uint8_t)(final_min_angle & 0xFF), (uint8_t)((final_min_angle >> 8) & 0xFF));
		servo_write_byte(id, 0x0B, 2, (uint8_t)(final_max_angle & 0xFF), (uint8_t)((final_max_angle >> 8) & 0xFF));
		HAL_UART_Transmit(&huart2, (uint8_t*)"Calibrated\r\n", 12, HAL_MAX_DELAY);
		HAL_Delay(20);
		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET){}
		HAL_Delay(20);
	}
	//lock servos
	servo_write_byte(0xFE, 0x37, 1, 1);
	servo_write_byte(0xFE, 0x28, 1, 1);

	HAL_UART_Transmit(&huart2, (uint8_t*)"min max Calibration completed\r\n", 31, HAL_MAX_DELAY);
}

void servo_max_torque_initialization(void){
	for(uint8_t id = 1; id < 7; id++){
		servo_write_byte(id, 0x37, 1, 0);
		servo_write_byte(id, 0x10, 2, 0x12, 0x01);
	}
	HAL_UART_Transmit(&huart2, (uint8_t*)"torque Calibration completed\r\n", 30, HAL_MAX_DELAY);
	HAL_Delay(1000);
}
