/*
 * servo.h
 *
 *  Created on: 17.06.2026
 *      Author: marcp
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern uint8_t motor_buffer_ready[7];
extern uint8_t motor_buffer[7][30];

#define SERVO_MAX_WAIT_TIME 15
#define NUM_SERVOS 6

void terminal_print_servo_data(void);
void servo_write_byte(uint8_t id, uint8_t start_addr, uint8_t num_bytes, ...);
void servo_read_all(void);
void servo_read_id(uint8_t id);
void servo_set_angle(uint8_t id, float angle);
void servo_min_max_calibration(uint8_t motor_buffer[][30], uint8_t *motor_buffer_ready);
void servo_max_torque_initialization(void);
#endif /* INC_SERVO_H_ */
