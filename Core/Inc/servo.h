/**
 * @file servo.h
 * @brief library for the serial control of the Waveshare St3215 servos.
 * @note developed for the so101 robot arm project via STM32 HAL.
 * @author marcp
 * @date 17.06.2026
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern uint8_t motor_buffer_ready[7];
extern uint8_t motor_buffer[7][30];

#define SERVO_CMD_SYNC_WRITE 0x83
#define SERVO_ID_BROADCAST_TO_ALL 0xFE
#define SERVO_REG_TORQUE_ENABLE 0x28
#define SERVO_REG_TARGET_LOCATION 0x2A
#define SERVO_MAX_WAIT_TIME 15

/**
 * @brief structer to represent a robot arm.
 */
typedef struct{
	UART_HandleTypeDef *huart;
	uint8_t servo_count;
	uint8_t servo_buffer[7][30];
	uint8_t servo_buffer_ready[7];
} ServoArm_t;

void terminal_print_servo_data(ServoArm_t *arm);
void servo_write_byte(ServoArm_t *arm, uint8_t id, uint8_t start_addr, uint8_t num_bytes, ...);

/**
 * @brief writes multiple bytes to all the servos of an robot arm at the same time.
 * @param[in] arm					pointer to robot arm instance.
 * @param[in] start_addr			the address of the first register byte written to.
 * @param[in] num_bytes_per_servo	number of bytes every servo receives.
 * @param[in] data_buffer			an array containing all the data to send.
 */
void servo_sync_write(ServoArm_t *arm, uint8_t start_addr, uint8_t num_bytes_per_servo, const uint8_t *data_buffer);
void servo_read_all(ServoArm_t *arm);
void servo_read_id(ServoArm_t *arm, uint8_t id);
void servo_set_angle(ServoArm_t *arm, uint8_t id, float angle);
void servo_set_all_angle(ServoArm_t *arm, float *angle);
void servo_zero_point_calibration(ServoArm_t *arm);
void servo_min_max_calibration(ServoArm_t *arm);
void servo_max_torque_initialization(ServoArm_t *arm);
#endif /* INC_SERVO_H_ */
