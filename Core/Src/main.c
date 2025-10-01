/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

/* Private define ------------------------------------------------------------*/
#define M_PI 3.14159265359f

#define SHOULDER_LENGTH 17.0f
#define ELBOW_LENGTH 23.0f
#define TOTAL_LENGTH 40.0f

#define BASE_HEIGHT 5.0f
#define SHOULDER_OFFSET 3.0f
#define SHOULDER_CORRECTION 12.0f
#define ELBOW_CORRECTION 18.0f

#define BASE_MIN_ANGLE    0.0f
#define BASE_MAX_ANGLE    180.0f
#define SHOULDER_MIN_ANGLE 30.0f
#define SHOULDER_MAX_ANGLE 150.0f
#define ELBOW_MIN_ANGLE   100.0f
#define ELBOW_MAX_ANGLE   180.0f

#define BASE_SERVO_CCR_MIN 1150
#define BASE_SERVO_CCR_MAX 5000
#define SHOULDER_SERVO_CCR_MIN 1250
#define SHOULDER_SERVO_CCR_MAX 5000
#define ELBOW_SERVO_CCR_MIN 200
#define ELBOW_SERVO_CCR_MAX 4500

#define SHOULDER_COMPENSATION 0.0f
#define ELBOW_COMPENSATION 0.0f

/* Private variables ---------------------------------------------------------*/
typedef struct {
    float base;
    float shoulder;
    float elbow;
} ArmAngles;

typedef struct {
    float x;
    float y; 
    float z;
} CartesianPoint;

ArmAngles current_angles = {90.0f, 90.0f, 165.0f}; // Начальные углы
CartesianPoint target_position = {2.0f, 0.0f, -1.0f}; // Пример начальной позиции
CartesianPoint actual_position = {0.0f, 0.0f, 0.0f}; // Актуальная позиция

uint8_t rx_buffer[64];
uint8_t rx_index = 0;
uint8_t command_ready = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static uint32_t map_angle_to_ccr(float angle, float min_angle, float max_angle, uint32_t ccr_min, uint32_t ccr_max);
static uint8_t inverse_kinematics(float x, float y, float z, ArmAngles* angles);
static void forward_kinematics(ArmAngles angles, CartesianPoint* point);
static uint8_t is_angle_valid(float angle, float min_angle, float max_angle);
static void process_command(uint8_t* command);
static void send_response(const char* response);
static void update_servos(void);

/* Private user code ---------------------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        uint8_t received_char = rx_buffer[rx_index];
        
        if (received_char == '\n' || received_char == '\r' || rx_index >= sizeof(rx_buffer)-1)
        {
            if (rx_index > 0)
            {
                rx_buffer[rx_index] = '\0';
                command_ready = 1;
            }
            rx_index = 0;
        }
        else
        {
            rx_buffer[rx_index++] = received_char;
        }
        
        HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        rx_index = 0;
        HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_USART2_UART_Init();

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Base
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // Shoulder
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // Elbow

    update_servos();
    
    send_response("Robotic Arm Ready\r\n");
    send_response("Commands: X,Y,Z | HOME | ANGLES | STATUSr\n");
    send_response("Example: X1.5 Y2.0 Z-1\r\n");

    HAL_UART_Receive_IT(&huart2, &rx_buffer[rx_index], 1);

    while (1)
    {
        if (command_ready)
        {
            process_command(rx_buffer);
            command_ready = 0;
        }
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
        HAL_Delay(500);
    }
}

/**
 * @brief  Преобразует угол в значение ШИМ для сервопривода с калибровкой.
 */
static uint32_t map_angle_to_ccr(float angle, float min_angle, float max_angle, uint32_t ccr_min, uint32_t ccr_max)
{
    if (angle < min_angle) angle = min_angle;
    if (angle > max_angle) angle = max_angle;
    
    return ccr_min + (uint32_t)((angle - min_angle) * 
                               (ccr_max - ccr_min) / 
                               (max_angle - min_angle));
}

/**
 * @brief  Проверяет, находится ли угол в допустимом диапазоне.
 */
static uint8_t is_angle_valid(float angle, float min_angle, float max_angle)
{
    return (angle >= min_angle && angle <= max_angle) ? 1 : 0;
}

/**
 * @brief  Улучшенная обратная кинематика с учетом реальной механики
 */
static uint8_t inverse_kinematics(float x, float y, float z, ArmAngles* angles)
{
    float corrected_x = x;
    float corrected_y = y; 
    float corrected_z = z - BASE_HEIGHT;
    
    if (corrected_x == 0.0f && corrected_y == 0.0f && corrected_z == 0.0f) {
        return 0;
    }
    
    float distance_squared = corrected_x*corrected_x + corrected_y*corrected_y + corrected_z*corrected_z;
    float distance_to_target = sqrtf(distance_squared);
    
    float min_reach = 5.0f;
    float max_reach = TOTAL_LENGTH * 0.9f;
    
    if (distance_to_target < min_reach || distance_to_target > max_reach) {
        return 0;
    }
		
    angles->base = atan2f(corrected_y, corrected_x) * 180.0f / M_PI;
    if (angles->base < 0) angles->base += 360.0f;
    
    float L1 = SHOULDER_LENGTH;
    float L2 = ELBOW_LENGTH;
    float r = sqrtf(corrected_x*corrected_x + corrected_y*corrected_y);
    
    float cos_elbow = (distance_squared - L1*L1 - L2*L2) / (2 * L1 * L2);
    
    if (cos_elbow < -1.0f || cos_elbow > 1.0f) {
        return 0;
    }
    
    angles->elbow = acosf(cos_elbow) * 180.0f / M_PI;
    
    float alpha = atan2f(corrected_z, r);
    float beta = atan2f(L2 * sinf(angles->elbow * M_PI / 180.0f), 
                       L1 + L2 * cosf(angles->elbow * M_PI / 180.0f));
    
    angles->shoulder = (alpha + beta) * 180.0f / M_PI;
    
    angles->shoulder += SHOULDER_CORRECTION;
    angles->elbow -= ELBOW_CORRECTION;
    
    if (!is_angle_valid(angles->base, BASE_MIN_ANGLE, BASE_MAX_ANGLE) ||
        !is_angle_valid(angles->shoulder, SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE) ||
        !is_angle_valid(angles->elbow, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE)) {
        return 0;
    }
    
    return 1;
}

/**
 * @brief  Улучшенная прямая кинематика
 */
static void forward_kinematics(ArmAngles angles, CartesianPoint* point)
{
    float base_rad = angles.base * M_PI / 180.0f;
    float shoulder_rad = (angles.shoulder - SHOULDER_CORRECTION) * M_PI / 180.0f;
    float elbow_rad = (angles.elbow + ELBOW_CORRECTION) * M_PI / 180.0f;
    
    float x_plane = SHOULDER_LENGTH * cosf(shoulder_rad) + 
                   ELBOW_LENGTH * cosf(shoulder_rad + elbow_rad);
    
    point->x = x_plane * cosf(base_rad);
    point->y = x_plane * sinf(base_rad);
    point->z = BASE_HEIGHT + SHOULDER_LENGTH * sinf(shoulder_rad) + 
               ELBOW_LENGTH * sinf(shoulder_rad + elbow_rad);
}

/**
 * @brief  Обновляет сервоприводы согласно текущим углам.
 */
static void update_servos(void)
{
    float base_angle = current_angles.base;
    float shoulder_angle = current_angles.shoulder + SHOULDER_COMPENSATION;
    float elbow_angle = current_angles.elbow + ELBOW_COMPENSATION;

    if (shoulder_angle > SHOULDER_MAX_ANGLE) shoulder_angle = SHOULDER_MAX_ANGLE;
    if (shoulder_angle < SHOULDER_MIN_ANGLE) shoulder_angle = SHOULDER_MIN_ANGLE;
    if (elbow_angle > ELBOW_MAX_ANGLE) elbow_angle = ELBOW_MAX_ANGLE;
    if (elbow_angle < ELBOW_MIN_ANGLE) elbow_angle = ELBOW_MIN_ANGLE;

    uint32_t ccr1 = map_angle_to_ccr(base_angle, BASE_MIN_ANGLE, BASE_MAX_ANGLE, BASE_SERVO_CCR_MIN, BASE_SERVO_CCR_MAX);
    uint32_t ccr2 = map_angle_to_ccr(shoulder_angle, SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE, SHOULDER_SERVO_CCR_MIN, SHOULDER_SERVO_CCR_MAX);
    uint32_t ccr3 = map_angle_to_ccr(elbow_angle, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE, ELBOW_SERVO_CCR_MIN, ELBOW_SERVO_CCR_MAX);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
}

/**
 * @brief  Отправляет ответ через UART.
 */
static void send_response(const char* response)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)response, strlen(response), 100);
}
/**
 * @brief  Обрабатывает полученную команду.
 */
static void process_command(uint8_t* command)
{
    char response[128];
    
    for(int i = 0; command[i]; i++) {
        command[i] = toupper(command[i]);
    }
    
    if (strstr((char*)command, "HOME") != NULL)
    {
        float test_positions[3] = {-1.9f, 0.0f, -1.0f};
        if (inverse_kinematics(test_positions[0], test_positions[1], test_positions[2], &current_angles)) {
            update_servos();
            forward_kinematics(current_angles, &actual_position);
            
            char msg[128];
            snprintf(msg, sizeof(msg),
                    "OK: Moving to home position\r\n"
                    "Target: X%.1f Y%.1f Z%.1f\r\n"
                    "Actual: X%.1f Y%.1f Z%.1f\r\n"
                    "Angles: Base%.1f Shoulder%.1f Elbow%.1f\r\n",
                    target_position.x, target_position.y, target_position.z,
                    actual_position.x, actual_position.y, actual_position.z,
                    current_angles.base, current_angles.shoulder, current_angles.elbow);
            send_response(msg);
        } else {
            current_angles.base = 90.0f;
            current_angles.shoulder = 80.0f;
            current_angles.elbow = 140.0f;
            update_servos();
            forward_kinematics(current_angles, &actual_position);
            send_response("OK: Moving to safe default position\r\n");
        }
    }
    else if (strstr((char*)command, "STATUS") != NULL)
    {
        forward_kinematics(current_angles, &actual_position);
        snprintf(response, sizeof(response), 
                "Target: X%.1f Y%.1f Z%.1f\r\nActual: X%.1f Y%.1f Z%.1f\r\nAngles: Base%.1f Shoulder%.1f Elbow%.1f\r\n",
                target_position.x, target_position.y, target_position.z,
                actual_position.x, actual_position.y, actual_position.z,
                current_angles.base, current_angles.shoulder, current_angles.elbow);
        send_response(response);
    }
    else if (strstr((char*)command, "ANGLES") != NULL)
    {
        float base, shoulder, elbow;
        if (sscanf((char*)command, "ANGLES %f %f %f", &base, &shoulder, &elbow) == 3)
        {
            if (is_angle_valid(base, BASE_MIN_ANGLE, BASE_MAX_ANGLE) &&
                is_angle_valid(shoulder, SHOULDER_MIN_ANGLE, SHOULDER_MAX_ANGLE) &&
                is_angle_valid(elbow, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE))
            {
                current_angles.base = base;
                current_angles.shoulder = shoulder;
                current_angles.elbow = elbow;
                update_servos();
                forward_kinematics(current_angles, &actual_position);
                snprintf(response, sizeof(response), "OK: Angles set to %.1f, %.1f, %.1f\r\n", base, shoulder, elbow);
                send_response(response);
            }
            else
            {
                send_response("ERROR: Invalid angles\r\n");
            }
        }
        else
        {
            send_response("ERROR: Usage: ANGLES base shoulder elbow\r\n");
        }
    }
    else
    {
        float x = target_position.x;
        float y = target_position.y;
        float z = target_position.z;
        
        char* token = strtok((char*)command, " ,");
        while (token != NULL)
        {
            if (token[0] == 'X' && sscanf(token + 1, "%f", &x) == 1) {}
            else if (token[0] == 'Y' && sscanf(token + 1, "%f", &y) == 1) {}
            else if (token[0] == 'Z' && sscanf(token + 1, "%f", &z) == 1) {}
            token = strtok(NULL, " ,");
        }
        
        if (x != target_position.x || y != target_position.y || z != target_position.z)
        {
            target_position.x = x;
            target_position.y = y;
            target_position.z = z;
            
            if (inverse_kinematics(target_position.x, target_position.y, target_position.z, &current_angles))
            {
                update_servos();
                forward_kinematics(current_angles, &actual_position);
                snprintf(response, sizeof(response), "OK: Moving to X%.1f Y%.1f Z%.1f\r\n", x, y, z);
                send_response(response);
            }
            else
            {
                snprintf(response, sizeof(response), "ERROR: Position X%.1f Y%.1f Z%.1f unreachable\r\n", x, y, z);
                send_response(response);
            }
        }
        else
        {
            send_response("OK: Position unchanged\r\n");
        }
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|
                                  RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}