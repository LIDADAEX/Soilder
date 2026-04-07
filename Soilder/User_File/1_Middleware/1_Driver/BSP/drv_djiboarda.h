/**
 * @file drv_BoardC_mpu6500.h
 * @author lidada (1940868801@qq.com)
 * @brief C板的板级支持包
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2023-09-28 1.1 新增A板MPU6500的IST8310开关
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

#ifndef DRV_DJIBOARDC_H
#define DRV_DJIBOARDC_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

// C板引脚别名
/* --- CAN 1 --- */
#define BoardC_CAN1_RX_Pin            GPIO_PIN_0
#define BoardC_CAN1_RX_GPIO_Port      GPIOD
#define BoardC_CAN1_TX_Pin            GPIO_PIN_1
#define BoardC_CAN1_TX_GPIO_Port      GPIOD

/* --- CAN 2 --- */
#define BoardC_CAN2_TX_ALT_Pin        GPIO_PIN_5
#define BoardC_CAN2_TX_ALT_GPIO_Port  GPIOB
#define BoardC_CAN2_TX_Pin            GPIO_PIN_6
#define BoardC_CAN2_TX_GPIO_Port      GPIOB

/* --- USART 1 --- */
#define BoardC_USART1_RX_Pin          GPIO_PIN_10
#define BoardC_USART1_RX_GPIO_Port    GPIOA
#define BoardC_USART1_TX_Pin          GPIO_PIN_9
#define BoardC_USART1_TX_GPIO_Port    GPIOA

/* --- USART 2 --- */
#define BoardC_USART2_RX_Pin          GPIO_PIN_3
#define BoardC_USART2_RX_GPIO_Port    GPIOA
#define BoardC_USART2_TX_Pin          GPIO_PIN_2
#define BoardC_USART2_TX_GPIO_Port    GPIOA

/* --- USART 3 --- */
#define BoardC_USART3_RX_Pin          GPIO_PIN_11
#define BoardC_USART3_RX_GPIO_Port    GPIOC
#define BoardC_USART3_TX_Pin          GPIO_PIN_10
#define BoardC_USART3_TX_GPIO_Port    GPIOC

/* --- UART 4 --- */
#define BoardC_UART4_RX_Pin           GPIO_PIN_1
#define BoardC_UART4_RX_GPIO_Port     GPIOA
#define BoardC_UART4_TX_Pin           GPIO_PIN_0
#define BoardC_UART4_TX_GPIO_Port     GPIOA

/* --- UART 5 --- */
#define BoardC_UART5_RX_Pin           GPIO_PIN_2
#define BoardC_UART5_RX_GPIO_Port     GPIOD
#define BoardC_UART5_TX_Pin           GPIO_PIN_12
#define BoardC_UART5_TX_GPIO_Port     GPIOC

/* --- USART 6 --- */
#define BoardC_USART6_RX_Pin          GPIO_PIN_7
#define BoardC_USART6_RX_GPIO_Port    GPIOC
#define BoardC_USART6_TX_Pin          GPIO_PIN_6
#define BoardC_USART6_TX_GPIO_Port    GPIOC

/* --- USB (DP/DM) --- */
#define BoardC_USB_DM_Pin             GPIO_PIN_11
#define BoardC_USB_DM_GPIO_Port       GPIOA
#define BoardC_USB_DP_Pin             GPIO_PIN_12
#define BoardC_USB_DP_GPIO_Port       GPIOA

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
