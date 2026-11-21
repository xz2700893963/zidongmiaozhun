#ifndef __UART_MAIXCAM_H
#define __UART_MAIXCAM_H

// 标准库所需头文件
#include "stm32f4xx.h"
#include <stdlib.h>

// 单轴数据最大缓存长度
#define RV_MAX_LEN 10

#define UART5_DEFAULT_AVERAGE_FRAMES 2U
uint8_t Uart5_SetAverageFrames(uint8_t frames);
uint8_t Uart5_GetAverageFrames(void);

/* Provisional: camera sends XY + 32768, high byte first. */
#define UART5_XY_OFFSET 32768L

/* Clear coordinates after this long without a complete camera frame. */
#define UART5_RX_TIMEOUT_MS 2000U
#define UART5_ZERO_TIMEOUT_MS 1000U
void Uart5_Tick1ms(void);
typedef struct
{
    int16_t x;
    int16_t y;
    uint32_t frame_id;
    uint32_t timestamp_ms;
    uint8_t valid;
    uint8_t zero_timeout; /* Both decoded coordinates stayed zero for 2 s. */
} Uart5Sample;

void Uart5_ReadSample(Uart5Sample *sample);
uint32_t Uart5_GetTimeMs(void);

// 外部全局变量声明
extern volatile uint8_t rv_data_ready;
extern int16_t target_x;
extern int16_t target_y;
extern int16_t target_z;

// 函数声明
void Uart5_en_Init(void);// UART5串口初始化（标准库）
void Uart5_ProcessData(void);// 解析串口接收的三轴数据
int16_t Uart5_getdata(uint8_t choosexy);// 获取对应轴目标数据

#endif

