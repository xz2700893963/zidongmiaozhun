#ifndef SERIAL_TUNE_H
#define SERIAL_TUNE_H

#include <stdint.h>

typedef struct {
    float center_x, center_y;
    float kp_x, kp_y;
    float max_x, max_y;
    float slew_x, slew_y;
    float acc_x, acc_y;
    float near_kp_x, kd_x;
    uint8_t shape_x;
    float kd_y;
    float near_kp_y, far_kd_x, far_kd_y;
    float near_error_x, far_error_x, near_error_y, far_error_y;
    uint8_t shape_y;
} TuneParams;

extern TuneParams tune;
/* Called only from USART1_IRQHandler; no parsing/formatting inside the ISR. */
void Tune_IRQHandler(void);
void Tune_Poll(void);
void Tune_Plot(uint32_t now, int16_t x, int16_t y,
               int16_t cmd_x, int16_t cmd_y, uint8_t valid, uint8_t active);

#endif
