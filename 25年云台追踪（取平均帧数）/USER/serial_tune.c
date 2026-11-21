#include "stm32f4xx.h"
#include "serial_tune.h"
#include "UART_maxcam.h"
#include "aim_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define TUNE_DEFAULTS { AIM_CENTER_X, AIM_CENTER_Y, AIM_FAR_KP_X, AIM_FAR_KP_Y, \
    AIM_MAX_RPM_X, AIM_MAX_RPM_Y, AIM_SLEW_X, AIM_SLEW_Y, \
    AIM_MOTOR_ACC_X, AIM_MOTOR_ACC_Y, AIM_NEAR_KP_X, AIM_NEAR_KD_X, AIM_SHAPE_X, AIM_NEAR_KD_Y, \
    AIM_NEAR_KP_Y, AIM_FAR_KD_X, AIM_FAR_KD_Y, AIM_BOOST_START_ERROR, AIM_BOOST_FULL_ERROR, \
    AIM_BOOST_START_ERROR, AIM_BOOST_FULL_ERROR, AIM_SHAPE_Y }
#define RX_SIZE 128U
#define TX_SIZE 256U
TuneParams tune = TUNE_DEFAULTS;
static const TuneParams defaults = TUNE_DEFAULTS;
static volatile uint8_t rx[RX_SIZE];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_fault;
static char tx[TX_SIZE];
static volatile uint16_t tx_pos, tx_len;
static char line[64];
static uint8_t line_len, discard_line;
static uint32_t last_plot_ms, command_seq;
static int command_result;
static uint8_t plot_enabled = 1;

void Tune_IRQHandler(void)
{
    uint32_t status = USART1->SR;
    uint16_t next;
    uint8_t value;
    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
    {
        value = (uint8_t)USART1->DR;
        next = (rx_head + 1U) % RX_SIZE;
        if ((status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) || next == rx_tail)
            rx_fault = 1;
        else
        {
            rx[rx_head] = value;
            rx_head = next;
        }
    }
    if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
        if (tx_pos < tx_len)
            USART1->DR = (uint8_t)tx[tx_pos++];
        else
        {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
            tx_len = 0;
        }
    }
}

/* All parameter writes happen in main, before the next control update. */
static int Tune_Command(char *command)
{
    char key[16], value_text[32], extra;
    char *end;
    float value;
    union { float number; uint32_t bits; } checked;
    float *dest = 0;
    float low = 0.0f, high = 0.0f;
    int count;
    while (isspace((unsigned char)*command)) ++command;
    if (strcmp(command, "RESET") == 0)
    {
        tune = defaults;
        Uart5_SetAverageFrames(UART5_DEFAULT_AVERAGE_FRAMES);
        plot_enabled = 1;
        return 1;
    }
    count = sscanf(command, "SET %15s %31s %c", key, value_text, &extra);
    if (count != 2) return -1;
    value = (float)strtod(value_text, &end);
    if (*end || end == value_text) return -1;
    checked.number = value;
    if ((checked.bits & 0x7F800000UL) == 0x7F800000UL) return -1;
    if (strcmp(key, "PLOT") == 0)
    {
        if (!(value == 0.0f || value == 1.0f)) return -1;
        plot_enabled = (uint8_t)value;
        return 1;
    }
    if (strcmp(key, "AVG") == 0)
    {
        if (!(value >= 1.0f && value <= 3.0f) || value != (float)(uint8_t)value) return -1;
        return Uart5_SetAverageFrames((uint8_t)value) ? 1 : -1;
    }
    if (strcmp(key, "SHAPEX") == 0)
    {
        if (!(value == 0.0f || value == 1.0f)) return -1;
        tune.shape_x = (uint8_t)value;
        return 1;
    }
    if (strcmp(key, "SHAPEY") == 0)
    {
        if (!(value == 0.0f || value == 1.0f)) return -1;
        tune.shape_y = (uint8_t)value;
        return 1;
    }
    if (strcmp(key, "KPX") == 0 || strcmp(key, "KPFX") == 0) { dest = &tune.kp_x; high = 2.0f; }
    else if (strcmp(key, "KPNX") == 0) { dest = &tune.near_kp_x; high = 2.0f; }
    else if (strcmp(key, "KDX") == 0 || strcmp(key, "KDNX") == 0) { dest = &tune.kd_x; high = 0.10f; }
    else if (strcmp(key, "KDY") == 0 || strcmp(key, "KDNY") == 0) { dest = &tune.kd_y; high = 0.10f; }
    else if (strcmp(key, "KPY") == 0 || strcmp(key, "KPFY") == 0) { dest = &tune.kp_y; high = 2.0f; }
    else if (strcmp(key, "KPNY") == 0) { dest = &tune.near_kp_y; high = 2.0f; }
    else if (strcmp(key, "KDFX") == 0) { dest = &tune.far_kd_x; high = 0.10f; }
    else if (strcmp(key, "KDFY") == 0) { dest = &tune.far_kd_y; high = 0.10f; }
    else if (strcmp(key, "NEARX") == 0) { dest = &tune.near_error_x; low = AIM_RESTART_ERROR; high = 1000.0f; }
    else if (strcmp(key, "FARX") == 0) { dest = &tune.far_error_x; low = AIM_RESTART_ERROR; high = 2000.0f; }
    else if (strcmp(key, "NEARY") == 0) { dest = &tune.near_error_y; low = AIM_RESTART_ERROR; high = 1000.0f; }
    else if (strcmp(key, "FARY") == 0) { dest = &tune.far_error_y; low = AIM_RESTART_ERROR; high = 2000.0f; }
    else if (strcmp(key, "CX") == 0) { dest = &tune.center_x; low = -32768.0f; high = 32767.0f; }
    else if (strcmp(key, "CY") == 0) { dest = &tune.center_y; low = -32768.0f; high = 32767.0f; }
    /* Online tuning cannot raise speed above the existing compiled limits. */
    else if (strcmp(key, "MAXX") == 0) { dest = &tune.max_x; low = AIM_MIN_RPM; high = AIM_MAX_RPM_X; }
    else if (strcmp(key, "MAXY") == 0) { dest = &tune.max_y; low = AIM_MIN_RPM; high = AIM_MAX_RPM_Y; }
    else if (strcmp(key, "SLEWX") == 0) { dest = &tune.slew_x; low = 1.0f; high = 500.0f; }
    else if (strcmp(key, "SLEWY") == 0) { dest = &tune.slew_y; low = 1.0f; high = 500.0f; }
    else if (strcmp(key, "ACCX") == 0) { dest = &tune.acc_x; low = 1.0f; high = 255.0f; }
    else if (strcmp(key, "ACCY") == 0) { dest = &tune.acc_y; low = 1.0f; high = 255.0f; }
    if (!dest || !(value >= low && value <= high)) return -1; /* Also rejects NaN/Inf. */
    if ((dest == &tune.acc_x || dest == &tune.acc_y ||
         dest == &tune.max_x || dest == &tune.max_y) && value != (float)(uint16_t)value)
        return -1;
    if ((dest == &tune.near_error_x && value >= tune.far_error_x) ||
        (dest == &tune.far_error_x && value <= tune.near_error_x) ||
        (dest == &tune.near_error_y && value >= tune.far_error_y) ||
        (dest == &tune.far_error_y && value <= tune.near_error_y)) return -1;
    *dest = value;
    return 1;
}

void Tune_Poll(void)
{
    unsigned int budget = RX_SIZE;
    uint8_t value;
    uint32_t mask;
    if (rx_fault)
    {
        mask = __get_PRIMASK();
        __disable_irq();
        rx_tail = rx_head;
        rx_fault = 0;
        __set_PRIMASK(mask);
        line_len = 0;
        discard_line = 1;
        command_result = -1;
        ++command_seq;
    }
    while (budget-- && rx_tail != rx_head)
    {
        value = rx[rx_tail];
        rx_tail = (rx_tail + 1U) % RX_SIZE;
        if (value == '\r' || value == '\n' || value == ';')
        {
            if (discard_line)
            {
                discard_line = 0;
                line_len = 0;
            }
            else if (line_len)
            {
                line[line_len] = 0;
                command_result = Tune_Command(line);
                ++command_seq;
                line_len = 0;
            }
        }
        else if (!discard_line)
        {
            if (value < 32 || value > 126 || line_len >= sizeof(line) - 1U)
            {
                discard_line = 1;
                line_len = 0;
                command_result = -1;
                ++command_seq;
            }
            else line[line_len++] = (char)value;
        }
    }
}

void Tune_Plot(uint32_t now, int16_t x, int16_t y,
               int16_t cmd_x, int16_t cmd_y, uint8_t valid, uint8_t active)
{
    int size;
    if (!plot_enabled || (uint32_t)(now - last_plot_ms) < 50U || tx_len != 0)
        return;
    last_plot_ms = now;
    /* Pure numeric CSV: no text acknowledgements mixed into the waveform. */
    size = snprintf(tx, sizeof(tx),
        "%.2f,%d,%.2f,%d,%.2f,%.2f,%d,%d,%.4f,%.4f,%u,%u,%lu,%d\r\n",
        (double)tune.center_x, (int)x, (double)tune.center_y, (int)y,
        (double)((float)x - tune.center_x), (double)((float)y - tune.center_y),
        (int)cmd_x, (int)cmd_y, (double)tune.kp_x, (double)tune.kp_y,
        (unsigned int)valid, (unsigned int)active, (unsigned long)command_seq, command_result);
    if (size <= 0 || size >= (int)sizeof(tx)) return;
    tx_pos = 0;
    tx_len = (uint16_t)size;
    __DMB();
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}
