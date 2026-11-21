#include "UART_maxcam.h"

#define UART5_FRAME_SIZE 8

static uint8_t rx_frame[UART5_FRAME_SIZE];
static uint8_t rx_count = 0;
/* Updated once per complete camera frame, independently for X and Y. */
#define UART5_AVERAGE_FRAMES 3
static uint8_t average_frames = UART5_DEFAULT_AVERAGE_FRAMES;
static int16_t xy_history[2][UART5_AVERAGE_FRAMES];
static int32_t xy_sum[2];
static uint8_t history_index = 0;
static uint8_t history_count = 0;
static volatile int16_t pending_target[3];
static volatile uint32_t uart5_time_ms = 0;
static uint32_t last_frame_ms = 0;
static uint32_t received_frame_id = 0;
static uint8_t zero_pair_active = 0;
static uint32_t zero_pair_since_ms = 0;
static Uart5Sample published_sample;
volatile uint8_t rv_data_ready = 0;

int16_t target_x = 0;
int16_t target_y = 0;
int16_t target_z = 0;
void Uart5_en_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // 1. 开启时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    // PC12 TX
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // PD2 RX
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    // 复用映射 UART5
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

    // 2. USART5 参数配置 115200 8N1
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &USART_InitStruct);

    // 3. 开启接收非空中断 RXNE
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART5, ENABLE);

    // 4. NVIC 中断优先级配置
    NVIC_InitStruct.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}
/* Called by the existing 1 ms TIM6 interrupt. */
void Uart5_Tick1ms(void)
{
    ++uart5_time_ms;
}

static void Uart5_ResetAverage(void)
{
    uint8_t axis;
    uint8_t i;

    for (axis = 0; axis < 2; ++axis)
    {
        xy_sum[axis] = 0;
        for (i = 0; i < UART5_AVERAGE_FRAMES; ++i)
            xy_history[axis][i] = 0;
    }
    history_index = 0;
    history_count = 0;
}

uint8_t Uart5_SetAverageFrames(uint8_t frames)
{
    uint32_t mask;
    if (frames < 1U || frames > UART5_AVERAGE_FRAMES) return 0;
    mask = __get_PRIMASK();
    __disable_irq();
    if (frames != average_frames)
    {
        average_frames = frames;
        Uart5_ResetAverage();
        /* Drop only the old filtered publication, preserving a partial RX frame. */
        rv_data_ready = 0;
    }
    __set_PRIMASK(mask);
    return 1;
}

uint8_t Uart5_GetAverageFrames(void)
{
    return average_frames;
}

/* Run at frame reception so a busy main loop cannot skip filter samples. */
static void Uart5_FilterFrame(void)
{
    uint8_t axis;
    uint16_t raw;
    int16_t sample;
    uint8_t both_zero = 1;
    uint32_t now = uart5_time_ms;

    /* A resumed stream starts a fresh average, even if main was busy. */
    if ((uint32_t)(now - last_frame_ms) >= UART5_RX_TIMEOUT_MS)
    {
        Uart5_ResetAverage();
        zero_pair_active = 0;
    }
    last_frame_ms = now;
    ++received_frame_id;

    if (history_count < average_frames)
        ++history_count;

    for (axis = 0; axis < 2; ++axis)
    {
        raw = ((uint16_t)rx_frame[1 + axis * 2] << 8)
            | rx_frame[2 + axis * 2];
        sample = (int16_t)((int32_t)raw - UART5_XY_OFFSET);
        if (sample != 0) both_zero = 0;
        xy_sum[axis] -= xy_history[axis][history_index];
        xy_history[axis][history_index] = sample;
        xy_sum[axis] += sample;
        pending_target[axis] = (int16_t)(xy_sum[axis] / history_count);
    }

    /* Watch actual decoded coordinates before averaging/calibration.
     * Check every frame so nonzero frames cannot be hidden by main-loop delays.
     */
    if (both_zero)
    {
        if (!zero_pair_active)
        {
            zero_pair_since_ms = now;
            zero_pair_active = 1;
        }
    }
    else
        zero_pair_active = 0;

    history_index = (history_index + 1) % average_frames;
    /* Preserve the auxiliary word without assuming it is a coordinate. */
    pending_target[2] = (int16_t)(((uint16_t)rx_frame[5] << 8) | rx_frame[6]);
    rv_data_ready = 1;
}

/* Fixed-length binary frame: 7B XH XL YH YL AUXH AUXL 7D.
 * Payload bytes may also equal 7B or 7D; check the tail at byte 8 only.
 */
static void Uart5_ReceiveByte(uint8_t value)
{
    uint8_t i;
    uint8_t next;

    if (rx_count == 0 && value != 0x7B)
        return;

    rx_frame[rx_count++] = value;
    if (rx_count < UART5_FRAME_SIZE)
        return;

    if (rx_frame[7] == 0x7D)
    {
        Uart5_FilterFrame();
        rx_count = 0;
    }
    else
    {
        /* Keep a possible next header after a missing/corrupted byte. */
        for (next = 1; next < UART5_FRAME_SIZE; ++next)
            if (rx_frame[next] == 0x7B)
                break;
        rx_count = UART5_FRAME_SIZE - next;
        for (i = 0; i < rx_count; ++i)
            rx_frame[i] = rx_frame[next + i];
    }
}

void UART5_IRQHandler(void)
{
    uint32_t status = UART5->SR;
    uint8_t value;

    if (status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
    {
        /* Reading SR then DR also clears receive error flags. */
        value = (uint8_t)UART5->DR;
        if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE))
        {
            rx_count = 0;
            return;
        }
        if (status & USART_SR_RXNE)
            Uart5_ReceiveByte(value);
    }
}

void Uart5_ProcessData(void)
{
    uint32_t primask;

    /* Check age and copy/reset state atomically with UART reception. */
    primask = __get_PRIMASK();
    __disable_irq();
    published_sample.zero_timeout = zero_pair_active &&
        (uint32_t)(uart5_time_ms - zero_pair_since_ms) >= UART5_ZERO_TIMEOUT_MS;
    if ((uint32_t)(uart5_time_ms - last_frame_ms) >= UART5_RX_TIMEOUT_MS)
    {
        target_x = 0;
        target_y = 0;
        target_z = 0;
        published_sample.x = 0;
        published_sample.y = 0;
        published_sample.valid = 0;
        rv_data_ready = 0;
        if (history_count != 0)
        {
            rx_count = 0;
            Uart5_ResetAverage();
        }
    }
    else if (rv_data_ready)
    {
        target_x = pending_target[0];
        target_y = pending_target[1];
        target_z = pending_target[2];
        published_sample.x = target_x;
        published_sample.y = target_y;
        published_sample.frame_id = received_frame_id;
        published_sample.timestamp_ms = last_frame_ms;
        published_sample.valid = 1;
        rv_data_ready = 0;
    }
    __set_PRIMASK(primask);
}

/* Published state is written only by main-loop processing, not by the ISR. */
void Uart5_ReadSample(Uart5Sample *sample)
{
    Uart5_ProcessData();
    *sample = published_sample;
}

uint32_t Uart5_GetTimeMs(void)
{
    return uart5_time_ms;
}

int16_t Uart5_getdata(uint8_t choosexy)
{
    if (choosexy == 0)
        return target_z;
    else if (choosexy == 1)
        return target_x;
    else
        return target_y;
}
