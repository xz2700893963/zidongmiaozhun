/* Compile with function sections and entry test_main, then run in a Cortex-M
 * emulator. Include the real implementation so frame gating and timeout paths
 * are exercised along with the filter and speed calculation. Motor TX is mocked.
 */
#include "../UART_maxcam.c"
#define main firmware_main
#include "../main.c"
#undef main

uint8_t main_mode = 1, mode1, mode2, flag, flag_z, number;
static int x_command, y_command, tx_count;
static int x_stops, y_stops;
static int x_zero_speeds, y_zero_speeds;

void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
    (void)addr; (void)snF;
    x_command = 0;
    ++x_stops;
    ++tx_count;
}

void Emm_V5_Stop_Now3(uint8_t addr, bool snF)
{
    (void)addr; (void)snF;
    y_command = 0;
    ++y_stops;
    ++tx_count;
}

void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel,
                       uint8_t acc, bool snF)
{
    (void)addr; (void)acc; (void)snF;
    x_command = dir == AIM_X_POSITIVE_DIR ? (int)vel : -(int)vel;
    if (vel == 0) ++x_zero_speeds;
    ++tx_count;
}

void Emm_V5_Vel_Control3(uint8_t addr, uint8_t dir, uint16_t vel,
                        uint8_t acc, bool snF)
{
    (void)addr; (void)acc; (void)snF;
    y_command = dir == AIM_Y_POSITIVE_DIR ? (int)vel : -(int)vel;
    if (vel == 0) ++y_zero_speeds;
    ++tx_count;
}

static void reset_test(void)
{
    Uart5_ResetAverage();
    rx_count = 0;
    uart5_time_ms = 0;
    last_frame_ms = 0;
    received_frame_id = 0;
    zero_pair_active = 0;
    zero_pair_since_ms = 0;
    rv_data_ready = 0;
    target_x = target_y = target_z = 0;
    published_sample.valid = 0;
    published_sample.zero_timeout = 0;
    aim_active = aim_have_frame = aim_stop_sent = 0;
    AimAxis_Reset(&aim_x);
    AimAxis_Reset(&aim_y);
    x_command = y_command = tx_count = 0;
    x_stops = y_stops = 0;
    x_zero_speeds = y_zero_speeds = 0;
    main_mode = 2;
}

static void tick(unsigned int ms)
{
    while (ms--) Uart5_Tick1ms();
}

static void feed_frame(int16_t x_value, int16_t y_value)
{
    uint16_t x_word = (uint16_t)((int32_t)x_value + UART5_XY_OFFSET);
    uint16_t y_word = (uint16_t)((int32_t)y_value + UART5_XY_OFFSET);
    Uart5_ReceiveByte(0x7B);
    Uart5_ReceiveByte((uint8_t)(x_word >> 8));
    Uart5_ReceiveByte((uint8_t)x_word);
    Uart5_ReceiveByte((uint8_t)(y_word >> 8));
    Uart5_ReceiveByte((uint8_t)y_word);
    Uart5_ReceiveByte(0x11);
    Uart5_ReceiveByte(0xD7);
    Uart5_ReceiveByte(0x7D);
}

#define CHECK(condition, code) do { if (!(condition)) return code; } while (0)

int test_main(void)
{
    Uart5Sample sample;
    AimAxis axis;
    int i, old_x, old_y, sends;
    float speed;

    reset_test();
    feed_frame(10, -30); Uart5_ReadSample(&sample);
    CHECK(sample.valid && sample.x == 10 && sample.y == -30, 1);
    feed_frame(20, -60); Uart5_ReadSample(&sample);
    CHECK(sample.x == 15 && sample.y == -45, 2);
    feed_frame(30, -90); Uart5_ReadSample(&sample);
    CHECK(sample.x == 20 && sample.y == -60, 3);
    feed_frame(40, -120); Uart5_ReadSample(&sample);
    CHECK(sample.x == 30 && sample.y == -90, 4);

    /* Every physical frame contributes even when main does not poll. */
    reset_test();
    feed_frame(10, -10); feed_frame(20, -20); feed_frame(30, -30); feed_frame(40, -40);
    Uart5_ReadSample(&sample);
    CHECK(sample.x == 30 && sample.y == -30 && sample.frame_id == 4, 5);
    tick(UART5_RX_TIMEOUT_MS - 1); Uart5_ReadSample(&sample);
    CHECK(sample.valid, 6);
    tick(1); Uart5_ReadSample(&sample);
    CHECK(!sample.valid && target_x == 0 && target_y == 0, 7);

    /* Polling during a partial resumed frame must not destroy that frame. */
    Uart5_ReceiveByte(0x7B); Uart5_ProcessData();
    Uart5_ReceiveByte(0x80); Uart5_ProcessData();
    Uart5_ReceiveByte(0x04); Uart5_ProcessData();
    Uart5_ReceiveByte(0x80); Uart5_ProcessData();
    Uart5_ReceiveByte(0x01); Uart5_ProcessData();
    Uart5_ReceiveByte(0x11); Uart5_ReceiveByte(0xD7); Uart5_ReceiveByte(0x7D);
    Uart5_ReadSample(&sample);
    CHECK(sample.valid && sample.x == 4 && sample.y == 1, 8);

    reset_test();
    uart5_time_ms = 0xFFFFFFF0U;
    feed_frame(5, 3); tick(20); Uart5_ReadSample(&sample);
    CHECK(sample.valid, 9);
    tick(UART5_RX_TIMEOUT_MS - 20); Uart5_ReadSample(&sample);
    CHECK(!sample.valid, 10);

    reset_test();
    feed_frame(-1155, -645); Uart5_ReadSample(&sample);
    CHECK(sample.x == -1155 && sample.y == -645, 11); /* 7B/7D in payload */
    Uart5_ReceiveByte(0x7B);
    for (i = 0; i < 7; ++i) Uart5_ReceiveByte(0x00);
    Uart5_ReadSample(&sample);
    CHECK(sample.frame_id == 1 && sample.x == -1155, 12);

    AimAxis_Reset(&axis);
    CHECK(AimAxis_Update(&axis, 3, .12f, 15, 40, .02f) == 0, 13);
    speed = AimAxis_Update(&axis, 1000, .12f, 15, 40, .02f);
    CHECK(speed > .95f && speed < .97f, 14);
    CHECK(AimAxis_Update(&axis, 2, .12f, 15, 40, .02f) == 0, 15);
    CHECK(AimAxis_Update(&axis, 3, .12f, 15, 40, .02f) == 0, 16);
    AimAxis_Update(&axis, 8, .05f, 10, 25, .02f);
    speed = AimAxis_Update(&axis, 3, .05f, 10, 25, .02f);
    CHECK(AimAxis_Rpm(speed) == 1, 32); /* Small active errors can converge. */
    CHECK(AimAxis_Update(&axis, 2, .05f, 10, 25, .02f) == 0, 33);

    reset_test();
    feed_frame(1005, -997); ennmotor();
    CHECK(x_command == 1 && y_command == -1 && tx_count == 2, 17);
    sends = tx_count;
    for (i = 0; i < 100; ++i) ennmotor();
    CHECK(tx_count == sends, 18); /* No repeated-frame control updates. */
    for (i = 0; i < 100; ++i)
    {
        old_x = x_command; old_y = y_command;
        tick(20); feed_frame(1005, -997); ennmotor();
        CHECK(x_command >= 0 && x_command <= 15, 19);
        CHECK(y_command <= 0 && y_command >= -10, 20);
        CHECK(x_command - old_x <= 1 && old_y - y_command <= 1, 21);
    }
    CHECK(x_command == 15 && y_command == -10, 22);
    for (i = 0; i < 100; ++i)
    {
        old_x = x_command; old_y = y_command;
        tick(20); feed_frame(-995, 1003); ennmotor();
        CHECK(x_command >= -15 && x_command <= 15, 23);
        CHECK(y_command >= -10 && y_command <= 10, 24);
        CHECK(!(old_x > 0 && x_command < 0), 25);
        CHECK(!(old_y < 0 && y_command > 0), 26);
    }
    CHECK(x_command == -15 && y_command == 10, 27);
    sends = tx_count;
    tick(UART5_RX_TIMEOUT_MS - 1); ennmotor();
    CHECK(x_command == -15 && y_command == 10 && x_stops == 0 && y_stops == 0, 34);
    tick(1); ennmotor();
    CHECK(x_command == 0 && y_command == 0 && !aim_active, 28);
    CHECK(x_stops == 0 && y_stops == 0, 35); /* Use velocity commands, not stop opcodes. */
    CHECK(x_zero_speeds > 0 && y_zero_speeds > 0, 38);
    CHECK(tx_count == sends + 2, 39); /* One zero-velocity command per axis. */
    sends = tx_count; ennmotor();
    CHECK(tx_count == sends, 29);
    feed_frame(1005, -997); ennmotor();
    CHECK(x_command == 1 && y_command == -1, 30);
    main_mode = 1; ennmotor();
    CHECK(x_command == 0 && y_command == 0 && !aim_active, 31);
    CHECK(x_stops == 0 && y_stops == 0, 36);
    reset_test();
    tick(UART5_RX_TIMEOUT_MS); ennmotor();
    CHECK(x_zero_speeds == 1 && y_zero_speeds == 1, 37); /* No first frame: send zero RPM. */

    /* Fresh zero packets must not keep the coordinate controller running. */
    reset_test();
    feed_frame(0, 0); ennmotor();
    for (i = 0; i < (int)(UART5_ZERO_TIMEOUT_MS / 20U) - 1; ++i)
    {
        tick(20); feed_frame(0, 0); ennmotor();
    }
    tick(19); feed_frame(0, 0); ennmotor();
    Uart5_ReadSample(&sample);
    CHECK(sample.valid && !sample.zero_timeout, 40); /* One ms before configured threshold. */
    tick(1); ennmotor(); Uart5_ReadSample(&sample);
    CHECK(sample.valid && sample.zero_timeout, 41); /* Still receiving frames. */
    CHECK(x_command == 0 && y_command == 0 && !aim_active, 42);
    sends = tx_count;
    for (i = 0; i < 10; ++i)
    {
        tick(20); feed_frame(0, 0); ennmotor();
    }
    CHECK(x_command == 0 && y_command == 0 && tx_count == sends, 43);
    tick(20); feed_frame(1005, 1003); ennmotor();
    CHECK(aim_active && x_command == 1 && y_command == 1, 44);

    /* A nonzero frame resets the zero timer even if main never saw it,
     * or averaging would have rounded that one-pixel value to zero.
     */
    reset_test(); feed_frame(0, 0);
    tick(UART5_ZERO_TIMEOUT_MS / 2); feed_frame(1, 0); feed_frame(0, 0);
    tick(UART5_ZERO_TIMEOUT_MS / 2); feed_frame(0, 0); Uart5_ReadSample(&sample);
    CHECK(!sample.zero_timeout, 45);
    tick(UART5_ZERO_TIMEOUT_MS / 2); feed_frame(0, 0); Uart5_ReadSample(&sample);
    CHECK(sample.valid && sample.zero_timeout, 46);

    reset_test(); feed_frame(0, 1);
    tick(1999); feed_frame(0, 1); tick(1); Uart5_ReadSample(&sample);
    CHECK(sample.valid && !sample.zero_timeout, 47);

    /* Preserve near-center tuning; smoothly boost unsaturated larger errors. */
    AimAxis_Reset(&axis);
    speed = AimAxis_Update(&axis, 10, .12f, 15, 1000, .02f);
    CHECK(speed > 1.199f && speed < 1.201f, 48);
    speed = AimAxis_Update(&axis, 20, .12f, 15, 1000, .02f);
    CHECK(speed > 2.399f && speed < 2.401f, 49);
    speed = AimAxis_Update(&axis, 40, .12f, 15, 1000, .02f);
    CHECK(speed > 5.159f && speed < 5.161f, 50);
    speed = AimAxis_Update(&axis, 60, .12f, 15, 1000, .02f);
    CHECK(speed > 8.279f && speed < 8.281f, 51);
    speed = AimAxis_Update(&axis, -60, .12f, 15, 40, .02f);
    CHECK(speed > 7.479f && speed < 7.481f, 52); /* Original braking, no instant reversal. */
    return 0;
}
