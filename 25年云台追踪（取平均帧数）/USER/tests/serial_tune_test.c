/* Exercise the real parser, RX framing, telemetry and camera->motor path. */
#include "../serial_tune.c"
#include <locale.h>
#define test_main previous_aim_test_main
#include "aim_stability_test.c"
#undef test_main

static void command(const char *text)
{
    while (*text)
    {
        rx[rx_head] = (uint8_t)*text++;
        rx_head = (rx_head + 1U) % RX_SIZE;
    }
    Tune_Poll();
}

int test_main(void)
{
    int i, commas;
    uint32_t seq;
    AimAxis normal_axis, shaped_axis;
    AimDamping damping;
    float old_output, shaped_output;
    /* The emulator enters here directly, bypassing the normal C startup. */
    setlocale(LC_ALL, "C");
    CHECK(tune.kp_x == .58f && tune.kd_x == .010f && !tune.shape_x, 701);
    CHECK(tune.shape_y && tune.near_kp_y == .18f && tune.kp_y == .24f &&
          tune.kd_y == .003f && tune.far_kd_y == .0005f, 702);
    CHECK(tune.near_error_y == 7 && tune.far_error_y == 24, 703);
    reset_test();
    command("SET KPX 0.30;");
    CHECK(tune.kp_x == 0.30f && command_result == 1, 101);
    command("SET KPY 0.07\r\nSET CX -5;SET CY 8;");
    CHECK(tune.kp_y == 0.07f && tune.center_x == -5 && tune.center_y == 8, 102);
    seq = command_seq;
    command("SET KPX ");
    CHECK(command_seq == seq && tune.kp_x == 0.30f, 103);
    command("0.25;");
    CHECK(tune.kp_x == 0.25f && command_seq == seq + 1, 104);
    command("SET KPX nan;");
    CHECK(command_result == -1 && tune.kp_x == 0.25f, 105);
    command("SET KPX inf;SET KPX -1;SET KPX 3;");
    CHECK(command_result == -1 && tune.kp_x == 0.25f, 106);
    command("SET KPX 0.9bad;SET KPX 0.9 extra;");
    CHECK(command_result == -1 && tune.kp_x == 0.25f, 107);
    command("SET MAXX 16;SET MAXY 11;SET SLEWX 0;SET ACCX 1.5;");
    CHECK(command_result == -1 && tune.max_x == AIM_MAX_RPM_X && tune.acc_x == AIM_MOTOR_ACC_X, 108);
    command("SET MAXX 4.5;");
    CHECK(command_result == -1 && tune.max_x == AIM_MAX_RPM_X, 125);
    command("SET SLEWX 50;SET SLEWY 30;SET ACCX 90;SET ACCY 50;");
    CHECK(tune.slew_x == 50 && tune.slew_y == 30 && tune.acc_x == 90 && tune.acc_y == 50, 109);
    command("SET PLOT 0;");
    CHECK(!plot_enabled, 110);
    command("RESET;");
    CHECK(plot_enabled && tune.kp_x == AIM_KP_X && tune.center_y == AIM_CENTER_Y, 111);
    /* An overlong or interrupted command must never apply its trailing text. */
    for (i = 0; i < 80; ++i) command("A");
    command("SET KPX 1;SET KPX 0.4;");
    CHECK(tune.kp_x == 0.4f && command_result == 1, 112);
    command("SET KPX ");
    rx_fault = 1;
    Tune_Poll();
    command("1;SET KPX 0.5;");
    CHECK(tune.kp_x == 0.5f && command_result == 1, 113);

    command("RESET;");
    reset_test();
    feed_frame(1005, -988);
    ennmotor();
    for (i = 0; i < 100; ++i) { tick(20); feed_frame(1005, -988); ennmotor(); }
    CHECK(x_command == 15 && y_command == -10, 114);
    command("SET MAXX 4;SET MAXY 3;");
    tick(20); feed_frame(1005, -988); ennmotor();
    CHECK(x_command <= 4 && y_command >= -3, 115);
    command("SET KPX 0;SET KPY 0;");
    for (i = 0; i < 100; ++i) { tick(20); feed_frame(1005, -988); ennmotor(); }
    CHECK(x_command == 0 && y_command == 0, 116);
    command("RESET;");
    tick(20); feed_frame(1005, -988); ennmotor();
    tick(UART5_RX_TIMEOUT_MS); ennmotor();
    CHECK(x_command == 0 && y_command == 0 && !aim_active, 117);
    feed_frame(1005, -988); ennmotor();
    main_mode = 1; ennmotor();
    CHECK(x_command == 0 && y_command == 0, 118);

    /* RX/TX interrupt registers are mapped by the emulator. */
    Tune_Plot(50, 15, 22, 3, -2, 1, 1);
    CHECK(tx_len > 0 && tx_len < TX_SIZE, 119);
    CHECK(strncmp(tx, "5.00,15,12.00,22,10.00,10.00,3,-2,", 32) == 0, 120);
    commas = 0;
    for (i = 0; i < tx_len; ++i) if (tx[i] == ',') ++commas;
    CHECK(commas == 13 && tx[tx_len - 2] == '\r' && tx[tx_len - 1] == '\n', 121);
    seq = tx_len;
    Tune_Plot(100, 0, 0, 0, 0, 0, 0);
    CHECK(tx_len == seq, 122); /* Busy TX frame is not overwritten. */
    USART1->CR1 |= USART_CR1_TXEIE;
    USART1->SR = USART_SR_TXE;
    for (i = 0; i <= (int)seq; ++i) Tune_IRQHandler();
    CHECK(tx_len == 0 && !(USART1->CR1 & USART_CR1_TXEIE), 123);
    USART1->SR = USART_SR_RXNE;
    USART1->DR = ';'; Tune_IRQHandler(); Tune_Poll();
    USART1->SR = USART_SR_ORE;
    Tune_IRQHandler();
    CHECK(rx_fault, 124);

    /* Optional shaping defaults off; command validation never enables it by accident. */
    rx_fault = 0;
    command("RESET;SET KPNX 0.40;SET KDX 0.02;SET SHAPEX 1;");
    CHECK(tune.shape_x && tune.near_kp_x == 0.4f && tune.kd_x == 0.02f, 201);
    command("SET KDX 0.2;SET SHAPEX 2;");
    CHECK(command_result == -1 && tune.kd_x == 0.02f && tune.shape_x == 1, 202);
    command("RESET;");
    CHECK(tune.shape_x == AIM_SHAPE_X && tune.kd_x == AIM_NEAR_KD_X, 203);

    /* Equal gains and zero damping reproduce the existing controller exactly. */
    AimAxis_Reset(&normal_axis); AimAxis_Reset(&shaped_axis); AimDamping_Reset(&damping);
    for (i = 0; i < 100; ++i)
    {
        float error = i < 40 ? 35.0f : (i < 75 ? -10.0f : 0.0f);
        old_output = AimAxis_Update(&normal_axis, error, 0.4f, 15, 40, .02f);
        shaped_output = AimAxis_UpdateShaped(&shaped_axis, &damping, error,
                            0.4f, 0.4f, 0, 15, 40, .02f);
        CHECK(old_output == shaped_output, 204);
    }
    AimAxis_Reset(&normal_axis); AimAxis_Reset(&shaped_axis); AimDamping_Reset(&damping);
    old_output = AimAxis_Update(&normal_axis, 5, .5f, 15, 10000, .02f);
    shaped_output = AimAxis_UpdateShaped(&shaped_axis, &damping, 5, .2f, .5f, 0, 15, 10000, .02f);
    CHECK(shaped_output == 1.0f && old_output == 2.5f, 205);
    CHECK(damping.filtered_rate == 0.0f, 206); /* No first-frame derivative kick. */
    AimAxis_Reset(&normal_axis); AimAxis_Reset(&shaped_axis); AimDamping_Reset(&damping);
    old_output = AimAxis_Update(&normal_axis, 30, .5f, 15, 10000, .02f);
    shaped_output = AimAxis_UpdateShaped(&shaped_axis, &damping, 30, .2f, .5f, 0, 15, 10000, .02f);
    CHECK(old_output == shaped_output && shaped_output == 15.0f, 207);

    /* Falling error brakes earlier; damping alone cannot request an early reversal. */
    AimAxis_Reset(&normal_axis); AimAxis_Reset(&shaped_axis); AimDamping_Reset(&damping);
    AimAxis_UpdateShaped(&shaped_axis, &damping, 12, .4f, .5f, .02f, 15, 10000, .02f);
    old_output = AimAxis_Update(&normal_axis, 8, .4f, 15, 10000, .02f);
    shaped_output = AimAxis_UpdateShaped(&shaped_axis, &damping, 8, .4f, .5f, .02f, 15, 10000, .02f);
    CHECK(shaped_output >= 0 && shaped_output < old_output && damping.filtered_rate < 0, 208);
    shaped_output = AimAxis_UpdateShaped(&shaped_axis, &damping, 4, .4f, .5f, .1f, 15, 10000, .02f);
    CHECK(shaped_output == 0, 209);

    /* Real main-loop path: center changes and camera loss clear derivative history. */
    command("RESET;SET KPX 0.52;SET KPNX 0.40;SET KDX 0.02;SET SHAPEX 1;");
    reset_test();
    feed_frame(30, 12); ennmotor();
    tick(20); feed_frame(20, 12); ennmotor();
    CHECK(aim_damping_x.have_error && aim_damping_x.filtered_rate != 0, 210);
    command("SET CX 10;");
    tick(20); feed_frame(20, 12); ennmotor();
    CHECK(aim_damping_x.have_error && aim_damping_x.filtered_rate == 0, 211);
    tick(UART5_RX_TIMEOUT_MS); ennmotor();
    CHECK(!aim_damping_x.have_error && x_command == 0 && y_command == 0, 212);
    feed_frame(30, 12); ennmotor();
    CHECK(aim_damping_x.have_error && aim_damping_x.filtered_rate == 0, 213);
    main_mode = 1; ennmotor();
    CHECK(!aim_damping_x.have_error && x_command == 0 && y_command == 0, 214);
    command("RESET;");

    /* Filter length can change online without mixing histories or losing RX bytes. */
    reset_test();
    command("SET AVG 1;");
    CHECK(command_result == 1 && Uart5_GetAverageFrames() == 1, 301);
    feed_frame(10, -10); Uart5_ProcessData();
    CHECK(target_x == 10 && target_y == -10, 302);
    feed_frame(30, -30); Uart5_ProcessData();
    CHECK(target_x == 30 && target_y == -30, 303);
    command("SET AVG 2;");
    feed_frame(10, -10); Uart5_ProcessData();
    CHECK(target_x == 10 && target_y == -10, 304);
    feed_frame(30, -30); Uart5_ProcessData();
    CHECK(target_x == 20 && target_y == -20, 305);
    feed_frame(50, -50); Uart5_ProcessData();
    CHECK(target_x == 40 && target_y == -40, 306);
    command("SET AVG 3;");
    feed_frame(10, -10); feed_frame(30, -30); feed_frame(50, -50); Uart5_ProcessData();
    CHECK(target_x == 30 && target_y == -30, 307);
    command("SET AVG 0;SET AVG 4;SET AVG 1.5;");
    CHECK(command_result == -1 && Uart5_GetAverageFrames() == 3, 308);
    Uart5_ReceiveByte(0x7B); Uart5_ReceiveByte(0x80);
    command("SET AVG 1;");
    Uart5_ReceiveByte(0x0A); Uart5_ReceiveByte(0x80); Uart5_ReceiveByte(0x14);
    Uart5_ReceiveByte(0); Uart5_ReceiveByte(0); Uart5_ReceiveByte(0x7D);
    Uart5_ProcessData();
    CHECK(target_x == 10 && target_y == 20, 309);
    seq = received_frame_id;
    command("SET AVG 2;");
    CHECK(received_frame_id == seq && rv_data_ready == 0, 310);
    tick(UART5_RX_TIMEOUT_MS); Uart5_ProcessData();
    CHECK(!published_sample.valid && target_x == 0 && target_y == 0, 311);
    command("RESET;");
    CHECK(Uart5_GetAverageFrames() == UART5_DEFAULT_AVERAGE_FRAMES, 312);

    /* Independent PD commands and exact zero-D legacy behavior. */
    command("RESET;SET KDX 0.02;SET KDY 0.015;");
    CHECK(!tune.shape_x && tune.kd_x == .02f && tune.kd_y == .015f, 501);
    command("SET KDY -1;SET KDY nan;SET KDY 0.11;");
    CHECK(command_result == -1 && tune.kd_y == .015f, 502);
    AimAxis_Reset(&normal_axis); AimAxis_Reset(&shaped_axis); AimDamping_Reset(&damping);
    for (i = 0; i < 100; ++i)
    {
        float error = i < 40 ? 35.0f : (i < 75 ? -10.0f : 0.0f);
        old_output = AimAxis_Update(&normal_axis, error, .55f, 15, 260, .02f);
        shaped_output = AimAxis_UpdatePD(&shaped_axis, &damping, error, .55f, 0, 15, 260, .02f);
        CHECK(old_output == shaped_output && !damping.have_error, 503);
    }
    command("RESET;SET SHAPEY 0;SET AVG 1;SET KDX 0.02;SET KDY 0.02;");
    reset_test();
    feed_frame(17, 24); ennmotor();
    CHECK(aim_damping_x.filtered_rate == 0 && aim_damping_y.filtered_rate == 0, 504);
    tick(20); feed_frame(13, 20); ennmotor();
    CHECK(aim_damping_x.filtered_rate < 0 && aim_damping_y.filtered_rate < 0, 505);
    CHECK(aim_x.speed < 8 * AIM_KP_X && aim_y.speed < 8 * AIM_KP_Y, 506);
    command("SET KDX 0.03;SET KDY 0.03;");
    tick(20); feed_frame(12, 19); ennmotor();
    CHECK(aim_damping_x.filtered_rate == 0 && aim_damping_y.filtered_rate == 0, 507);
    command("SET CX 6;SET CY 13;");
    tick(20); feed_frame(11, 18); ennmotor();
    CHECK(aim_damping_x.filtered_rate == 0 && aim_damping_y.filtered_rate == 0, 508);
    tick(20); feed_frame(12, 19); ennmotor();
    CHECK(aim_damping_x.filtered_rate > 0 && aim_damping_y.filtered_rate > 0, 509);
    command("SET AVG 2;");
    tick(20); feed_frame(13, 20); ennmotor();
    CHECK(aim_damping_x.filtered_rate == 0 && aim_damping_y.filtered_rate == 0, 510);
    main_mode = 1; ennmotor();
    CHECK(!aim_damping_x.have_error && !aim_damping_y.have_error, 511);
    main_mode = 2; tick(20); feed_frame(14, 21); ennmotor();
    CHECK(aim_damping_x.filtered_rate == 0 && aim_damping_y.filtered_rate == 0, 512);
    tick(UART5_RX_TIMEOUT_MS); ennmotor();
    CHECK(!aim_damping_x.have_error && !aim_damping_y.have_error && x_command == 0 && y_command == 0, 513);
    command("RESET;");
    CHECK(tune.shape_x == AIM_SHAPE_X && tune.kd_x == AIM_NEAR_KD_X && tune.kd_y == AIM_NEAR_KD_Y, 514);
    AimAxis_Reset(&shaped_axis); AimDamping_Reset(&damping);
    AimAxis_UpdatePD(&shaped_axis, &damping, 12, .55f, .1f, 15, 10000, .02f);
    shaped_output = AimAxis_UpdatePD(&shaped_axis, &damping, 4, .55f, .1f, 15, 10000, .02f);
    CHECK(shaped_output == 0, 515);
    AimAxis_UpdatePD(&shaped_axis, &damping, 10, .55f, .02f, 15, 10000, .2f);
    CHECK(damping.filtered_rate == 0, 516);
    /* Scheduled PD: endpoint gains, continuous symmetric transition, no double boost. */
    CHECK(AimAxis_ScheduleBlend(7, 7, 24) == 0, 601);
    CHECK(AimAxis_ScheduleBlend(24, 7, 24) == 1, 602);
    CHECK(AimAxis_ScheduleBlend(15.5f, 7, 24) == .5f &&
          AimAxis_ScheduleBlend(-15.5f, 7, 24) == .5f, 603);
    CHECK(AimAxis_ScheduleBlend(6.99f, 7, 24) == 0 &&
          AimAxis_ScheduleBlend(7.01f, 7, 24) < .00001f, 604);
    AimAxis_Reset(&shaped_axis); AimDamping_Reset(&damping);
    shaped_output = AimAxis_UpdateScheduled(&shaped_axis, &damping, 6,
                        .4f, .8f, .04f, 0, 7, 24, 100, 10000, .02f);
    CHECK(shaped_output > 2.399f && shaped_output < 2.401f && damping.filtered_rate == 0, 605);
    AimAxis_Reset(&shaped_axis); AimDamping_Reset(&damping);
    shaped_output = AimAxis_UpdateScheduled(&shaped_axis, &damping, 30,
                        .4f, .8f, .04f, 0, 7, 24, 100, 10000, .02f);
    CHECK(shaped_output == 24, 606); /* Exactly .8*30, no legacy extra 35 percent. */
    shaped_output = AimAxis_UpdateScheduled(&shaped_axis, &damping, 29,
                        .4f, .8f, .04f, 0, 7, 24, 100, 10000, .02f);
    CHECK(shaped_output > 23.199f && shaped_output < 23.201f && damping.filtered_rate < 0, 607);
    AimAxis_Reset(&shaped_axis);
    damping.have_error = 1; damping.previous_error = 15.5f; damping.filtered_rate = 100;
    shaped_output = AimAxis_UpdateScheduled(&shaped_axis, &damping, 15.5f,
                        .4f, .8f, .04f, 0, 7, 24, 100, 10000, .02f);
    CHECK(shaped_output > 10.799f && shaped_output < 10.801f, 608); /* P=.6, D=.02. */
    shaped_output = AimAxis_UpdateScheduled(&shaped_axis, &damping, 4,
                        .4f, .8f, .1f, 0, 7, 24, 100, 10000, .02f);
    CHECK(shaped_output == 0, 609); /* Near damping cannot reverse the command. */
    AimAxis_UpdateScheduled(&shaped_axis, &damping, 10,
                        .4f, .8f, .04f, 0, 7, 24, 100, 10000, .2f);
    CHECK(damping.filtered_rate == 0, 610);

    command("RESET;SET KPNX 0.4;SET KPFX 0.8;SET KDNX 0.04;SET KDFX 0.001;");
    command("SET KPNY 0.2;SET KPFY 0.4;SET KDNY 0.02;SET KDFY 0.002;");
    command("SET NEARX 8;SET FARX 30;SET NEARY 9;SET FARY 31;");
    command("SET SHAPEX 1;SET SHAPEY 1;");
    CHECK(tune.shape_x && tune.shape_y && tune.near_kp_x == .4f && tune.kp_x == .8f &&
          tune.kd_x == .04f && tune.far_kd_x == .001f && tune.near_kp_y == .2f &&
          tune.kp_y == .4f && tune.kd_y == .02f && tune.far_kd_y == .002f, 611);
    command("SET NEARX 30;SET FARX 8;SET NEARY 31;SET FARY 9;");
    CHECK(command_result == -1 && tune.near_error_x == 8 && tune.far_error_x == 30 &&
          tune.near_error_y == 9 && tune.far_error_y == 31, 612);
    command("SET NEARX 3;SET KDFX -1;SET KDFY nan;SET SHAPEY 2;");
    CHECK(command_result == -1 && tune.near_error_x == 8 && tune.far_kd_x == .001f &&
          tune.far_kd_y == .002f && tune.shape_y == 1, 613);
    command("SET AVG 1;"); reset_test();
    feed_frame(35, 42); ennmotor();
    CHECK(aim_damping_x.have_error && aim_damping_y.have_error &&
          aim_damping_x.filtered_rate == 0 && aim_damping_y.filtered_rate == 0, 614);
    tick(20); feed_frame(25, 32); ennmotor();
    CHECK(aim_damping_x.filtered_rate < 0 && aim_damping_y.filtered_rate < 0, 615);
    command("SET FARX 32;SET KDFY 0.003;");
    tick(20); feed_frame(20, 27); ennmotor();
    CHECK(aim_damping_x.filtered_rate == 0 && aim_damping_y.filtered_rate == 0, 616);
    tick(20); feed_frame(15, 22); ennmotor();
    CHECK(aim_damping_x.filtered_rate < 0 && aim_damping_y.filtered_rate < 0, 617);
    command("SET SHAPEX 0;SET SHAPEY 0;");
    tick(20); feed_frame(14, 21); ennmotor();
    CHECK(aim_damping_x.filtered_rate == 0 && aim_damping_y.filtered_rate == 0, 618);
    command("SET SHAPEX 1;SET SHAPEY 1;");
    tick(20); feed_frame(13, 20); ennmotor();
    CHECK(aim_damping_x.filtered_rate == 0 && aim_damping_y.filtered_rate == 0, 619);
    tick(UART5_RX_TIMEOUT_MS); ennmotor();
    CHECK(!aim_damping_x.have_error && !aim_damping_y.have_error && x_command == 0 && y_command == 0, 620);
    command("RESET;");
    CHECK(tune.shape_x == AIM_SHAPE_X && tune.shape_y == AIM_SHAPE_Y &&
          tune.kd_x == AIM_NEAR_KD_X && tune.kd_y == AIM_NEAR_KD_Y &&
          tune.near_kp_x == AIM_NEAR_KP_X && tune.near_kp_y == AIM_NEAR_KP_Y &&
          tune.far_kd_x == AIM_FAR_KD_X && tune.far_kd_y == AIM_FAR_KD_Y &&
          tune.near_error_x == 7 && tune.far_error_y == 24, 621);
    /* RESET restores persisted gains/modes after online tuning; main uses Y PD. */
    command("SET SHAPEY 0;SET KPY 0.8;SET KDY 0;SET KPX 0.9;SET KDX 0;RESET;");
    CHECK(tune.kp_x == .58f && tune.kd_x == .010f && !tune.shape_x &&
          tune.shape_y && tune.near_kp_y == .18f && tune.kp_y == .24f &&
          tune.kd_y == .003f && tune.far_kd_y == .0005f, 704);
    reset_test(); command("SET AVG 1;");
    feed_frame(5, 24); ennmotor();
    CHECK(aim_damping_y.have_error && aim_damping_y.filtered_rate == 0, 705);
    tick(20); feed_frame(5, 20); ennmotor();
    CHECK(aim_damping_y.filtered_rate < 0 && y_command >= 0 && y_command < 2, 706);
    tick(UART5_RX_TIMEOUT_MS); ennmotor();
    CHECK(!aim_damping_y.have_error && y_command == 0, 707);
    return 0;
}
