/*
							 _ooOoo_
							o8888888o
							88" . "88
							(| -_- |)
							O\  =  /O
					 ____/`---'\____
				 .'  \\|     |//  `.
				/  \\|||  :  |||//  \
		   /  _||||| -:- |||||-  \
		   |   | \\\  -  /// |   |
		   | \_|  ''\---/''  |   |
		   \  .-\__  `-`  ___/-. /
		 ___`. .'  /--.--\  `. . __
	  ."" '<  `.___\_<|>_/___.'  >'"".
	 | | :  `- \`.;`\ _ /`;.`/ - ` : | |
	 \  \ `-.   \_ __\ /__ _/   .-` /  /
======`-.____`-.___\_____/___.-`____.-'======
				   `=---='
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		 佛祖保佑       永无BUG
*/
#include "stm32f4xx.h"
#include "sys.h"
#include "board.h"
#include "delay.h"
#include "usart.h"
#include "Emm_V5.h"
#include "UART_maxcam.h"
#include "aim_control.h"
#include "serial_tune.h"

int Right_Speed = 0;
int Left_Speed = 0;
// 用来保存串口收到的数据
uint8_t Serial_RxData;
// 标记串口接收完成标志位
uint8_t Serial_RxFlag;
uint8_t buffer[9];     // 接收缓冲区
uint8_t RxData_Cnt;    // 接收计数
int Yaw;
uint8_t need_motor_update = 0;  // 电机更新标志
char str[30] = "";
uint8_t com_data;
uint8_t RxBuffer1[4]={0};
uint8_t RxCounter1=0;
uint8_t RxState = 0;
uint8_t Header1=0,Header2=0,x=0,frame=0;
uint8_t cx,cy;
uint16_t cnttt;
uint16_t cnttt1,cnttt2;
double distance;
double distance2=0;
double distance3=0;
int Left_Speed_cnt = 0;
int Right_Speed_cnt = 0;
int xmode1,ymode1,zmode1,amode1,cmode1=0;
int a,b,c,d,e,f=0;
float goal_smooth=0;
float last_goal_smooth=0;
float ennagle=0;
float last_ennagle=0;
float goal_ennagle=0;
float rad=0;
float yennagle=0;
float ylast_ennagle=0;
float ygoal_ennagle=0;
float yrad=0;
float smooth_ennagle_x,smooth_ennagle_y,pulse_target,pulse_target_y=0;
extern uint8_t main_mode,mode2,mode1,flag,flag_z,number;
float ennagle_x,ennagle_y,last_ennagle_x,last_ennagle_y,add_x=0;

/* 图像误差 -> 限幅带符号电机转速，每帧相机图像更新一次 */
static AimAxis aim_x;
static AimAxis aim_y;
static AimDamping aim_damping_x;
static AimDamping aim_damping_y;
static TuneParams aim_previous_tune;
static uint8_t aim_previous_average;
static uint8_t aim_active = 0;
static int16_t aim_cmd_x, aim_cmd_y;
static uint8_t aim_have_frame = 0;
static uint8_t aim_stop_sent = 0;
static uint32_t aim_last_frame_id = 0;
static uint32_t aim_last_frame_ms = 0;

static void Aim_SendSpeed(int16_t x_rpm, int16_t y_rpm)
{
    uint8_t x_dir = x_rpm < 0 ? (AIM_X_POSITIVE_DIR ^ 1U) : AIM_X_POSITIVE_DIR;
    uint8_t y_dir = y_rpm < 0 ? (AIM_Y_POSITIVE_DIR ^ 1U) : AIM_Y_POSITIVE_DIR;
    uint16_t x_speed = (uint16_t)(x_rpm < 0 ? -x_rpm : x_rpm);
    uint16_t y_speed = (uint16_t)(y_rpm < 0 ? -y_rpm : y_rpm);

    aim_cmd_x = x_rpm;
    aim_cmd_y = y_rpm;
    Emm_V5_Vel_Control(1, x_dir, x_speed, (uint8_t)tune.acc_x, 0);
    Emm_V5_Vel_Control3(2, y_dir, y_speed, (uint8_t)tune.acc_y, 0);
}

void ennmotor(void)
{
    Uart5Sample sample;
    uint32_t elapsed_ms;
    float dt;
    float x_speed;
    float y_speed;

    Uart5_ReadSample(&sample);
    if (main_mode != 2 || !sample.valid || sample.zero_timeout)
    {
        /* 超时或者持续收到(0,0)：跳过标定，直接发送0转速 */
        if (!aim_stop_sent)
        {
            Aim_SendSpeed(0, 0);
            aim_stop_sent = 1;
        }
        AimAxis_Reset(&aim_x);
        AimAxis_Reset(&aim_y);
        AimDamping_Reset(&aim_damping_x);
        AimDamping_Reset(&aim_damping_y);
        aim_active = 0;
        aim_have_frame = 0;
        return;
    }

    if (!aim_active)
    {
        AimAxis_Reset(&aim_x);
        AimAxis_Reset(&aim_y);
        AimDamping_Reset(&aim_damping_x);
        AimDamping_Reset(&aim_damping_y);
    }
    aim_stop_sent = 0;
    aim_active = 1;
    if (aim_have_frame && sample.frame_id == aim_last_frame_id)
        return;

    elapsed_ms = aim_have_frame ? sample.timestamp_ms - aim_last_frame_ms : 20U;
    if (elapsed_ms >= UART5_RX_TIMEOUT_MS)
    {
        AimAxis_Reset(&aim_x);
        AimAxis_Reset(&aim_y);
        AimDamping_Reset(&aim_damping_x);
        AimDamping_Reset(&aim_damping_y);
        elapsed_ms = 20U;
    }
    dt = (float)elapsed_ms * 0.001f;
    aim_last_frame_id = sample.frame_id;
    aim_last_frame_ms = sample.timestamp_ms;
    aim_have_frame = 1;

    if (aim_previous_tune.shape_x != tune.shape_x || aim_previous_tune.center_x != tune.center_x ||
        aim_previous_tune.kd_x != tune.kd_x || aim_previous_tune.far_kd_x != tune.far_kd_x ||
        aim_previous_tune.near_error_x != tune.near_error_x ||
        aim_previous_tune.far_error_x != tune.far_error_x ||
        aim_previous_average != Uart5_GetAverageFrames())
        AimDamping_Reset(&aim_damping_x);
    if (aim_previous_tune.shape_y != tune.shape_y || aim_previous_tune.center_y != tune.center_y ||
        aim_previous_tune.kd_y != tune.kd_y || aim_previous_tune.far_kd_y != tune.far_kd_y ||
        aim_previous_tune.near_error_y != tune.near_error_y ||
        aim_previous_tune.far_error_y != tune.far_error_y ||
        aim_previous_average != Uart5_GetAverageFrames())
        AimDamping_Reset(&aim_damping_y);
    aim_previous_tune = tune;
    aim_previous_average = Uart5_GetAverageFrames();
    if (tune.shape_x)
        x_speed = AimAxis_UpdateScheduled(&aim_x, &aim_damping_x,
                    (float)sample.x - tune.center_x, tune.near_kp_x, tune.kp_x,
                    tune.kd_x, tune.far_kd_x, tune.near_error_x, tune.far_error_x,
                    tune.max_x, tune.slew_x, dt);
    else
        x_speed = AimAxis_UpdatePD(&aim_x, &aim_damping_x,
                    (float)sample.x - tune.center_x, tune.kp_x, tune.kd_x,
                    tune.max_x, tune.slew_x, dt);
    if (tune.shape_y)
        y_speed = AimAxis_UpdateScheduled(&aim_y, &aim_damping_y,
                    (float)sample.y - tune.center_y, tune.near_kp_y, tune.kp_y,
                    tune.kd_y, tune.far_kd_y, tune.near_error_y, tune.far_error_y,
                    tune.max_y, tune.slew_y, dt);
    else
        y_speed = AimAxis_UpdatePD(&aim_y, &aim_damping_y,
                    (float)sample.y - tune.center_y, tune.kp_y, tune.kd_y,
                    tune.max_y, tune.slew_y, dt);
    /* 在线限幅，同时限制上一帧残留的转速 */
    if (aim_x.speed > tune.max_x) aim_x.speed = tune.max_x;
    if (aim_x.speed < -tune.max_x) aim_x.speed = -tune.max_x;
    if (aim_y.speed > tune.max_y) aim_y.speed = tune.max_y;
    if (aim_y.speed < -tune.max_y) aim_y.speed = -tune.max_y;
    x_speed = aim_x.speed;
    y_speed = aim_y.speed;
    Aim_SendSpeed(AimAxis_Rpm(x_speed), AimAxis_Rpm(y_speed));
}

int main(void)
{
    uint32_t last_oled_ms = 0;
	// 初始化OLED显示

	OLED_Init();
	Key_Init();
	EXTIXa_Init();
	// 初始化定时器4
	TIMer4_Init();
	TIMer2_Init();
	TIMer6_Init();
	// 初始化电机接口
	Motor_Init();
	// 初始化电机PWM，输出频率10KHZ
	Motor_PWM_Init();
	// 初始化左编码器
	Left_Encoder_Init();
	// 初始化右编码器
	Right_Encoder_Init();
	// 初始化串口1
	uart1_init();
	uart2_init();
	uart3_init();
	LED_Init();
	Uart5_en_Init();
	board_init();
	delay_ms(500);

    while (1)
    {
        uint32_t now;

        /* 先执行控制，再刷新显示；退出模式、相机失联时会停止瞄准 */
        Tune_Poll();
        ennmotor();
        now = Uart5_GetTimeMs();
        {
            Uart5Sample plot_sample;
            Uart5_ReadSample(&plot_sample);
            Tune_Plot(now, plot_sample.x, plot_sample.y, aim_cmd_x, aim_cmd_y,
                      plot_sample.valid && !plot_sample.zero_timeout, aim_active);
        }
        if ((uint32_t)(now - last_oled_ms) >= AIM_OLED_PERIOD_MS)
        {
            last_oled_ms = now;
            sprintf(str, "X:%6d", (int)Uart5_getdata(1));
            OLED_ShowString(0, 0, (uint8_t *)str, 16);
            sprintf(str, "Y:%6d", (int)Uart5_getdata(2));
            OLED_ShowString(0, 16, (uint8_t *)str, 16);
            sprintf(str, "Mode:%3u   ", (unsigned int)main_mode);
            OLED_ShowString(0, 32, (uint8_t *)str, 16);
            sprintf(str, "D:%3u      ", (unsigned int)number);
            OLED_ShowString(0, 48, (uint8_t *)str, 16);
            OLED_Refresh();
        }
    }
}

void TIM2_IRQHandler(void)//TIM2中断服务函数
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==1)
	{
	if(main_mode==1)
	{



    }

	if(main_mode==2)
	{

	}

	if(main_mode==3)
	{
        KUAIDIYITI2();
	}

}


	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);//清除中断标志位
}

void TIM6_DAC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update)==1)
	{
        Uart5_Tick1ms();

	}
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
}


void TIM4_IRQHandler(void)//TIM4中断服务函数
{
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==1)
	{
	    Left_Speed =(short)TIM_GetCounter(TIM5);  // 读取左编码器计数值；
	    Right_Speed =(short)TIM_GetCounter(TIM3);  // 读取右编码器计数值；
	    Left_Speed_cnt+=Left_Speed;
	    Right_Speed_cnt+=Right_Speed;
	    distance=Count_Distance((Left_Speed_cnt+Right_Speed_cnt)/2);
		if(Left_Speed!=0||Right_Speed!=0)
		{
			TIM_SetCounter(TIM5,0);
			TIM_SetCounter(TIM3,0);
		}
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);//清除中断标志位
}


void USART1_IRQHandler(void)
{
    Tune_IRQHandler();
}

//void ennmotor(void)
//{
////    Uart5_ProcessData();
////		int16_t x_target = Uart5_getdata(1);
////		int16_t y_target = Uart5_getdata(2);
////    rad= atan2(40, x_target);
////	  ennagle=rad * 180.0f / PI;
////	if(ennagle>0)
////	{
////		Emm_V5_Pos_Control(1,0,500,0,ennagle,1,0);
////	}
////   if(ennagle<0)
////	 {
////	 		Emm_V5_Pos_Control(1,0,500,0,-ennagle,1,0);

////	 }
//Emm_V5_Pos_Control(1,0,500,0,Yaw,1,0);

//}


//void USART3_IRQHandler(void)
//{
//
//	 if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
//    {
//        com_data = USART_ReceiveData(USART3);
//
//        // 判断收到的数据是否为ASCII数字：'0'-'9'
//        if(com_data >= '0' && com_data <= '9')
//        {
//            // ASCII转数字
//            x = com_data - '0';
//
//            // 根据数字切换对应模式
//            if(x >= 1 && x <= 8)
//            {
//                main_mode = 1;    // 自动模式
//                mode1 = 0;        // 手动模式
//                distance = 0;     // 距离清零
//                Left_Speed_cnt = 0;
//                Right_Speed_cnt = 0;
//
//                // 闪烁LED提示模式切换
//                LED0 = !LED0;
//                BEEP = 1;
//                delay_ms(100);
//                BEEP = 0;
//            }
//            else
//            {
//                // 无效模式值
//                main_mode = 0;
//            }
//        }
//
//        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
//    }
//}
//
//
//
//	// 判断USART3的接收中断是否触发
//	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
//	{
//		// 读取USART3数据
//		com_data = USART_ReceiveData(USART3);
//
//		// 根据收到的数据和当前接收状态RxState进行帧解析
//		if(RxState == 0 && com_data == 0xA5)  // 当前状态0，收到帧头0xA5
//		{
//			RxState = 1;                     // 切换状态为1
//			RxBuffer1[RxCounter1++] = com_data; // 将收到数据存入缓冲区并计数
//		}
//		else if(RxState == 1 && com_data == 0xA6)  // 当前状态1，收到第二个帧头0xA6
//		{
//			RxState = 2;                     // 切换状态为2
//			RxBuffer1[RxCounter1++] = com_data; // 将收到数据存入缓冲区并计数
//		}
//		else if(RxState == 2)  // 当前状态2
//		{
//			RxBuffer1[RxCounter1++] = com_data; // 将收到数据存入缓冲区并计数
//			if(com_data == 0x5B)  // 收到结束符0x5B
//			{
//				RxState = 3;                 // 切换状态为3
//			}
//		}
//
//		// 收到结束符0x5B且当前状态为3
//		if(com_data == 0x5B && RxState == 3)
//		{
//			// 读取前6字节数据赋值
//			Header1 = RxBuffer1[0];
//			Header2 = RxBuffer1[1];
//			x = RxBuffer1[2];
//			frame = RxBuffer1[3];
//			// 复位接收状态和计数
//			RxState = 0;
//			RxCounter1 = 0;
//		}
//
//		// 清除USART3接收中断标志位
//		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
//	}
