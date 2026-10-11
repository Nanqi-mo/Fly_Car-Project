#include "sys.h"

//更新遥控器数据，并在0核使用
static void multicore_core0_update_inputs(void)
{
    mc_rc_frame_t rc_frame;

    if (multicore_read_rc(&rc_frame))
    {
		RC_data.rc_roll = rc_frame.rc_roll;
		RC_data.rc_pitch = rc_frame.rc_pitch;
		RC_data.rc_height = rc_frame.rc_height;
		RC_data.rc_servo = rc_frame.rc_servo;
		RC_data.mode = rc_frame.mode;
		RC_data.change = rc_frame.change;
		RC_data.update_flag = rc_frame.update_flag;
        RC_data.online = rc_frame.online;
    }
}

//周期中断处理函数
void pit_hanlder(uint32 state, void *ptr)
{
	(void)state;
	(void)ptr;

    static uint8 Time_5ms = 0;
    static uint8 Time_10ms = 0;
    static uint8 Time_20ms = 0;

    IMU_Solving();                  //四元数姿态解算
    RTK_Data();                     //GPS数据解析 + 同步CORE2解算结果
    multicore_core0_update_inputs();//接收遥控器数据

    Time_5ms++;
    Time_10ms++;
    Time_20ms++;

    PID_flag.Rate = 1;        //角速度环计算
    PID_flag.Turn = 1;        //转向环计算

    if (Time_5ms >= 5)
    {
        Time_5ms = 0;
        PID_flag.Angle = 1;   //角度环计算
        Get_YAW();            //解算偏航角

    }

    if (Time_10ms >= 10)
    {
        Time_10ms = 0;
        PID_flag.Position = 1;//位置环计算
	
    }

    if (Time_20ms >= 20)
    {
        Time_20ms = 0;
        PID_flag.Position = 1;//位置环计算
    	if(!Control_flag.Plane_Start || Control_flag.Plane_Break)
		{
    		 Key_scan();     //按键扫描
		}
    }
}

void pit_time_init(void)
{

    zf_pit_ms_init(PIT, 1, pit_hanlder, NULL);        // PIT 初始化 1ms 中断一次 中断处理函数为 pit_handler1


}
