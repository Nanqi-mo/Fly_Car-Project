#include "sys.h"
#include <math.h>

/*********************** Plane ***********************/
//位置环PID
PID_TYPE PID_ROL_Position;  //ROLL
PID_TYPE PID_PIT_Position;  //PITCH
//角度环PID
PID_TYPE PID_ROL_Angle;     //ROLL
PID_TYPE PID_PIT_Angle;     //PITCH
PID_TYPE PID_YAW_Angle;     //YAW
//角速度环PID
PID_TYPE PID_ROL_Rate;      //ROLL
PID_TYPE PID_PIT_Rate;      //PITCH
PID_TYPE PID_YAW_Rate;      //YAW
//高度环PID
PID_TYPE PID_HEI_Rate;      //高度速度环
PID_TYPE PID_HEI;           //高度环

/*********************** Car ***********************/
PID_TYPE PID_Left_Motor;    //左电机速度环
PID_TYPE PID_Right_Motor;   //右电机速度环
PID_TYPE PID_Error;         //误差环--直线时
PID_TYPE PID_Turn;          //转向环--转弯时

/*********************** Angle ***********************/
//定义角度，估计角度+目标角度+设置角度
FLOAT_ANGLE Measure_Angle;
FLOAT_ANGLE Target_Angle;
FLOAT_ANGLE Set_Angle;

/*********************** Flag ***********************/
//标志位
Control_Flag  Control_flag;
PID_Flag      PID_flag;

/*********************** Control ***********************/
Car_Control   Car_control;
Plane_Control Plane_control;
float PWM_HOVER = 0;        //基础油门
float HEI_HOVER = 0;        //缓冲高度
float Set_Step  = 0;        //设定步长

/*********************** Motor ***********************/
float Moto_PWM_1=0.0f,
      Moto_PWM_2=0.0f,
      Moto_PWM_3=0.0f,
      Moto_PWM_4=0.0f;
int   Moto_Left_PWM  = 0,   //左电机PWM
	  Moto_Right_PWM = 0;   //右电机PWM


//模式控制函数
void model_control(void)
{
    uint8_t key_clear = switch_flag ;

    if (key_clear)
    {
        ips200_clear();

    }

    if(switch_flag == 1)
    {
        key1_flag = 0;
        key2_flag = 0;
        key3_flag = 0;
        key4_flag = 0;
    }

   //修改菜单步长
   if(switch_set2 == step1)//步长1
   {
       Set_Step = 1;

   }
   else if(switch_set2 == step2)//步长0.1
   {
       Set_Step = 0.1;

   }
   else if(switch_set2 == step3)//步长0.01
   {
       Set_Step = 0.01;

   }
   else if(switch_set2 == step4)//步长0.001
   {
       Set_Step = 0.001;

   }

    ///切换显示逻辑
    if(switch_set1 == go)//起飞
    {

    }
    else//其余模式下禁止四旋翼起飞
    {
    	Control_flag.Plane_Start  = 0;
    	Control_flag.Plane_Break  = 1;
        Control_flag.Plane_Buffer = 0;
        HEI_HOVER = 0;
        PWM_HOVER = 0;

    }

    if(switch_set1 == menu_state)//显示菜单
    {
        IPS200_show_float(16,10,Set_Step     ,1,4);

        key_Menu_event();
        Show_Cur_Menu(P_Menu);
    }

    if(switch_set1 == show)//显示函数
    {
    	IPS200_show_string(0,10,"Start_Show");

		IPS200_show_float(16,0,RC_data.mode, 4,2);
        IPS200_show_float(16,1,RC_data.change, 4,2);
        IPS200_show_float(16,2,RC_data.rc_roll, 4,2);
        IPS200_show_float(16,3,RC_data.rc_pitch, 4,2);
        IPS200_show_float(16,4,RC_data.rc_height, 4,2);
        IPS200_show_float(16,5,RC_data.rc_servo, 4,2);

        IPS200_show_float(16,6,IMU_Data.roll, 2,2);
        IPS200_show_float(16,7,IMU_Data.pitch,2,2);
        IPS200_show_float(16,8,IMU_Data.yaw,  4,2);

    }

    if(switch_set1 == rtk)//GPS打点
    {
    	Control_flag.Plane_Start = 0;
    	Control_flag.Plane_Break = 1;

        //GPS打点+存点+去偏置
        Plan_marking();

    	//显示当前XY+距离+方位角+点位
        IPS200_show_string(0,0,"cur X:");
        IPS200_show_string(0,1,"cur Y:");
        IPS200_show_string(0,2,"dist:");
        IPS200_show_string(0,3,"azi:");
        IPS200_show_string(0,4,"yaw:");
        IPS200_show_string(0,5,"num:");
        IPS200_show_string(0,6,"state:");
        IPS200_show_string(0,7,"lon:");
        IPS200_show_string(0,8,"lat:");

		IPS200_show_float(6,0,RTK_data.x_current, 3,2);
		IPS200_show_float(6,1,RTK_data.y_current, 3,2);
		IPS200_show_float(6,2,RTK_data.distance, 4,2);
		IPS200_show_float(6,3,RTK_data.azimuth , 4,1);
		IPS200_show_int(6,4,-IMU_Data.yaw / 10.0f, 3);
        IPS200_show_float(6,5,RTK_data.target_index, 4,1);
        IPS200_show_int(6,6,gnss.state, 1);
        IPS200_show_float(6,7,gnss.longitude, 2,6);
        IPS200_show_float(6,8,gnss.latitude, 2,6);
        
		//显示目标点XY及类型(1=飞机 0=车)
		for(uint8_t i = 0; i < 9; i++)
		{
			IPS200_show_float(0, i + 9, RTK_data.x_target[i], 3,1);
			IPS200_show_float(8, i + 9, RTK_data.y_target[i], 3,1);
			IPS200_show_int(16, i + 9, RTK_data.switchpoint[i], 1);
		}

    }

}


/******************************************************************************************
*函  数：void Control_Plane(IMU_Data_Def *att_angle,FLOAT_ANGLE *target_angle)
*功  能：姿态控制,角度环控制和角速度环控制
*参  数：att_in：测量值
*       gry_in: IMU660RA读取的角速度值

*       armed记录命令
*返回值：无
*备  注：RoboFly 四轴机头与电机示意图
           机头(Y+)

          M1    ↑    M2
          \   |   /
           \  |  /
            \ | /
          ————————+————————>X+
            / | \
           /  |  \
          /   |   \
          M4    |    M3


  1. M1 M3电机逆时针旋转，M2 M4电机顺时针旋转
  2. X:是IMU660RA的 X 轴，Y:是IMU660RA的 Y 轴，Z轴正方向垂直 X-Y 面，竖直向上
  3. 绕 X 轴旋转为PITCH 角
     绕 Y 轴旋转为 ROLL 角
     绕 Z 轴旋转为 YAW  角
  4. 自己DIY时进行动力分配可以一个轴一个轴的分配，切勿三个轴同时分配。
*******************************************************************************************/
//飞机计算控制函数
void Control_Plane(IMU_Data_Def *att_angle,FLOAT_ANGLE *target_angle)
{
  Measure_Angle.roll  =  att_angle->roll;       // 横滚实际角度（度）
  Measure_Angle.pitch =  att_angle->pitch;      // 俯仰实际角度（度）
  Measure_Angle.yaw   =  att_angle->yaw;        // 偏航实际角度（度）
  Target_Angle.roll   =  target_angle->roll;    // 横滚目标角度（度）
  Target_Angle.pitch  =  target_angle->pitch;   // 俯仰目标角度（度）
  Target_Angle.yaw    =  target_angle->yaw;     // 偏航目标角度（度）

  //-------------------------姿态限幅保护（超过±60°自动上锁，避免坠毁）-------------------------
  if (func_abs(Measure_Angle.roll) > ATT_LIMIT || func_abs(Measure_Angle.pitch) > ATT_LIMIT)
  {
      Moto_PWM_1 = 0;
      Moto_PWM_2 = 0;
      Moto_PWM_3 = 0;
      Moto_PWM_4 = 0;
      HEI_HOVER  = 0;
      PWM_HOVER  = 0;

      //重置PID积分（避免再次解锁时积分饱和）
      PID_Init();

      Control_flag.Plane_Start  = 0;
      Control_flag.Plane_Buffer = 0;
      Control_flag.Plane_Break  = 1;

      BLDC_PWM(0, 0, 0, 0);
      return;

  }

  //-------------------------PID计算-------------------------
  //位置环
  if(PID_flag.Position)
  {
	  PID_flag.Position = 0;
 	  PID_Position_Cal(&PID_ROL_Position, 0.8,  RTK_data.distance); // ROLL位置环PID

  }

  //角度环
  if(PID_flag.Angle)
  {
	  PID_flag.Angle = 0;
       if(!RC_data.mode)//手动挡
       {
            PID_Attitude_Cal(&PID_ROL_Angle,Target_Angle.roll  - RC_data.rc_roll,Measure_Angle.roll);   // ROLL角度环PID
            PID_Attitude_Cal(&PID_PIT_Angle,Target_Angle.pitch + RC_data.rc_pitch,Measure_Angle.pitch); // PITCH角度环PID
            PID_Attitude_Cal(&PID_YAW_Angle,Target_Angle.yaw,Measure_Angle.yaw);                        // YAW角度环PID

       }
       else//自动档
       {
            PID_Attitude_Cal(&PID_ROL_Angle,PID_ROL_Position.OutPut + Target_Angle.roll  - RC_data.rc_roll,Measure_Angle.roll); // ROLL角度环PID
            PID_Attitude_Cal(&PID_PIT_Angle,Target_Angle.pitch + Target_Angle.pitch + RC_data.rc_pitch    ,Measure_Angle.pitch);// PITCH角度环PID
            //PID_Attitude_Cal(&PID_YAW_Angle,RTK_data.azimuth,Measure_Angle.yaw); 
            PID_Attitude_Cal(&PID_YAW_Angle,Target_Angle.yaw,Measure_Angle.yaw);                                                 // YAW角度环PID

       }

  }

  //角速度环
  if(PID_flag.Rate)
  {
      PID_flag.Rate = 0;
      PID_Attitude_Cal(&PID_ROL_Rate,PID_ROL_Angle.OutPut,(float)GYRO_Real_x);// ROLL角速度环PID
      PID_Attitude_Cal(&PID_PIT_Rate,PID_PIT_Angle.OutPut,(float)GYRO_Real_y);// PITCH角速度环PID
      PID_Attitude_Cal(&PID_YAW_Rate,PID_YAW_Angle.OutPut,(float)GYRO_Real_z);// YAW角速度环PID

  }

    if(!RC_data.mode)//手动挡
    {
        Moto_PWM_1 = 2000 + RC_data.rc_height + PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
        Moto_PWM_2 = 2000 + RC_data.rc_height + PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;
        Moto_PWM_3 = 2000 + RC_data.rc_height - PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
        Moto_PWM_4 = 2000 + RC_data.rc_height - PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;

    }
    else//自动挡
    {
        //-------------------------混控-------------------------
        //开启--启动缓慢上升然后进行悬停移动定高操作
        if (Control_flag.Plane_Start && !Control_flag.Plane_Land && !Control_flag.Plane_Break)
        {
            if(Control_flag.Plane_Buffer == 1)
            {
                PWM_HOVER = Plane_control.Plane_Speed;
                // 基础油门 + PID修正量
                Moto_PWM_1 = PWM_HOVER + PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
                Moto_PWM_2 = PWM_HOVER + PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;
                Moto_PWM_3 = PWM_HOVER - PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
                Moto_PWM_4 = PWM_HOVER - PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;

            }
            else
            {
                //缓启动
                if(PWM_HOVER >= (Plane_control.Plane_Speed - 20))
                {
                    Control_flag.Plane_Buffer = 1;

                }
                else if(PWM_HOVER >= 2700)
                {
                    // 基础油门 + PID修正量
                    Moto_PWM_1 = PWM_HOVER + PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
                    Moto_PWM_2 = PWM_HOVER + PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;
                    Moto_PWM_3 = PWM_HOVER - PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
                    Moto_PWM_4 = PWM_HOVER - PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;

                    PWM_HOVER += 10;

                }
                else if(PWM_HOVER >= 2000 && PWM_HOVER < 2700)
                {
                    // 基础油门 + PID修正量
                    Moto_PWM_1 = PWM_HOVER;
                    Moto_PWM_2 = PWM_HOVER;
                    Moto_PWM_3 = PWM_HOVER;
                    Moto_PWM_4 = PWM_HOVER;
                    PWM_HOVER += 20;

                }
                else
                {
                    Moto_PWM_1 = 0;
                    Moto_PWM_2 = 0;
                    Moto_PWM_3 = 0;
                    Moto_PWM_4 = 0;
                    PWM_HOVER = 2000;    // 逐步给电机低PWM，避免瞬间启动

                }
            }
        }
        //降落--判断角度逐步减小PWM_HOVER或目标高度
        else if(Control_flag.Plane_Land && !Control_flag.Plane_Break)
        {
            // 基础油门 + PID修正量
            Moto_PWM_1 = PWM_HOVER + PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
            Moto_PWM_2 = PWM_HOVER + PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;
            Moto_PWM_3 = PWM_HOVER - PID_ROL_Rate.OutPut - PID_PIT_Rate.OutPut + PID_YAW_Rate.OutPut;
            Moto_PWM_4 = PWM_HOVER - PID_ROL_Rate.OutPut + PID_PIT_Rate.OutPut - PID_YAW_Rate.OutPut;

            if(PWM_HOVER > 2750)
            {
                PWM_HOVER --;

            }
            else if(PWM_HOVER > 2650)
            {
                PWM_HOVER -= 0.1;

            }
            else
            {
                Control_flag.Plane_Finish = 1;
                Control_flag.Plane_Buffer = 0;
                Control_flag.Plane_Start  = 0;
                PWM_HOVER = 0;

            }

        }
        //急停
        else if(Control_flag.Plane_Break)
        {
            PID_Init();//重置PID积分（避免再次解锁时积分饱和）
            Control_flag.Plane_Start  = 0;
            HEI_HOVER = 0;
            PWM_HOVER = 0;
            Moto_PWM_1 = 0;
            Moto_PWM_2 = 0;
            Moto_PWM_3 = 0;
            Moto_PWM_4 = 0;
            BLDC_PWM(0, 0, 0, 0);

        }
        //待机
        else
        {
            PID_Init();//重置PID
            HEI_HOVER = 0;
            PWM_HOVER = 0;
            Moto_PWM_1 = 0;
            Moto_PWM_2 = 0;
            Moto_PWM_3 = 0;
            Moto_PWM_4 = 0;

        }
    }

  //电机PWM输出
  BLDC_PWM((int)Moto_PWM_1,(int)Moto_PWM_2,(int)Moto_PWM_3,(int)Moto_PWM_4);

}

float yaw_error = 0;

//倾角转换
static float control_wrap_angle_deg(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

//更新导航打点
static void control_update_target_navigation(void)
{
    if (RTK_data.target_index < RTK_data.waypoint)
    {
        //更新经纬度之差
        float dx = RTK_data.x_target[RTK_data.target_index] - RTK_data.x_current;
        float dy = RTK_data.y_target[RTK_data.target_index] - RTK_data.y_current;

        //算实际距离
        RTK_data.distance = sqrtf(dx * dx + dy * dy);
        //算方向角
        RTK_data.azimuth  = control_wrap_angle_deg(atan2f(dx, dy) * 57.29578f);
    }
    else
    {
        RTK_data.distance = 0.0f;
        RTK_data.azimuth  = 0.0f;
    }
}

//车控主函数
void Control_Car(void)
{
    // 转向环：根据目标方位角与当前yaw的偏差计算舵机角度
    if (PID_flag.Turn)
    {
        PID_flag.Turn = 0;

        //目标角度
        float target_yaw_deg = control_wrap_angle_deg(RTK_data.azimuth);

        //实际角度
        float current_yaw_deg = control_wrap_angle_deg(-IMU_Data.yaw / 10.0f);

        //角度偏差
        yaw_error = control_wrap_angle_deg(target_yaw_deg - current_yaw_deg);

        //计算转向环PID输出
        PID_Attitude_Cal(&PID_Turn, yaw_error, 0.0f);
        Servo_Angle = 90.0f - PID_Turn.OutPut;
    }

    // 输出
    Moto_Left_PWM  = Car_control.Car_Speed;
    Moto_Right_PWM = Car_control.Car_Speed;

    //驱动舵机
    Servo_SetAngle(Servo_Angle);
    //加载电机
    MOTO_PWM(Moto_Left_PWM, Moto_Right_PWM);

    //防止起飞时还按原定目标偏航角作为目标值
    Set_Angle.yaw = IMU_Data.yaw;
}

//总控制函数
void Control(void)
{
    const float arrive_distance = 0.5f;        // m
    const float arrive_unlock_distance = 1.2f; // m, hysteresis unlock

    /*
     * 点位类型：
     * 0：车点
     * 1：飞机点
     *
     * 切换类型：
     * 车 -> 车
     * 车 -> 飞机
     * 飞机 -> 车
     * 飞机 -> 飞机
     */
    enum
    {
        POINT_CAR   = 0,
        POINT_PLANE = 1,

        CAR_TO_CAR     = 0,
        CAR_TO_PLANE   = 1,
        PLANE_TO_CAR   = 2,
        PLANE_TO_PLANE = 3
    };

    static uint8_t  switch_type      = CAR_TO_CAR;  // 当前切换类型
    static uint16_t switch_delay_cnt = 0;           // 切换延迟计数
    static uint8_t  wait_plane_land  = 0;           // 飞机转车时等待降落完成
    static uint8_t  arrive_latched   = 0;           // 到点锁存，防止连续跳点

    uint8_t current_type = POINT_CAR;               //初始是车点


    /*********************** 无点位或已经到达终点 ***********************/
    if (RTK_data.waypoint == 0 || RTK_data.target_index >= RTK_data.waypoint)
    {
        switch_type      = CAR_TO_CAR;

        //状态清除
        switch_delay_cnt = 0;
        wait_plane_land  = 0;
        arrive_latched   = 0;

        //标志位
        Control_flag.Car_Start    = 0;
        Control_flag.Plane_Start  = 0;
        Control_flag.Plane_Land   = 0;
        Control_flag.Plane_Buffer = 0;
        Control_flag.Plane_Finish = 0;
        Control_flag.Plane_Break  = 1;

        //相应油门
        Moto_Left_PWM  = 0;
        Moto_Right_PWM = 0;

        PWM_HOVER = 0;
        Moto_PWM_1 = 0;
        Moto_PWM_2 = 0;
        Moto_PWM_3 = 0;
        Moto_PWM_4 = 0;

        //舵机默认值
        Servo_Angle = 90.0f;
        Servo_SetAngle(90);

        MOTO_PWM(0, 0);
        BLDC_PWM(0, 0, 0, 0);

        return;
    }


    /*********************** 起点状态初始化 ***********************/
    /*
     * 当前点是车点，就按车->车普通行驶处理；
     * 当前点是飞机点，就按飞机->飞机普通飞行处理。
     */
    if (RTK_data.target_index <= 1 && wait_plane_land == 0 && switch_delay_cnt == 0)
    {
        if (RTK_data.switchpoint[RTK_data.target_index] == POINT_PLANE)
        {
            switch_type = CAR_TO_PLANE;
        }
        else
        {
            switch_type = CAR_TO_CAR;
        }
    }


    /*********************** 到点后，判断当前点到下一个点属于哪种切换 ***********************/
    /*
     * 距离 <= 0.5 m，认为到达当前目标点。
     * arrive_latched 用来防止 GPS 距离还没刷新时连续 target_index++。
     */
    if (RTK_data.distance > arrive_unlock_distance)
    {
        arrive_latched = 0;
    }

    if (gnss.state && !arrive_latched && RTK_data.distance <= arrive_distance)
    {

        /*
         * 只有下一个点存在时，才能读取 target_index + 1。
         * waypoint 是总点数，target_index 从 0 开始。
         */
        if ((RTK_data.target_index + 1) < RTK_data.waypoint)
        {
            uint8_t old_type = RTK_data.switchpoint[RTK_data.target_index];      // 当前点类型
            uint8_t new_type = RTK_data.switchpoint[RTK_data.target_index + 1];  // 下一个点类型

            //先判断切换类型，再切到下一个目标点。
            if (old_type == POINT_CAR && new_type == POINT_CAR)
            {
                switch_type      = CAR_TO_CAR;
                switch_delay_cnt = 0;      // 车->车不延迟，车继续走
                wait_plane_land  = 0;
            }
            else if (old_type == POINT_CAR && new_type == POINT_PLANE)
            {
                switch_type      = CAR_TO_PLANE;
                switch_delay_cnt = 50;    // 车->飞机，延迟 200 次让舵机回正
                wait_plane_land  = 0;
            }
            else if (old_type == POINT_PLANE && new_type == POINT_CAR)
            {
                switch_type      = PLANE_TO_CAR;
                switch_delay_cnt = 0;
                wait_plane_land  = 1;      // 飞机->车，必须先等待降落完成
            }
            else
            {
                switch_type      = PLANE_TO_PLANE;
                switch_delay_cnt = 0;      // 飞机->飞机不延迟，继续飞
                wait_plane_land  = 0;
            }

            RTK_data.target_index++;
            arrive_latched = 1;
            control_update_target_navigation();
            if (RTK_data.distance > arrive_unlock_distance)
            {
                arrive_latched = 0;
            }

        }
        else
        {
            //当前点已经是最后一个点，到达后直接结束任务。
            RTK_data.target_index = RTK_data.waypoint;
            arrive_latched = 1;

            Control_flag.Car_Start    = 0;
            Control_flag.Plane_Start  = 0;
            Control_flag.Plane_Land   = 0;
            Control_flag.Plane_Buffer = 0;
            Control_flag.Plane_Finish = 0;
            Control_flag.Plane_Break  = 1;

            Moto_Left_PWM  = 0;
            Moto_Right_PWM = 0;

            PWM_HOVER = 0;
            Moto_PWM_1 = 0;
            Moto_PWM_2 = 0;
            Moto_PWM_3 = 0;
            Moto_PWM_4 = 0;

            Servo_Angle = 90.0f;
            Servo_SetAngle(90);

            MOTO_PWM(0, 0);
            BLDC_PWM(0, 0, 0, 0);

            return;
        }
    }



    /*********************** 车 -> 车 ***********************/
    if (switch_type == CAR_TO_CAR)
    {
        /*
         * 车点到车点：
         * 车继续发车；
         * 飞机全部关闭；
         * 不要求 Plane_Finish。
         */
        Control_flag.Car_Start    = 1;
        Control_flag.Plane_Start  = 0;
        Control_flag.Plane_Land   = 0;
        Control_flag.Plane_Buffer = 0;
        Control_flag.Plane_Finish = 0;
        Control_flag.Plane_Break  = 0;

        PWM_HOVER = 0;
        Moto_PWM_1 = 0;
        Moto_PWM_2 = 0;
        Moto_PWM_3 = 0;
        Moto_PWM_4 = 0;

        BLDC_PWM(0, 0, 0, 0);

        Control_Car();
        return;
    }

    /*********************** 车 -> 飞机 ***********************/
    else
    {
        Beep_OFF();
    }

    if (switch_type == CAR_TO_PLANE)
    {
        /*
         * 车点到飞机点：
         * 车停止；
         * 舵机回正到 90 度；
         * 先延迟 200 次；
         * 延迟结束后飞机起飞。
         */
        Control_flag.Car_Start = 0;

        Moto_Left_PWM  = 0;
        Moto_Right_PWM = 0;

        Servo_Angle = 90.0f;
        Servo_SetAngle(90);
        MOTO_PWM(0, 0);

        //延迟开始
        if (switch_delay_cnt > 0)
        {
            switch_delay_cnt--;

            Control_flag.Plane_Start  = 0;
            Control_flag.Plane_Land   = 0;
            Control_flag.Plane_Buffer = 0;
            Control_flag.Plane_Finish = 0;
            Control_flag.Plane_Break  = 0;

            PWM_HOVER = 0;
            Moto_PWM_1 = 0;
            Moto_PWM_2 = 0;
            Moto_PWM_3 = 0;
            Moto_PWM_4 = 0;

            BLDC_PWM(0, 0, 0, 0);
            Beep_OFF();

            return;
        }
        else//延迟结束，飞机进入正常起飞
        {
            Beep_ON();
            Control_flag.Plane_Start  = 1;
            arrive_latched = 0;

            Control_Plane(&IMU_Data, &Set_Angle);
            return;
        }
    }
    else
    {
            Beep_OFF();
    }


    /*********************** 飞机 -> 车 ***********************/
    if (switch_type == PLANE_TO_CAR)
    {
        /*
         * 飞机点到车点：
         * 不能马上发车；
         * 先让飞机降落；
         * 等 Plane_Finish == 1；
         * 再延迟 200 次清零标志位；
         * 最后车发车。
         */

        if (wait_plane_land)
        {
            Control_flag.Car_Start    = 0;
            Control_flag.Plane_Start  = 1; 
            Control_flag.Plane_Land   = 1;
            Control_flag.Plane_Buffer = 0;

            Moto_Left_PWM  = 0;
            Moto_Right_PWM = 0;

            Servo_Angle = 90.0f;
            Servo_SetAngle(90);
            MOTO_PWM(0, 0);

            //飞机还没降落完成，继续执行飞机降落控制。
            if (Control_flag.Plane_Finish == 0)
            {
                Control_Plane(&IMU_Data, &Set_Angle);
                return;
            }
            else//飞机降落完成，进入 50 次清零延迟。
            {
                wait_plane_land  = 0;
                switch_delay_cnt = 50;
            }

            return;
        }

        //延迟开始，调整车的姿态
        if (switch_delay_cnt > 0)
        {
            switch_delay_cnt--;

            Control_flag.Car_Start    = 0;
            Control_flag.Plane_Start  = 0;
            Control_flag.Plane_Land   = 0;
            Control_flag.Plane_Buffer = 0;
            Control_flag.Plane_Finish = 0; 

            Moto_Left_PWM  = 0;
            Moto_Right_PWM = 0;

            PWM_HOVER = 0;
            Moto_PWM_1 = 0;
            Moto_PWM_2 = 0;
            Moto_PWM_3 = 0;
            Moto_PWM_4 = 0;

            Servo_Angle = 90.0f;
            Servo_SetAngle(90);

            MOTO_PWM(0, 0);
            BLDC_PWM(0, 0, 0, 0);

            return;
        }
        /*
         * 降落完成 + 延迟结束，车开始发车。
         * 从这里开始，不再关心 Plane_Finish 是否为 1。
         */
        else
        {
            Control_flag.Car_Start    = 1;
            Control_flag.Plane_Start  = 0;
            Control_flag.Plane_Land   = 0;
            Control_flag.Plane_Buffer = 0;
            Control_flag.Plane_Finish = 0;

            PWM_HOVER = 0;
            Moto_PWM_1 = 0;
            Moto_PWM_2 = 0;
            Moto_PWM_3 = 0;
            Moto_PWM_4 = 0;

            BLDC_PWM(0, 0, 0, 0);

            Control_Car();
            return;
        }

    }

    /*********************** 飞机 -> 飞机 ***********************/
    if (switch_type == PLANE_TO_PLANE)
    {
        /*
         * 飞机点到飞机点：
         * 车停止；
         * 舵机回正；
         * 飞机继续飞；
         * 不需要降落；
         * 不需要 200 次延迟。
         */
        Control_flag.Car_Start    = 0;
        Control_flag.Plane_Start  = 1;

        Moto_Left_PWM  = 0;
        Moto_Right_PWM = 0;

        Servo_Angle = 90.0f;
        Servo_SetAngle(90);
        MOTO_PWM(0, 0);

        Control_Plane(&IMU_Data, &Set_Angle);
        return;
    }
}
