#include "sys.h"

// 遥控接收结构体外部变量，存放各通道值、更新标志、在线状态
RC_Data RC_data;

static uint8_t rc_rx_buf[RC_FRAME_LEN];    // 遥控完整帧接收缓存
static uint8_t rc_rx_index = 0;            // 接收缓存索引
static uint16_t rc_lost_cnt = 0;           // 遥控丢帧计数

#define RC_VALID_CHANNEL_NUM   (6)         // 遥控器通道数量
#define RC_CHANNEL_VALUE_MIN   (800)       // 遥控通道PWM数值下限
#define RC_CHANNEL_VALUE_MAX   (2200)      // 遥控通道PWM数值上限

uint8 uart_get_data[64];                   // UART底层FIFO挂载缓冲区
uint8 fifo_get_data[64];                   // 从FIFO读出数据的临时缓冲区
uint32 fifo_data_count = 0;                // FIFO内有效数据字节个数
uint8 get_data = 0;                        // 单字节临时存储变量
fifo_struct uart_data_fifo;                // 串口软件FIFO结构体


//数据校验函数
uint16_t RC_Check(const uint8_t *buf)
{
    uint16_t sum = 0xFFFF;

    //计算得到的16位校验和
    for(uint8_t i = 0; i < 30; i++)
    {
        sum -= buf[i];
    }

    return sum;
}

//帧解析函数 
uint8_t RC_ParseFrame(const uint8_t *buf)
{
    uint16_t calc_sum;
    uint16_t recv_sum;
    uint16_t ch_temp[RC_CHANNEL_NUM];

    // 空指针退出
    if(buf == NULL)
    {
        return 0;
    }

    // 判断帧头 0x20 0x40
    if(buf[0] != 0x20 || buf[1] != 0x40)
    {
        return 0;
    }

    calc_sum = RC_Check(buf);

    // 取出帧内接收的校验和（小端）
    recv_sum = (uint16_t)buf[30] | ((uint16_t)buf[31] << 8);

    // 如果校验和不一致，丢弃本帧退出
    if(calc_sum != recv_sum)
    {
        return 0;
    }

    // 将原始字节拼接为16位通道数值
    for(uint8_t i = 0; i < RC_CHANNEL_NUM; i++)
    {
        ch_temp[i] = (uint16_t)buf[2 + i * 2] |
                    ((uint16_t)buf[3 + i * 2] << 8);
    }

    // 通道数值范围校验
    for(uint8_t i = 0; i < RC_VALID_CHANNEL_NUM; i++)
    {
        if(ch_temp[i] < RC_CHANNEL_VALUE_MIN || ch_temp[i] > RC_CHANNEL_VALUE_MAX)
        {
            return 0;
        }
    }

    // 解析成功，通道数据赋值到遥控结构体
    for(uint8_t i = 0; i < RC_CHANNEL_NUM; i++)
    {
        RC_data.ch[i] = ch_temp[i];
    }

    RC_data.update_flag = 1;   // 置位新数据更新标志位，主循环读取后清零
    RC_data.online = 1;        // 标记遥控器在线
    rc_lost_cnt = 0;           // 丢帧计数器清零

    return 1;
}

//单字节输入状态机 
void RC_ByteInput(uint8_t data)
{
    // 状态0：等待第一个帧头字节0x20
    if(rc_rx_index == 0)
    {
        if(data == 0x20)
        {
            rc_rx_buf[rc_rx_index++] = data;
        }
    }
    // 状态1：已经收到0x20，等待第二个帧头0x40
    else if(rc_rx_index == 1)
    {
        if(data == 0x40)
        {
            rc_rx_buf[rc_rx_index++] = data;
        }
        else if(data == 0x20)
        {
            // 收到新0x20，更新帧头，索引保持1
            rc_rx_buf[0] = data;
            rc_rx_index = 1;
        }
        else
        {
            // 字节不匹配帧头，重置状态机
            rc_rx_index = 0;
        }
    }
    // 状态2：接收帧体剩余字节
    else
    {
        rc_rx_buf[rc_rx_index++] = data;
        // 接收字节达到帧长度，进行帧解析，重置索引
        if(rc_rx_index >= RC_FRAME_LEN)
        {
            RC_ParseFrame(rc_rx_buf);
            rc_rx_index = 0;
        }
    }
}


//串口接收中断回调 
void RC_Callback(uint32 event, void *ptr)
{
    (void)event;
    (void)ptr;

    zf_uart_query_byte(RC_UART, &get_data);              // 查询是否收到1字节数据
    zf_fifo_write_buffer(&uart_data_fifo, &get_data, 1); // 将字节存入软件FIFO缓存
}

//遥控初始化 
void RC_Init(void)
{
    memset(&RC_data, 0, sizeof(RC_Data));
    rc_rx_index = 0;
    rc_lost_cnt = 0;

    zf_fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);       // 初始化串口软件FIFO
    zf_uart_init(RC_UART, RC_UART_BAUDRATE, RC_UART_TX, RC_UART_RX);        // 初始化遥控串口硬件引脚波特率

    zf_uart_set_rx_interrupt(RC_UART, RC_Callback, NULL);                   // 注册串口接收中断回调函数
    zf_uart_set_interrupt_config(RC_UART, UART_INTERRUPT_CONFIG_RX_ENABLE); // 使能串口接收中断
    zf_interrupt_set_priority((interrupt_index_enum)zf_uart_irq_map[RC_UART], INTERRUPT_PRIORITY_2); // 设置串口中断优先级
}

//遥控轮询处理函数
void RC_poll_rx(void)
{
	fifo_data_count = zf_fifo_used(&uart_data_fifo);                            // 查看 fifo 是否有数据
	if(fifo_data_count != 0)                                                    // 读取到数据了
	{
        //读出FIFO数据并清空FIFO
		zf_fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);   

        // 逐个字节识别遥控帧
		for (uint32_t i = 0; i < fifo_data_count; i++)
		{
			RC_ByteInput(fifo_get_data[i]);
		}
	}
}

//遥控在线检测--保证实时连接，防止失控 
void RC_Online_Check(void)
{
    if(RC_data.update_flag)
    {
        RC_data.online = 1;
        rc_lost_cnt = 0;
    }
    else
    {
        if(rc_lost_cnt < 500)
        {
            rc_lost_cnt++;
        }

        if(rc_lost_cnt > 50)
        {
            RC_data.online = 0;
        }
    }
}

//遥控数据换算 
void RC_Update_Calibration(void)
{
    //读取解析帧数据
    int16_t ch0 = RC_data.ch[0];
    int16_t ch1 = RC_data.ch[1];
    int16_t ch2 = RC_data.ch[2];
    int16_t ch3 = RC_data.ch[3];

    //通道一至通道四
    if (ch0 > 1550)         RC_data.rc_roll = (ch0 - 1550) / 25.0f;
    else if (ch0 < 1450)    RC_data.rc_roll = (ch0 - 1450) / 25.0f;
    else                    RC_data.rc_roll = 0.0f;

    if (ch1 > 1550)         RC_data.rc_pitch = (ch1 - 1550) / 25.0f;
    else if (ch1 < 1450)    RC_data.rc_pitch = (ch1 - 1450) / 25.0f;
    else                    RC_data.rc_pitch = 0.0f;

    if (ch2 <= 1200)        RC_data.rc_height = 0.0f;
    else                    RC_data.rc_height = (ch2 - 1200) / 8.0f;

    if (ch3 > 1550)         RC_data.rc_servo = (ch3 - 1550) / 25.0f;
    else if (ch3 < 1450)    RC_data.rc_servo = (ch3 - 1450) / 25.0f;
    else                    RC_data.rc_servo = 0.0f;
}

//整体遥控器数据处理
void RC_Data_Cal(void)
{
    //读取解析帧数据
    int16_t ch0 = RC_data.ch[0];
    int16_t ch1 = RC_data.ch[1];

    //遥控数据换算 
    RC_Update_Calibration();

    //用与判断当前是控制车还是飞机
    if (RC_data.ch[4] == 1000)//控制车
    {
        Moto_PWM_1 = 0;
        Moto_PWM_2 = 0;
        Moto_PWM_3 = 0;
        Moto_PWM_4 = 0;

        PID_Init();

        Control_flag.Plane_Start = 0;
        Control_flag.Plane_Break = 1;

        BLDC_PWM(0, 0, 0, 0);
    }
    else if(RC_data.ch[4] == 2000)//控制飞机
    {
        Control_flag.Plane_Start = 1;
        Control_flag.Plane_Break = 0;
    }

    //用与判断飞机是否降落以及当前是自动挡还是手动挡--上电默认手动挡
    if (RC_data.ch[5] == 1000)       Control_flag.Plane_Land = 0;
    else if(RC_data.ch[5] == 2000)   Control_flag.Plane_Land = 1;

    //死区限制--用于判断是否控制移动
    if(ch0 <= 1550 && ch0 >= 1450 && ch1 <= 1550 && ch1 >= 1450)  Control_flag.RC = 0;
    else                                                          Control_flag.RC = 1;

    RC_data.update_flag = 0;
}

//最终控制函数
void RC_Control(void)
{
    //没收到遥控器数据--安全保护
    if(!RC_data.online)
    {
        Moto_PWM_1 = 0;
        Moto_PWM_2 = 0;
        Moto_PWM_3 = 0;
        Moto_PWM_4 = 0;
        BLDC_PWM(0,0,0,0);
        MOTO_PWM(0,0);
        Servo_SetAngle(90);
        return;
    }

    if(!RC_data.mode)//手动挡
    {
        if(!RC_data.change)//车控部分 
        {
            Moto_PWM_1 = 0;
            Moto_PWM_2 = 0;
            Moto_PWM_3 = 0;
            Moto_PWM_4 = 0;
            BLDC_PWM(0,0,0,0);
            Servo_SetAngle(90-RC_data.rc_servo);
            MOTO_PWM(RC_data.rc_roll*100,RC_data.rc_roll*100);                
        }
        else//飞控部分    
        {
            Control_Plane(&IMU_Data, &Set_Angle);
        }
    }
    else//自动挡
    {
        if(RC_data.change) Control_flag.Plane_Land = 1;
        else               Control_flag.Plane_Land = 0;   
        Control();         //总控制程序
    }
}
