#include "sys.h"

  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
u8 Way_Angle=1;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 默认搭载卡尔曼滤波
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //蓝牙遥控相关的变量
u8 Flag_Stop=1,Flag_Show=1;                 //停止标志位和 显示标志位 默认停止 显示打开
int Encoder_Left,Encoder_Right;             //左右编码器的脉冲计数
int Moto1,Moto2;                            //电机PWM变量 应是Motor的 向Moto致敬	
int Temperature;                            //显示温度
int Voltage;                                //电池电压采样相关的变量
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
float Show_Data_Mb;                         //全局显示变量，用于显示需要查看的数据
int main(void)
{ 
	u8 i = 0;
	short gx,gy,gz;
	short ax,ay,az;
	Stm32_Clock_Init(9);            //系统时钟设置
	delay_init(72);                 //延时初始化
	JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
	LED_Init();                     //初始化与 LED 连接的硬件接口
	KEY_Init();                     //按键初始化
	//OLED_Init();                    //OLED初始化
	uart_init(72,115200);           //初始化串口1
	uart3_init(36,9600);            //串口3初始化
	Adc_Init();                     //ADC初始化
	MiniBalance_PWM_Init(7199,0);   //=====初始化PWM 10KHZ 高频可以防止电机低频时的尖叫声
	Encoder_Init_TIM2();            //初始化编码器1 
	Encoder_Init_TIM4();            //初始化编码器2 
	IIC_Init();                     //模拟IIC初始化
  MPU6050_initialize();           //=====MPU6050初始化	
	DMP_Init();                     //初始化DMP     
  Timer1_Init(49,7199);           //=====5MS进一次中断服务函数，中断服务函数在control.c
	while(1)
		{
			if(Way_Angle==1)                     //DMP没有涉及到严格的时序问题，在主函数读取
				{
					Read_DMP();                      //===读取角速度和倾角
					Angle_Balance=Pitch;             //===更新平衡倾角
					Gyro_Balance=gyro[1];            //===更新平衡角速度
					Gyro_Turn=gyro[2];               //===更新转向角速度
					ax = accel[0];
					ay = accel[1];
					az = accel[2];
					
					gx = gyro[0];
					gy = gyro[1];
					gz = gyro[2];
					

					
						//mpu6050_send_data(ax,ay,az,gx,gy,gz);//用自定义帧发送加速度和陀螺仪原始数据
						//usart1_report_imu(ax,ay,az,gx,gy,gz,(int)(Roll*100),(int)(Pitch*100),(int)(Yaw*10));
						i = 0;
					
					//if(Angle_Balance<0)		printf("角度：%f\n", Angle_Balance + 360);
					//else					        printf("角度：%f\n", Angle_Balance);
					//printf("角度：%f\n", Angle_Balance);
					//usart1_send_char((u8)Angle_Balance);
				}

				if(Flag_Stop==1||Way_Angle>1)      //===电机关闭后或者没有使用DMP时，开启上位机监控
				{
					if(Flag_Show==1)  //使用MiniBalanceV3.5 APP
					{
					delay_ms(500);//延时减缓数据传输频率，确保通信的稳定
				  //APP_Show();
					}
					else             //使用OLED显示屏和PC端串口调试助手
					{
					Temperature=Read_Temperature();  //===读取MPU6050内置温度传感器数据，近似表示主板温度。	
					//oled_show(); //===显示屏打开
				  printf("平衡倾角%f  左轮编码器%d  右轮编码器%d  电池电压%dmV\r\n",Angle_Balance,Encoder_Left,Encoder_Right,Voltage);//向上位机发送数据
					if(Angle_Balance<0)		printf("角度：%f\n", Angle_Balance + 360);
					else					        printf("角度：%f\n", Angle_Balance);
					delay_ms(100);	//延时减缓数据传输频率，确保通信的稳定
					}	
				}
		} 
}
