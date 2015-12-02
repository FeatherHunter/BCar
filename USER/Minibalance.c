#include "sys.h"

  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
u8 Way_Angle=1;                             //��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲� Ĭ�ϴ��ؿ������˲�
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //����ң����صı���
u8 Flag_Stop=1,Flag_Show=1;                 //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
int Encoder_Left,Encoder_Right;             //���ұ��������������
int Moto1,Moto2;                            //���PWM���� Ӧ��Motor�� ��Moto�¾�	
int Temperature;                            //��ʾ�¶�
int Voltage;                                //��ص�ѹ������صı���
float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ�������� ת��������
float Show_Data_Mb;                         //ȫ����ʾ������������ʾ��Ҫ�鿴������
int main(void)
{ 
	u8 i = 0;
	short gx,gy,gz;
	short ax,ay,az;
	Stm32_Clock_Init(9);            //ϵͳʱ������
	delay_init(72);                 //��ʱ��ʼ��
	JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
	JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
	LED_Init();                     //��ʼ���� LED ���ӵ�Ӳ���ӿ�
	KEY_Init();                     //������ʼ��
	//OLED_Init();                    //OLED��ʼ��
	uart_init(72,115200);           //��ʼ������1
	uart3_init(36,9600);            //����3��ʼ��
	Adc_Init();                     //ADC��ʼ��
	MiniBalance_PWM_Init(7199,0);   //=====��ʼ��PWM 10KHZ ��Ƶ���Է�ֹ�����Ƶʱ�ļ����
	Encoder_Init_TIM2();            //��ʼ��������1 
	Encoder_Init_TIM4();            //��ʼ��������2 
	IIC_Init();                     //ģ��IIC��ʼ��
  MPU6050_initialize();           //=====MPU6050��ʼ��	
	DMP_Init();                     //��ʼ��DMP     
  Timer1_Init(49,7199);           //=====5MS��һ���жϷ��������жϷ�������control.c
	while(1)
		{
			if(Way_Angle==1)                     //DMPû���漰���ϸ��ʱ�����⣬����������ȡ
				{
					Read_DMP();                      //===��ȡ���ٶȺ����
					Angle_Balance=Pitch;             //===����ƽ�����
					Gyro_Balance=gyro[1];            //===����ƽ����ٶ�
					Gyro_Turn=gyro[2];               //===����ת����ٶ�
					ax = accel[0];
					ay = accel[1];
					az = accel[2];
					
					gx = gyro[0];
					gy = gyro[1];
					gz = gyro[2];
					

					
						//mpu6050_send_data(ax,ay,az,gx,gy,gz);//���Զ���֡���ͼ��ٶȺ�������ԭʼ����
						//usart1_report_imu(ax,ay,az,gx,gy,gz,(int)(Roll*100),(int)(Pitch*100),(int)(Yaw*10));
						i = 0;
					
					//if(Angle_Balance<0)		printf("�Ƕȣ�%f\n", Angle_Balance + 360);
					//else					        printf("�Ƕȣ�%f\n", Angle_Balance);
					//printf("�Ƕȣ�%f\n", Angle_Balance);
					//usart1_send_char((u8)Angle_Balance);
				}

				if(Flag_Stop==1||Way_Angle>1)      //===����رպ����û��ʹ��DMPʱ��������λ�����
				{
					if(Flag_Show==1)  //ʹ��MiniBalanceV3.5 APP
					{
					delay_ms(500);//��ʱ�������ݴ���Ƶ�ʣ�ȷ��ͨ�ŵ��ȶ�
				  //APP_Show();
					}
					else             //ʹ��OLED��ʾ����PC�˴��ڵ�������
					{
					Temperature=Read_Temperature();  //===��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�	
					//oled_show(); //===��ʾ����
				  printf("ƽ�����%f  ���ֱ�����%d  ���ֱ�����%d  ��ص�ѹ%dmV\r\n",Angle_Balance,Encoder_Left,Encoder_Right,Voltage);//����λ����������
					if(Angle_Balance<0)		printf("�Ƕȣ�%f\n", Angle_Balance + 360);
					else					        printf("�Ƕȣ�%f\n", Angle_Balance);
					delay_ms(100);	//��ʱ�������ݴ���Ƶ�ʣ�ȷ��ͨ�ŵ��ȶ�
					}	
				}
		} 
}
