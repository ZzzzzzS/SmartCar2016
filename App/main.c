

#include "common.h"
#include "include.h"

//电压7.35-7.36

#define MOTOR1_IO   PTC12
#define MOTOR2_IO   PTE31
#define MOTOR3_IO   PTC13
#define MOTOR4_IO   PTC5

#define MOTOR_TPM   TPM0
#define MOTOR1_PWM  TPM_CH0
#define MOTOR2_PWM  TPM_CH1
#define MOTOR3_PWM  TPM_CH2
#define MOTOR4_PWM  TPM_CH3

#define MOTOR1_PWM_IO  TPM0_CH0
#define MOTOR2_PWM_IO  TPM0_CH1
#define MOTOR3_PWM_IO  TPM0_CH2
#define MOTOR4_PWM_IO  TPM0_CH3

#define MOTOR_HZ    (20*1000)

#define angle_set 15


char start_speed=10;


char table_code[42]={60,60,60,60,55,50,45,45,40,40,40,30,30,30,25,25,20,20,15,15,15,15,10,10,10,3,3,3,2,2,2,2,1,1,1,1,0,0,0,0};
                     //60有效

uint8 imgbuff[CAMERA_SIZE];                             //定义存储接收图像的数组
                           //由于鹰眼摄像头是一字节8个像素，因而需要解压为 1字节1个像素，方便处理
uint8 g_zzs_image[CAMERA_W*CAMERA_H];                              //转换到二维数组

uint8 wall_left=0;
uint8 wall_right=0;



//函数声明
void vcan_sendimg(uint8 *imgaddr, uint32 imgsize);
void img_extract(uint8 *dst, uint8 *src, uint32 srclen);
void PORTA_IRQHandler();
void DMA0_IRQHandler();





int turn(void)
{
	int zzs_i =2400, zzs_j;
	int zzs_number=0;
        int zzs_weight=0;

	for(zzs_i=30*80;zzs_i<4800;zzs_i+=80)
		for(zzs_j=0;zzs_j<80;zzs_j+=1)
		{
			if(g_zzs_image[zzs_i+zzs_j]!=0)
			{
				      //i为纵坐标 j为横坐标
                                if(zzs_j<40)
                                  zzs_weight-=table_code[zzs_j]*((zzs_i/80)-30);
                                else if(zzs_j>=40)                                  
                                  zzs_weight+=table_code[80-zzs_j-1]*((zzs_i/80)-30); 
			}
		}

        zzs_number=zzs_weight;
        if(zzs_number>100)
        {
          led(LED0, LED_ON);                  
          led(LED3, LED_OFF); 
        }
        if(zzs_number<-100)
        {
          led(LED3, LED_ON);                  
          led(LED0, LED_OFF);
        }
        if(zzs_number>=-100 && zzs_number<=100)
        {      
          led(LED3, LED_ON);                  
          led(LED0, LED_ON);
        }
        return -zzs_number;
}


void control_motor(int angle)
{   
  int turn=0;
  
    tpm_pwm_duty(MOTOR_TPM, MOTOR2_PWM,start_speed);
    tpm_pwm_duty(MOTOR_TPM, MOTOR4_PWM,start_speed);
    
    if(angle>0)
    {
      turn+=(int)sqrt((double)angle/angle_set);
    }
    else if(angle<0)
    {
      angle=-angle;
      turn-=(int)sqrt((double)angle/angle_set);
    }
    if(turn>=100-start_speed)
      turn=100-start_speed;
    if(turn>0)
    {
      tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,100 - turn);//turn
      tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,100);
    }
    else if(turn<0)
    {
      tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,100 + turn);//turn
      tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,100);
      
    }
    else if(turn==0)
    {
      tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,100);
      tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,100);
      
    }
}













/*!
 *  @brief      main函数
 *  @since      v5.0
 *  @note       山外 DMA 采集摄像头 实验
 */
void  main(void)
{
  unsigned int flag=0;
  
  led_init(LED0);                         //初始化LED
  led_init(LED3);
  led_init(LED2);
  led_init(LED1);

  
  
  tpm_pwm_init(MOTOR_TPM, MOTOR1_PWM,MOTOR_HZ,100);      //初始化 电机 PWM
  tpm_pwm_init(MOTOR_TPM, MOTOR2_PWM,MOTOR_HZ,100);      //初始化 电机 PWM
  tpm_pwm_init(MOTOR_TPM, MOTOR3_PWM,MOTOR_HZ,100);      //初始化 电机 PWM
  tpm_pwm_init(MOTOR_TPM, MOTOR4_PWM,MOTOR_HZ,100);      //初始化 电机 PWM
  
  
  //IO管脚配置
  gpio_init(MOTOR1_IO,GPO,LOW);
  gpio_init(MOTOR2_IO,GPO,LOW);
  gpio_init(MOTOR3_IO,GPO,LOW);
  gpio_init(MOTOR4_IO,GPO,LOW);
  
  
  //初始化串口
 // uart_init(UART0,115200);
    
    //初始化摄像头
    camera_init(imgbuff);

    //配置中断服务函数
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    //设置LPTMR的中断服务函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      //设置LPTMR的中断服务函数为 PORTA_IRQHandler

    
    tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,100);   //电机左
    tpm_pwm_duty(MOTOR_TPM, MOTOR2_PWM,start_speed);

    
    tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,100);   //电机右
    tpm_pwm_duty(MOTOR_TPM, MOTOR4_PWM,start_speed);
    
    
    gpio_init (PTA17,  GPI , 0);          //红外检测端口
    gpio_init (PTA16,  GPI , 0); 
    
    
    
    port_init (PTA17,  ALT1 | PULLUP ); 
    port_init (PTA16,  ALT1 | PULLUP );
    
    
    
    
    
    while(1)
    {
        //获取图像
        camera_get_img();                                   //摄像头获取图像

        //多功能调试助手上位机显示，需要配置成黑白模式
        //vcan_sendimg(imgbuff,CAMERA_SIZE);


        //解压图像  ，把解压的数据放到 img 数据里。
        img_extract(g_zzs_image,imgbuff,CAMERA_SIZE);
        
        //获得红外数据      
         wall_left=gpio_get(PTA17);
         wall_right=gpio_get(PTA16); 
         
         
         
         
         
         
         
         if(wall_right==0)    //判断墙壁
        {
           led(LED1, LED_ON);
           control_motor(-15*15*15);
        }
 
          else if(wall_left==0)     //判断墙壁
	{ 
          if(flag<=10)
          {
            led(LED2, LED_ON);
            control_motor(-30*30*15);
            DELAY_MS(2);
            flag++;
          }
          else
          {
            control_motor(0);
            led(LED2, LED_OFF);
          }          
	}
	else 
        {
            if(flag>=10)
              start_speed=40;
            control_motor(turn());//控制电机转弯 
            led(LED1, LED_OFF);
	}         
         
         
         
         
         
         
         
         
         
         
         
    }
}

//发送图像到上位机显示
//不同的上位机，不同的命令
//如果使用其他上位机，则需要修改代码
void vcan_sendimg(uint8 *imgaddr, uint32 imgsize)
{
    #define CMD_IMG  1
    uint8 cmdf[2] = {CMD_IMG, ~CMD_IMG};    //山外上位机 使用的命令
    uint8 cmdr[2] = {~CMD_IMG, CMD_IMG};    //山外上位机 使用的命令

    uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));    //先发送命令

    uart_putbuff(VCAN_PORT, imgaddr, imgsize); //再发送图像

    uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));    //先发送命令
}


/*!
 *  @brief      二值化图像解压（空间 换 时间 解压）
 *  @param      dst             图像解压目的地址
 *  @param      src             图像解压源地址
 *  @param      srclen          二值化图像的占用空间大小
 *  @since      v5.0            img_extract(img, imgbuff,CAMERA_SIZE);
 *  Sample usage:
 */
void img_extract(uint8 *dst, uint8 *src, uint32 srclen)
{
    uint8 colour[2] = {255, 0}; //0 和 1 分别对应的颜色
    //注：山外的摄像头 0 表示 白色，1表示 黑色
    uint8 tmpsrc;
    while(srclen --)
    {
        tmpsrc = *src++;
        *dst++ = colour[ (tmpsrc >> 7 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 6 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 5 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 4 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 3 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 2 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 1 ) & 0x01 ];
        *dst++ = colour[ (tmpsrc >> 0 ) & 0x01 ];
    }
}

/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n = 0;    //引脚号
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 6;                                              //场中断
    if(flag & (1 << n))                                 //PTA6触发中断
    {
        camera_vsync();
    }
}

/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}