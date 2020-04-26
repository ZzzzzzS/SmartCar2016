

#include "common.h"
#include "include.h"

//��ѹ7.35-7.36
//7.5Ҳ��

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

char angle_set=15;    //15 ע����滹�и�angle_setҪ�޸�

char stop_time=85;

char start_speed=100;

unsigned int flag=0,time=0;

char table_code[42]={60,60,60,60,55,50,45,45,40,40,40,30,30,30,25,25,20,20,15,15,15,15,10,10,10,3,3,3,2,2,2,2,1,1,1,1,0,0,0,0};
                     //60��Ч

uint8 imgbuff[CAMERA_SIZE];                             //����洢����ͼ�������
                           //����ӥ������ͷ��һ�ֽ�8�����أ������Ҫ��ѹΪ 1�ֽ�1�����أ����㴦��
uint8 g_zzs_image[CAMERA_W*CAMERA_H];                              //ת������ά����

uint8 wall_left=0;
uint8 wall_right=0;



//��������
void vcan_sendimg(uint8 *imgaddr, uint32 imgsize);
void img_extract(uint8 *dst, uint8 *src, uint32 srclen);
void PORTA_IRQHandler();
void DMA0_IRQHandler();



void LPTMR_IRQHandler(void)     //ͣ��
{   
  time++;
  if(flag>=8&&time>=stop_time)//����һ����ͣ��
  {
    tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,100);
    tpm_pwm_duty(MOTOR_TPM, MOTOR2_PWM,100);
    tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,100);
    tpm_pwm_duty(MOTOR_TPM, MOTOR4_PWM,100); 
    led(LED1, LED_ON);                  
    led(LED2, LED_ON); 
    led(LED3, LED_ON); 
    led(LED0, LED_ON); 
    while(1);
  }
  if(flag<=5)
    time=0;
  LPTMR_Flag_Clear();       //���жϱ�־λ
}












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
				      //iΪ������ jΪ������
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
 *  @brief      main����
 *  @since      v5.0
 *  @note       ɽ�� DMA �ɼ�����ͷ ʵ��
 */
void  main(void)
{
  char inin=0;
  
  led_init(LED0);                         //��ʼ��LED
  led_init(LED3);
  led_init(LED2);
  led_init(LED1);

  
  
  tpm_pwm_init(MOTOR_TPM, MOTOR1_PWM,MOTOR_HZ,100);      //��ʼ�� ��� PWM
  tpm_pwm_init(MOTOR_TPM, MOTOR2_PWM,MOTOR_HZ,100);      //��ʼ�� ��� PWM
  tpm_pwm_init(MOTOR_TPM, MOTOR3_PWM,MOTOR_HZ,100);      //��ʼ�� ��� PWM
  tpm_pwm_init(MOTOR_TPM, MOTOR4_PWM,MOTOR_HZ,100);      //��ʼ�� ��� PWM
  
  
  //IO�ܽ�����
  gpio_init(MOTOR1_IO,GPO,LOW);
  gpio_init(MOTOR2_IO,GPO,LOW);
  gpio_init(MOTOR3_IO,GPO,LOW);
  gpio_init(MOTOR4_IO,GPO,LOW);
  
  
  //��ʼ������
 // uart_init(UART0,115200);
    
    //��ʼ������ͷ
   // camera_init(imgbuff);

    //�����жϷ�����
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    //����LPTMR���жϷ�����Ϊ PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      //����LPTMR���жϷ�����Ϊ PORTA_IRQHandler

    
    tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,100);   //�����
    tpm_pwm_duty(MOTOR_TPM, MOTOR2_PWM,start_speed);

    
    tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,100);   //�����
    tpm_pwm_duty(MOTOR_TPM, MOTOR4_PWM,start_speed);
    
    
    gpio_init (PTC0,  GPI , 0);          //������˿�
    gpio_init (PTC1,  GPI , 0); 
     
    port_init (PTC0,  ALT1 | PULLUP ); 
    port_init (PTC1,  ALT1 | PULLUP );
    
    gpio_init(PTB0,GPI,0);                       //���밴��
    gpio_init(PTB1,GPI,0);
    
    
    port_init (PTB0, ALT1 | PULLUP );
    port_init (PTB1, ALT1 | PULLUP );
           
    lptmr_timing_ms(100);                                  //��ʼ��LPTMR����ʱʱ��Ϊ0.1s
    set_vector_handler(LPTMR_VECTORn ,LPTMR_IRQHandler);    //����LPTMR���жϷ�����Ϊ LPTMR_IRQHandler
    enable_irq (LPTMR_IRQn);                                //ʹ��LPTMR�ж�
    
    DELAY_MS(200);
    while(gpio_get(PTB1)!=0)
    { 
      if(gpio_get(PTB0)==0)
      {
        DELAY_MS(10);
        if(gpio_get(PTB0)==0)
        {
          inin++;
          if(inin>=3)
            inin=0;
        }
        while(gpio_get(PTB0)==0);
      }
      switch(inin)
      {
      case 0: stop_time=80;led(LED0, LED_ON);led(LED1, LED_OFF);led(LED2, LED_OFF);break;
      case 1: stop_time=85;led(LED1, LED_ON);led(LED2, LED_OFF);led(LED0, LED_OFF);break;
      case 2: stop_time=90;led(LED2, LED_ON);led(LED0, LED_OFF);led(LED1, LED_OFF);break;
      default:led(LED0, LED_OFF);led(LED1, LED_OFF);led(LED2, LED_OFF);break;
      }
    }
    while(gpio_get(PTB1)!=0);
    DELAY_MS(10);
    while(gpio_get(PTB1)!=0);
    start_speed=10;
    while(1)
    {
        //��ȡͼ��
        camera_get_img();                                   //����ͷ��ȡͼ��

        //�๦�ܵ���������λ����ʾ����Ҫ���óɺڰ�ģʽ
        vcan_sendimg(imgbuff,CAMERA_SIZE);


        //��ѹͼ��  ���ѽ�ѹ�����ݷŵ� img �����
        img_extract(g_zzs_image,imgbuff,CAMERA_SIZE);
        
        //��ú�������      
         wall_left=gpio_get(PTC0);    
         wall_right=gpio_get(PTC1); 
         
        

         
         if(wall_right==0)    //�ж�ǽ��
        {
           led(LED1, LED_ON);
           control_motor(-18*18*15);//15
        }
 
          else if(wall_left==0)     //�ж�ǽ��
	{ 
          if(flag<=10)
          {
            led(LED2, LED_ON);
            control_motor(-40*40*15);   //30
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
			{
				start_speed=45;//45
				angle_set= 10;
			} 
            control_motor(turn());//���Ƶ��ת�� 
            led(LED1, LED_OFF);
	}         
         
         
         
         
         
         
         
         
         
         
         
    }
}

//����ͼ����λ����ʾ
//��ͬ����λ������ͬ������
//���ʹ��������λ��������Ҫ�޸Ĵ���
void vcan_sendimg(uint8 *imgaddr, uint32 imgsize)
{
    #define CMD_IMG  1
    uint8 cmdf[2] = {CMD_IMG, ~CMD_IMG};    //ɽ����λ�� ʹ�õ�����
    uint8 cmdr[2] = {~CMD_IMG, CMD_IMG};    //ɽ����λ�� ʹ�õ�����

    uart_putbuff(VCAN_PORT, cmdf, sizeof(cmdf));    //�ȷ�������

    uart_putbuff(VCAN_PORT, imgaddr, imgsize); //�ٷ���ͼ��

    uart_putbuff(VCAN_PORT, cmdr, sizeof(cmdr));    //�ȷ�������
}


/*!
 *  @brief      ��ֵ��ͼ���ѹ���ռ� �� ʱ�� ��ѹ��
 *  @param      dst             ͼ���ѹĿ�ĵ�ַ
 *  @param      src             ͼ���ѹԴ��ַ
 *  @param      srclen          ��ֵ��ͼ���ռ�ÿռ��С
 *  @since      v5.0            img_extract(img, imgbuff,CAMERA_SIZE);
 *  Sample usage:
 */
void img_extract(uint8 *dst, uint8 *src, uint32 srclen)
{
    uint8 colour[2] = {255, 0}; //0 �� 1 �ֱ��Ӧ����ɫ
    //ע��ɽ�������ͷ 0 ��ʾ ��ɫ��1��ʾ ��ɫ
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
 *  @brief      PORTA�жϷ�����
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n = 0;    //���ź�
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ

    n = 6;                                              //���ж�
    if(flag & (1 << n))                                 //PTA6�����ж�
    {
        camera_vsync();
    }
}

/*!
 *  @brief      DMA0�жϷ�����
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}