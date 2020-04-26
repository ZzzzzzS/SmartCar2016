

#include "common.h"
#include "include.h"



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


#define start_left 63
#define start_right 60
#define angle_set 5


#define KL 1.7
#define KH 0.1



uint8 imgbuff[CAMERA_SIZE];                             //����洢����ͼ�������
//uint8 img[CAMERA_W*CAMERA_H];                           //����ӥ������ͷ��һ�ֽ�8�����أ������Ҫ��ѹΪ 1�ֽ�1�����أ����㴦��

uint8 g_zzs_image[CAMERA_W*CAMERA_H];                              //ת������ά����

//��������
void vcan_sendimg(uint8 *imgaddr, uint32 imgsize);
void img_extract(uint8 *dst, uint8 *src, uint32 srclen);
void PORTA_IRQHandler();
void DMA0_IRQHandler();





int turn(void)
{
	int zzs_i =2400, zzs_j;
	int zzs_number=0;
        int zzs_weight=0;
	for(zzs_i=20*80;zzs_i<4800;zzs_i+=80)
		for(zzs_j=10;zzs_j<70;zzs_j+=1)
		{
			if(g_zzs_image[zzs_i+zzs_j]!=0)
			{
			                                              //iΪ������ jΪ������
                          
                          
                          zzs_weight+=((zzs_j-40)*KL)*((zzs_i/80)*KH);
                          /*
                          if(zzs_j-40<0)
                            zzs_weight-=((zzs_j-40)*(zzs_j-40)*KL)*((zzs_i/80)*KH);
                          else if(zzs_j-40>=0)
                            zzs_weight+=((zzs_j-40)*(zzs_j-40)*KL)*((zzs_i/80)*KH);
                          */
                          zzs_number++;

			}   
		}
        zzs_number=zzs_weight/zzs_number;
        if(zzs_number>5)
        {
          led(LED0, LED_ON);                  
          led(LED3, LED_OFF); 
        }
        if(zzs_number<-5)
        {
          led(LED3, LED_ON);                  
          led(LED0, LED_OFF);
        }
        if(zzs_number>=-5 && zzs_number<=5)
        {      
          led(LED3, LED_ON);                  
          led(LED0, LED_ON);
        }
        return zzs_number;
}


void control_motor(int turn)
{   
  
    tpm_pwm_duty(MOTOR_TPM, MOTOR2_PWM,start_left);
    tpm_pwm_duty(MOTOR_TPM, MOTOR4_PWM,start_right);
    
    
    
    if(turn>=100-start_left||turn>=100-start_right)
      turn=100-(start_left+start_right)/2;
    
    if(turn)
    

    if(turn>0)
    {
      tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,100 - turn);       //��
      tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,100);   
    }
    else if(turn<0)
    {
      tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,100 + turn);       //��
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
  
  
  led_init(LED0);                         //��ʼ��LED0
  led_init(LED3);
  
  
  

  //int zzs_i,zzs_j;
  //int zzs_turn;
  
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
  uart_init(UART0,115200);
    
    //��ʼ������ͷ
    camera_init(imgbuff);

    //�����жϷ�����
    set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);    //����LPTMR���жϷ�����Ϊ PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);      //����LPTMR���жϷ�����Ϊ PORTA_IRQHandler

    
    tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,100);   //�����
    tpm_pwm_duty(MOTOR_TPM, MOTOR2_PWM,start_left);

    
    tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,100);   //�����
    tpm_pwm_duty(MOTOR_TPM, MOTOR4_PWM,start_right);
    
    while(1)
    {
        //��ȡͼ��
        camera_get_img();                                   //����ͷ��ȡͼ��

        //�๦�ܵ���������λ����ʾ����Ҫ���óɺڰ�ģʽ
        vcan_sendimg(imgbuff,CAMERA_SIZE);


        //��ѹͼ��  ���ѽ�ѹ�����ݷŵ� img �����
        img_extract(g_zzs_image,imgbuff,CAMERA_SIZE);
              
   
        
        
        //���ת��Ƕ�
        
        control_motor(turn());//���Ƶ��ת��
        
  
                                        
        

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