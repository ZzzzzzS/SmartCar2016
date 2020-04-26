

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


uint8 imgbuff[CAMERA_SIZE];                             //����洢����ͼ�������
//uint8 img[CAMERA_W*CAMERA_H];                           //����ӥ������ͷ��һ�ֽ�8�����أ������Ҫ��ѹΪ 1�ֽ�1�����أ����㴦��
uint8 g_zzs_image[CAMERA_W*CAMERA_H];                              //ת������ά����

unsigned char cross=0;

//��������
void vcan_sendimg(uint8 *imgaddr, uint32 imgsize);
void img_extract(uint8 *dst, uint8 *src, uint32 srclen);
void PORTA_IRQHandler();
void DMA0_IRQHandler();


//{50,50,50,40,40,40,40,40,40,40,40,35,30,30,30,25,25,25,25,10,10,10,10,10,5,5,5,1,1,0};
char weight[30]={0,1,1,5,5,5,10,10,10,10,10,25,25,25,25,30,30,30,35,35,35,40,40,40,40,40,40,50,50,50};

int turn(void)
{
  int row=0,col=0,mark_col=0;
  int mark_num=0,center[50]={0},flag=0;
  
  
  for(row=10*80;row<60*80;row+=80)
  {
    for(col=10;col<70;col++)//�е�Ϊ40
    {
      if(g_zzs_image[row+col]!=0)
      {
        mark_col+=col-40;
        mark_num++;
      }
    }
     
      
      
      
      if(mark_num!=0)
        center[row/80-10]=mark_col/mark_num;
      else if(mark_num==0)
        center[row/80-10]=0;
      
      
      
      if(mark_num>40)        //ʮ��
      {
        flag++;
      }   
      
      
      
      mark_col=0;
      mark_num=0;      
  }
  
  
  
  
  mark_col=0;              //�ٴ�����
  mark_num=0;
  
  
  
  if(flag>3)              //ʮ��
    cross++;
  
  
  
  
  for(row=10;row<50;row++)
  {
    if(center[row]==0)
      continue;
    else if(center[row]!=0)
    {
      mark_col+=center[row];
      mark_num++;
    }
    if(mark_num>=5)
      break;
  }
        mark_col/=5;

        if(mark_col>5)
        {
          led(LED3, LED_ON);                  
          led(LED0, LED_OFF); 
        }
        if(mark_col<-5)
        {
          led(LED0, LED_ON);                  
          led(LED3, LED_OFF);
        }
        if(mark_col>=-5 && mark_col<=5)
        {      
          led(LED3, LED_ON);                  
          led(LED0, LED_ON);
        }
        
        if(mark_col>0)
          return weight[mark_col];
        else if(mark_col<0)
          return -weight[-mark_col];
        else if(mark_col==0) 
          return 0;
}


void control_motor(int turn)
{   

    tpm_pwm_duty(MOTOR_TPM, MOTOR2_PWM,start_left);
    tpm_pwm_duty(MOTOR_TPM, MOTOR4_PWM,start_right);
    if(turn>0)
    {
      tpm_pwm_duty(MOTOR_TPM, MOTOR1_PWM,100 - turn);
      tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,100);
    }
    else if(turn<0)
    {
      tpm_pwm_duty(MOTOR_TPM, MOTOR3_PWM,100 + turn);
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