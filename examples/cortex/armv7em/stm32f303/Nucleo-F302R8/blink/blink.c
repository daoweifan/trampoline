#include "tpl_os.h"
#include "stm32f30x.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_usart.h"

#define APP_Task_led_control_START_SEC_CODE
#include "tpl_memmap.h"

//init PB.13 as output (LED 1 on pin 13).
void initUserLed()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

//init PC.13 as output (User Button on pin 13).
void initUserButton()
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Enable the GPIO_BUTTON Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

  /* Enable the EXTI Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* Configure the GPIO_BUTTON pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Connect Button EXTI Line to Button GPIO Pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);

  /* Configure Button EXTI line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line13;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set Button EXTI Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure); 
}

//baudrate
enum {
  BAUD_300 = 300,
  BAUD_1200 = 1200,
  BAUD_2400 = 2400,
  BAUD_4800 = 4800,
  BAUD_9600 = 9600,
  BAUD_115200 = 115200,
  BAUD_230400 = 230400,
  BAUD_460800 = 460800,
  BAUD_921600 = 921600,
};

typedef struct {
  unsigned baud;
} uart_cfg_t;

#define UART_CFG_DEF { \
  .baud = BAUD_9600, \
}

#define uart USART2

void uart_Init(const uart_cfg_t *cfg)
{
  USART_ClockInitTypeDef USART_ClockInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef uartinfo;

  USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
  USART_ClockInitStructure.USART_CPHA = USART_CPHA_1Edge;
  USART_ClockInitStructure.USART_CPOL = USART_CPOL_High;
  USART_ClockInitStructure.USART_LastBit = USART_LastBit_Enable;
  USART_ClockInit(uart, &USART_ClockInitStructure);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  /*configure PA2<uart2.tx>, PA3<uart2.rx>*/
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_7);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  // GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  // GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /*init serial port*/
  uartinfo.USART_BaudRate = 9600;
  uartinfo.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  uartinfo.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  uartinfo.USART_Parity = USART_Parity_No;
  uartinfo.USART_StopBits = USART_StopBits_1;
  uartinfo.USART_WordLength = USART_WordLength_8b;
  uartinfo.USART_BaudRate = cfg->baud;
  USART_Init(uart, &uartinfo);

  NVIC_InitTypeDef NVIC_InitStructure;

#if 1
  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
  /* Enable the USART2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  // NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  USART_ClearFlag(uart, USART_FLAG_RXNE);
  USART_ITConfig(uart, USART_IT_RXNE, ENABLE);
#endif

#if 0
  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  /* Enable the USART2 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  USART_ClearFlag(uart, USART_FLAG_TC);
  USART_ITConfig(uart, USART_IT_TC, ENABLE);
#endif

  USART_Cmd(uart, ENABLE);
}

static int uart_putchar(int data)
{
  char c = (char) data;

  USART_SendData(uart, c);
  while(USART_GetFlagStatus(uart, USART_FLAG_TXE) == RESET);

  return 0;
}

static int uart_print(unsigned char * str, int num)
{
  int i;
  for (i=0; i<num; i++)
    uart_putchar(str[i]);

  return 0;
}

static int uart_IsNotEmpty(void)
{
  int ret;
  ret = (int) USART_GetFlagStatus(uart, USART_FLAG_RXNE);
  return ret;
}

static int uart_getch(void)
{
  int ret;
  
  /*wait for a char*/
  while(1) {
    ret = uart_IsNotEmpty();
    if(ret) break;
  }
  ret = USART_ReceiveData(uart);
  return ret;
}

FUNC(int, OS_APPL_CODE) main(void)
{
  uart_cfg_t cfg = UART_CFG_DEF;
  initUserLed();
  // initUserButton();
  uart_Init(&cfg);
  StartOS(OSDEFAULTAPPMODE);
  return 0;
}

DeclareResource(uart_resource);

typedef enum
{
  LED_STATE_OFF,
  LED_STATE_ON,
  LED_STATE_BLINK
} led_state_t;
led_state_t led_state;
VAR(int, AUTOMATIC) blink_period_ms;
VAR(int, AUTOMATIC) blink_timer;
VAR(int, AUTOMATIC) led_control_task_counter;

TASK(led_control)
{
  led_control_task_counter++;
  switch(led_state)
  {
    case LED_STATE_OFF:
      GPIOB->ODR &= ~GPIO_Pin_13;
      break;

    case LED_STATE_ON:
      GPIOB->ODR |= GPIO_Pin_13;
      break;

    case LED_STATE_BLINK:
      blink_period_ms = blink_period_ms?blink_period_ms:100;
      blink_timer ++;
      if (blink_timer >= blink_period_ms)
      {
        GPIOB->ODR ^= GPIO_Pin_13;  //toggle user led.
        blink_timer = 0;
      }
      break;
    default:
      break;
  }

  // ActivateTask(cmd_process);

  TerminateTask();
}

#define APP_Task_led_control_STOP_SEC_CODE
#include "tpl_memmap.h"

VAR(unsigned char, AUTOMATIC) cmd_buf[256];
VAR(unsigned char, AUTOMATIC) cmd_size;
VAR(unsigned char, AUTOMATIC) cmd_head;
VAR(unsigned char, AUTOMATIC) cmd_tail;

DeclareTask(cmd_process);

ISR (uart_rx)
{
  unsigned char ch;

  // USART_ClearFlag(uart, uint32_t USART_FLAG);

  GetResource(uart_resource);
  if (uart_IsNotEmpty())
  {
    ch = uart_getch();
    cmd_buf[cmd_tail++] = ch;
    cmd_size++;
    cmd_tail = (cmd_tail==0x100)?0:cmd_tail;
    uart_putchar(ch);
    if (ch == '\r')
    {
      uart_putchar('\n');
      uart_print("trampoline> ", sizeof("trampoline> "));
    }
  }

  ReleaseResource(uart_resource);

  ActivateTask(cmd_process);
}

FUNC (void, AUTOMATIC ) USART2_IRQ_ClearFlag(void)
{
  //do nothing
}

ISR(user_button)
{
  EXTI_ClearITPendingBit(EXTI_Line13);
  EXTI_ClearFlag(EXTI_Line13);
  GetResource(uart_resource);
  uart_print("Button Pressed\r\n", sizeof("Button Pressed\r\n"));
  uart_print("trampoline> ", sizeof("trampoline> "));
  ReleaseResource(uart_resource);

}


#define APP_Task_cmd_process_START_SEC_CODE
#include "tpl_memmap.h"

VAR(unsigned char, AUTOMATIC) cmd_seg1[16];
VAR(unsigned char, AUTOMATIC) cmd_seg1_index;
VAR(unsigned char, AUTOMATIC) cmd_seg2[16];
VAR(unsigned char, AUTOMATIC) cmd_seg2_index;
VAR(unsigned char, AUTOMATIC) cmd_seg3[16];
VAR(unsigned char, AUTOMATIC) cmd_seg3_index;
VAR(unsigned char, AUTOMATIC) cmd_seg4[16];
VAR(unsigned char, AUTOMATIC) cmd_seg4_index;
VAR(unsigned char, AUTOMATIC) cmd_null_number;

FUNC(int, OS_APPL_CODE) str_cmp(P2VAR(unsigned char, AUTOMATIC, AUTOMATIC)src, P2VAR(unsigned char, AUTOMATIC, AUTOMATIC)dest, int size)
{
  int result = 0;
  for (int i=0; i < size; i++)
  {
    if(src[i] != dest[i])
    {
      result = 1;
      break;
    }
  }
  return result;
}

FUNC(int, OS_APPL_CODE) chnum(char str[])
{
  int i, n, num = 0;
  for (i=0;str[i]!='\0';i++)
    if(str[i]>='0'&&str[i]<='9')
      num = num*10+str[i]-'0';
  return (num);
}

VAR(int, AUTOMATIC) cmd_process_task_counter;

TASK(cmd_process)
{
  unsigned ch;

  cmd_process_task_counter++;

  if (cmd_head != cmd_tail)
  {
    ch = cmd_buf[cmd_head++];
    cmd_size--;
    cmd_head = (cmd_head==0x100)?0:cmd_head;

    if (ch == '\r')
    {
      if ((cmd_seg1_index == 3) && (str_cmp(cmd_seg1, "led", 3) == 0))
      {
        if ((cmd_seg2_index == 2) && (str_cmp(cmd_seg2, "on", 2) == 0))
        {
          led_state = LED_STATE_ON;
        }
        else if ((cmd_seg2_index == 3) && (str_cmp(cmd_seg2, "off", 3) == 0))
        {
          led_state = LED_STATE_OFF;
        }
        else if ((cmd_seg2_index == 5) && (str_cmp(cmd_seg2, "blink", 5) == 0))
        {
          if (cmd_seg3_index != 0)
          {
            cmd_seg3[cmd_seg3_index] = '\0';
            blink_period_ms = chnum(cmd_seg3);
            led_state = LED_STATE_BLINK;
          }
        }
      }

      cmd_seg1_index = 0;
      cmd_seg2_index = 0;
      cmd_seg3_index = 0;
      cmd_seg4_index = 0;
      cmd_null_number = 0;
    }
    else
    {
      switch (cmd_null_number)
      {
        case 0:
          if (ch != ' ')
            cmd_seg1[cmd_seg1_index++] = ch;
          else
            cmd_null_number = 1;
          break;
        case 1:
          if (ch != ' ')
            cmd_seg2[cmd_seg2_index++] = ch;
          else
            cmd_null_number = 2;
          break;
        case 2:
          if (ch != ' ')
            cmd_seg3[cmd_seg3_index++] = ch;
          else
            cmd_null_number = 3;
          break;
        case 3:
          if (ch != ' ')
            cmd_seg4[cmd_seg4_index++] = ch;
          break;
        default:
          break;
      }
    }
  }

  TerminateTask();
}

#define APP_Task_cmd_process_STOP_SEC_CODE
#include "tpl_memmap.h"

#define OS_START_SEC_CODE
#include "tpl_memmap.h"
/*
 *  * This is necessary for ST libraries
 *   */
FUNC(void, OS_CODE) assert_failed(uint8_t* file, uint32_t line)
{
}
#define OS_STOP_SEC_CODE
#include "tpl_memmap.h"

