/* This is a small demo of the high-performance ThreadX kernel.  It includes examples of eight
   threads of different priorities, using a message queue, semaphore, mutex, event flags group,
   byte pool, and block pool.  */

#include "tx_api.h"
#include "gpio.h"
#include "imu.h"
#include "communication.h"
// #include "includes.h"

#define LED_PRIO 15
#define LED_STACKSIZE 102400
static TX_THREAD led_tcb;
static UCHAR led_stack[LED_STACKSIZE];

typedef enum {
  number_1 = 0x01,
  number_2 = 0x02,
  number_3 = 0x04,
  number_4 = 0x08,
  all_led = 0x0F,
} led_num_t;

typedef enum {
  none,
  red,
  green,
  blue,
} led_color_t;

void led_set(led_num_t num, led_color_t color)
{
  for(int i = 1; i < all_led; (i = i << 1))
  {
    if(num & i)
    {
      switch(i)
      {
        case number_1:
                      switch(color)
                      {
                        case red: HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET); break;
                        case green: HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); break;
                        case none: HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET); break;
                        default: break;
                      }
                      break;
        case number_2:
                      break;
        case number_3:
                      if(color == blue)
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
                      else
                        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
                      break;
        case number_4:
                      switch(color)
                      {
                        case red: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); break;
                        case green: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); break;
                        case none: HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); break;
                        default: break;
                      }
                      break;
        default:      break;
      }
    }
  }
}

void led_init(void)
{
  /* led init none */
  led_set(all_led, none);

  /* led3 init blue */
  led_set(number_3, blue);
}

void led_entry(ULONG thread_input)
{
  UINT status;
  ULONG actual_flags;
  EXECUTION_TIME TotalTime, IdleTime, _thread_time, _isr_time, _idle_time, Delta_TotalTime, Delta_IdleTime;
  UINT uiCount = 0;
  double CpuUsage = 0;

  led_init();

  /* Wait for event flag 0.  */
  status =  tx_event_flags_get(&event_flags_led, IMU_INIT_ERROR | IMU_INIT_SUCCEED , TX_OR_CLEAR, 
                                          &actual_flags, TX_WAIT_FOREVER);
  /* Check status.  */
  if(actual_flags == IMU_INIT_SUCCEED)
    led_set(number_1 | number_4, green);
  else if(actual_flags == IMU_INIT_ERROR)
    led_set(number_1 | number_4, red);

  HAL_ResumeTick();

  _tx_execution_thread_total_time_get(&_thread_time);
  _tx_execution_isr_time_get(&_isr_time);
  _tx_execution_idle_time_get(&_idle_time);
  IdleTime = _idle_time;
  TotalTime = _thread_time + _isr_time + _idle_time;

  while (1)
  {
    /* TODO: led display uav status */

    /* Compute CPU usage */
    _tx_execution_thread_total_time_get(&_thread_time);
    _tx_execution_isr_time_get(&_isr_time);
    _tx_execution_idle_time_get(&_idle_time);

    if(++uiCount == 200)
    {
      uiCount = 0;
      
      Delta_IdleTime = _idle_time - IdleTime;
      Delta_TotalTime = _thread_time + _isr_time + _idle_time - TotalTime;

      CpuUsage = (double)Delta_IdleTime / Delta_TotalTime;
      CpuUsage = 100 - CpuUsage * 100;

      IdleTime = _idle_time;
      TotalTime = _thread_time + _isr_time + _idle_time;
      printf("%f\n", CpuUsage);
    }
    tx_thread_sleep(1);
  }
}

void tx_application_define(void *first_unused_memory)
{
  tx_thread_create(&led_tcb, "led", led_entry, 0,
                                &led_stack[0], LED_STACKSIZE,
                                LED_PRIO, LED_PRIO, TX_NO_TIME_SLICE, TX_AUTO_START);

  tx_thread_create(&imu_mag_tcb, "sensor", imu_mag_entry, 0,
                                &imu_mag_stack[0], IMU_MAG_STACKSIZE,
                                IMU_MAG_PRIO, IMU_MAG_PRIO, TX_NO_TIME_SLICE, TX_AUTO_START);

  tx_thread_create(&communication_tcb, "communication", communication_entry, 0,
                                &communication_stack[0], COMMUNICATION_STACKSIZE,
                                COMMUNICATION_PRIO, COMMUNICATION_PRIO, TX_NO_TIME_SLICE, TX_AUTO_START);

  /* Create the comm message queue shared by imu_mag and communication.  */
  tx_queue_create(&queue_comm, "comm queue", (sizeof(communication_data_t) / 4)*TX_1_ULONG, queue_imu_area, (sizeof(communication_data_t) / 4)*sizeof(float)*IMU_QUEUE_SIZE);

  /* Create the event flags group used by threads sensor and led.  */
  tx_event_flags_create(&event_flags_led, "led event flags ");

  /* Create the semaphore used by sensor and i2c dma rx transfer completed callback. */
  tx_semaphore_create(&semaphore_imu, "imu semaphore", 1);
}
