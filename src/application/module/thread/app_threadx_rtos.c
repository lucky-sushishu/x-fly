/* This is a small demo of the high-performance ThreadX kernel.  It includes examples of eight
   threads of different priorities, using a message queue, semaphore, mutex, event flags group,
   byte pool, and block pool.  */

#include "tx_api.h"
#include "gpio.h"
#include "imu.h"
#include "communication.h"

#define LED_PRIO 15
#define LED_STACKSIZE 2048
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
  UINT           status;
  ULONG          actual_flags;
  EXECUTION_TIME TotalTime, IdleTime, _thread_time, _isr_time, _idle_time, Delta_TotalTime, Delta_IdleTime;
  UINT           uiCount = 0;
  double         CpuUsage = 0;
  TX_THREAD *    p_tcb = &led_tcb;
  UINT           thread_numbers = 0;
  EXECUTION_TIME ThreadTime;
  double         ThreadUsage, ThreadCpuUsage;
  float          ThreadStackUsage;
  char           thread_state[20];

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

    // /* Compute CPU usage */
    // _tx_execution_thread_total_time_get(&_thread_time);
    // _tx_execution_isr_time_get(&_isr_time);
    // _tx_execution_idle_time_get(&_idle_time);

    // if(++uiCount == 200)
    // {
    //   uiCount = 0;
      
    //   Delta_IdleTime = _idle_time - IdleTime;
    //   Delta_TotalTime = _thread_time + _isr_time + _idle_time - TotalTime;

    //   CpuUsage = (double)Delta_IdleTime / Delta_TotalTime;
    //   CpuUsage = 100 - CpuUsage * 100;

    //   IdleTime = _idle_time;
    //   TotalTime = _thread_time + _isr_time + _idle_time;
    //   printf("\r\n====================================================================================================================================================\r\n");

    //   /* Check thread numbers */
    //   while(p_tcb != TX_NULL)
    //   {
    //     thread_numbers++;
    //     p_tcb = p_tcb->tx_thread_created_next;
    //     if(p_tcb == &led_tcb)
    //       break;
    //   }
    //   printf("| Prio |      ThreadName      | ThreadState | StkSize   StartAddr      EndAddr   CurStack  MaxStack  StackUsage | CPUUsage  Total(%5.2f%%) RunCount |\r\n", CpuUsage);
    //   for(int i = 0; i < TX_MAX_PRIORITIES; i++)
    //   {
    //     while(p_tcb != TX_NULL)
    //     {
    //       if(p_tcb->tx_thread_priority == i)
    //       {
    //         /* Get thread info */
    //         _tx_execution_thread_time_get(p_tcb, &ThreadTime);
    //         ThreadUsage = (double)ThreadTime / _thread_time * 100;
    //         ThreadCpuUsage = (double)ThreadTime / TotalTime * 100;
    //         ThreadStackUsage = (float)((int)p_tcb->tx_thread_stack_end - (int)p_tcb->tx_thread_stack_highest_ptr) / (float)p_tcb->tx_thread_stack_size;
            
    //         switch ((int)p_tcb->tx_thread_state)
    //         {
    //           case TX_READY:
    //               strcpy(thread_state, "RUNNING");
    //               break;
    //           case TX_COMPLETED:
    //               strcpy(thread_state, "COMPLETED");
    //               break;
    //           case TX_TERMINATED:
    //               strcpy(thread_state, "TERMINATED");
    //               break;
    //           case TX_SUSPENDED:
    //               strcpy(thread_state, "SUSPEND");
    //               break;
    //           case TX_SLEEP:
    //               strcpy(thread_state, "SLEEP");
    //               break;
    //           case TX_QUEUE_SUSP:
    //               strcpy(thread_state, "WAIT QUEUE");
    //               break;
    //           case TX_SEMAPHORE_SUSP:
    //               strcpy(thread_state, "WAIT SEM");
    //               break;
    //           case TX_EVENT_FLAG:
    //               strcpy(thread_state, "WAIT EVENT");
    //               break;
    //           case TX_BLOCK_MEMORY:
    //               strcpy(thread_state, "WAIT BLOCK");
    //               break;
    //           case TX_BYTE_MEMORY:
    //               strcpy(thread_state, "WAIT BYTE");
    //               break;
    //           case TX_MUTEX_SUSP:
    //               strcpy(thread_state, "WAIT MUTEX");
    //               break;
    //           default:
    //               strcpy(thread_state, "");
    //               break;
    //         }

    //         // printf("| %3d  | %20s | %10s  | %6lu  [0x%08x]  [0x%08x]  %5d    %6d      %4.1f%%    |  %4.1f%%      %7.4f%%    %8lu |\r\n", 
    //         //   p_tcb->tx_thread_priority,
    //         //   p_tcb->tx_thread_name,
    //         //   thread_state,
    //         //   p_tcb->tx_thread_stack_size, (int)p_tcb->tx_thread_stack_start, (int)p_tcb->tx_thread_stack_end,
    //         //   (int)p_tcb->tx_thread_stack_end - (int)p_tcb->tx_thread_stack_ptr,
    //         //   (uint32_t)p_tcb->tx_thread_stack_end - (uint32_t)p_tcb->tx_thread_stack_highest_ptr,
    //         //   ThreadStackUsage * 100.0f,
    //         //   ThreadUsage,
    //         //   ThreadCpuUsage,
    //         //   p_tcb->tx_thread_run_count);
    //       }
    //       p_tcb = p_tcb->tx_thread_created_next;
    //       if(p_tcb == &led_tcb)
    //       break;
    //     }
        
    //   }
    // }
    tx_thread_sleep(1000);
    printf("1");
  }
}

void tx_application_define(void *first_unused_memory)
{
  tx_thread_create(&led_tcb, "led", led_entry, 0,
                                &led_stack[0], LED_STACKSIZE,
                                LED_PRIO, LED_PRIO, TX_NO_TIME_SLICE, TX_AUTO_START);

//   tx_thread_create(&imu_mag_tcb, "sensor", imu_mag_entry, 0,
//                                 &imu_mag_stack[0], IMU_MAG_STACKSIZE,
//                                 IMU_MAG_PRIO, IMU_MAG_PRIO, TX_NO_TIME_SLICE, TX_AUTO_START);

//   tx_thread_create(&communication_tcb, "communication", communication_entry, 0,
//                                 &communication_stack[0], COMMUNICATION_STACKSIZE,
//                                 COMMUNICATION_PRIO, COMMUNICATION_PRIO, TX_NO_TIME_SLICE, TX_AUTO_START);

//   /* Create the comm message queue shared by imu_mag and communication.  */
//   tx_queue_create(&queue_comm, "comm queue", (sizeof(communication_data_t) / 4)*TX_1_ULONG, queue_imu_area, (sizeof(communication_data_t) / 4)*sizeof(float)*IMU_QUEUE_SIZE);

//   /* Create the event flags group used by threads sensor and led.  */
//   tx_event_flags_create(&event_flags_led, "led event flags ");

//   /* Create the semaphore used by sensor and i2c dma rx transfer completed callback. */
//   tx_semaphore_create(&semaphore_imu, "imu semaphore", 1);
}
