#include "led.h"

TX_THREAD led_tcb;
UCHAR led_stack[LED_STACKSIZE];

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

  led_init();

  /* Wait for event flag 0.  */
  status =  tx_event_flags_get(&event_flags_led, IMU_INIT_ERROR | IMU_INIT_SUCCEED , TX_OR_CLEAR, 
                                          &actual_flags, TX_WAIT_FOREVER);
  /* Check status.  */
  if(actual_flags == IMU_INIT_SUCCEED)
    led_set(number_1 | number_4, green);
  else if(actual_flags == IMU_INIT_ERROR)
    led_set(number_1 | number_4, red);

  while (1)
  {
    /* TODO: led display uav status */
    tx_thread_sleep(1000);
  }
}
