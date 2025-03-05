#ifndef _LED_H
#define _LED_H

#include "tx_api.h"

#include "stm32f4xx_hal.h"
// #include "gpio.h"

#include "includes.h"

#define LED_PRIO 15
#define LED_STACKSIZE 2048
extern TX_THREAD led_tcb;
extern UCHAR led_stack[LED_STACKSIZE];

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


void led_entry(ULONG thread_input);

#endif
