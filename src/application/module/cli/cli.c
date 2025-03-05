#include "cli.h"
#include "usbd_cdc_if.h"
#include "string.h"
#include "stdbool.h"

TX_THREAD cli_tcb;
UCHAR cli_stack[CLI_STACKSIZE];

void cli_entry(ULONG thread_input)
{
  uint8_t cli_data[24] = {0};
  uint32_t cli_len = 0;
  bool top = false;

  EXECUTION_TIME TotalTime, IdleTime, _thread_time, _isr_time, _idle_time, Delta_TotalTime, Delta_IdleTime;
  UINT           uiCount = 0;
  double         CpuUsage = 0;
  TX_THREAD *    p_tcb = &cli_tcb;
  // UINT           thread_numbers = 0;
  EXECUTION_TIME ThreadTime;
  double         ThreadUsage, ThreadCpuUsage;
  float          ThreadStackUsage;
  char           thread_state[20];

  HAL_ResumeTick();

  _tx_execution_thread_total_time_get(&_thread_time);
  _tx_execution_isr_time_get(&_isr_time);
  _tx_execution_idle_time_get(&_idle_time);
  IdleTime = _idle_time;
  TotalTime = _thread_time + _isr_time + _idle_time;

  while (1)
  {
    USBD_Interface_fops_FS.Receive(cli_data, &cli_len);

    if(strcmp((char *)&cli_data, "top") == 0)
    {
      top = true;
    }

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

      /* Check thread numbers */
      // while(p_tcb != TX_NULL)
      // {
      //   thread_numbers++;
      //   p_tcb = p_tcb->tx_thread_created_next;
      //   if(p_tcb == &cli_tcb)
      //     break;
      // }
    }

    if(top)
    {
      VirtualComPort_Printf("\r\n====================================================================================================================================================\r\n");
      VirtualComPort_Printf("| Prio |      ThreadName      | ThreadState | StkSize   StartAddr      EndAddr   CurStack  MaxStack  StackUsage | CPUUsage  Total(%5.2f%%) RunCount |\r\n", CpuUsage);

      for(int i = 0; i < TX_MAX_PRIORITIES; i++)
      {
        while(p_tcb != TX_NULL)
        {
          if(p_tcb->tx_thread_priority == i)
          {
            /* Get thread info */
            _tx_execution_thread_time_get(p_tcb, &ThreadTime);
            ThreadUsage = (double)ThreadTime / _thread_time * 100;
            ThreadCpuUsage = (double)ThreadTime / TotalTime * 100;
            ThreadStackUsage = (float)((int)p_tcb->tx_thread_stack_end - (int)p_tcb->tx_thread_stack_highest_ptr) / (float)p_tcb->tx_thread_stack_size;

            switch ((int)p_tcb->tx_thread_state)
            {
              case TX_READY:
                  strcpy(thread_state, "RUNNING");
                  break;
              case TX_COMPLETED:
                  strcpy(thread_state, "COMPLETED");
                  break;
              case TX_TERMINATED:
                  strcpy(thread_state, "TERMINATED");
                  break;
              case TX_SUSPENDED:
                  strcpy(thread_state, "SUSPEND");
                  break;
              case TX_SLEEP:
                  strcpy(thread_state, "SLEEP");
                  break;
              case TX_QUEUE_SUSP:
                  strcpy(thread_state, "WAIT QUEUE");
                  break;
              case TX_SEMAPHORE_SUSP:
                  strcpy(thread_state, "WAIT SEM");
                  break;
              case TX_EVENT_FLAG:
                  strcpy(thread_state, "WAIT EVENT");
                  break;
              case TX_BLOCK_MEMORY:
                  strcpy(thread_state, "WAIT BLOCK");
                  break;
              case TX_BYTE_MEMORY:
                  strcpy(thread_state, "WAIT BYTE");
                  break;
              case TX_MUTEX_SUSP:
                  strcpy(thread_state, "WAIT MUTEX");
                  break;
              default:
                  strcpy(thread_state, "");
                  break;
            }
            VirtualComPort_Printf("| %3d  | %20s | %10s  | %6lu  [0x%08x]  [0x%08x]  %5d    %6d      %4.1f%%    |  %4.1f%%      %7.4f%%    %8lu |\r\n", 
              p_tcb->tx_thread_priority,
              p_tcb->tx_thread_name,
              thread_state,
              p_tcb->tx_thread_stack_size, (int)p_tcb->tx_thread_stack_start, (int)p_tcb->tx_thread_stack_end,
              (int)p_tcb->tx_thread_stack_end - (int)p_tcb->tx_thread_stack_ptr,
              (uint32_t)p_tcb->tx_thread_stack_end - (uint32_t)p_tcb->tx_thread_stack_highest_ptr,
              ThreadStackUsage * 100.0f,
              ThreadUsage,
              ThreadCpuUsage,
              p_tcb->tx_thread_run_count);
          }
          p_tcb = p_tcb->tx_thread_created_next;
          if(p_tcb == &cli_tcb)
          break;
        }
      }

      top = false;
    }

    memset(cli_data, 0, sizeof(cli_data));
    tx_thread_sleep(1);
  }
}
