#include "bsp_usb.h"

uint8_t VirtualComPort_Sned(uint8_t* Buf, uint16_t Len)
{
	return CDC_Transmit_FS(Buf, Len);
}
