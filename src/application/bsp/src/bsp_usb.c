#include "bsp_usb.h"



void VirtualComPort_Printf(const char *format, ...)
{
	va_list args;
	uint32_t length;

	va_start(args, format);
	length = vsnprintf((char *)UserTxBufferFS, APP_TX_DATA_SIZE, format, args);
	va_end(args);

	CDC_Transmit_FS(UserTxBufferFS, length, 10);
}

uint8_t VirtualComPort_Sned(uint8_t *Buf, uint16_t Len)
{
	return CDC_Transmit_FS(Buf, Len, 10);
}
