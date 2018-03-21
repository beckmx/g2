/**
* USART1 = 0x4009C000
* DMA CHannel 13 - TX, 14 - RX
* Peripheral ID 18 (interrupt)
* CMSIS/Device/ATMEL/sam3xa/include/pio/pio_sam3x8c.h
* CMSIS/Device/ATMEL/sam3xa/include/sam3x8c.h
* CMSIS/Device/ATMEL/sam3xa/include/instance/instance_usart1.h
* ./TinyG2/CMSIS/Device/ATMEL/sam3xa/include/component/component_usart.h
* #define USART1     (0x4009C000U)
* #define PDC_USART1 (0x4009C100U)
*/

#ifndef TINY_UART_H_
#define TINY_UART_H_

#include "sam3xa.h"

class TinyUSART
{
public:
	TinyUSART()
	{ }

	~TinyUSART() { }

	void init();
	void enable();
	void disable();
	int16_t readByte();
	int16_t writeByte(const char value);
	void flush();
	bool txEmpty();

private:
	void enableClock();
};

#endif //TINY_UART_H_
