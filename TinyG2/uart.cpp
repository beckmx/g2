#include "uart.h"
#include "tinyg2.h"
#include "config.h"
#include "hardware.h"

int16_t TinyUSART::readByte()
{
	if (USART1->US_CSR & US_CSR_RXRDY) {
	    return (USART1->US_RHR & US_RHR_RXCHR_Msk);
	}
	return -2;
}

int16_t TinyUSART::writeByte(const char value)
{
	if (USART1->US_CSR & US_CSR_TXRDY) {
	    USART1->US_THR = US_THR_TXCHR(value);
	    return value;
	}
	return -1;
}

void TinyUSART::flush()
{
	while (!(USART1->US_CSR & US_CSR_TXEMPTY));
}

bool TinyUSART::txEmpty()
{
	return (USART1->US_CSR & US_CSR_TXEMPTY);
}

void TinyUSART::init()
{
	enableClock();

    uart1_rxd_pin.setMode(kInput);
    uart1_txd_pin.setMode(kPeripheralA);

	// Reset and disable TX and RX
    USART1->US_CR = US_CR_RSTRX | US_CR_RSTTX | US_CR_RXDIS | US_CR_TXDIS;

    // Disable interrupts
    USART1->US_IDR = US_IDR_TXRDY | US_IDR_RXRDY;

    // reset PCR to zero
    USART1->US_IDR = 0xffffffff; // disable all the things

    uint32_t mode = US_MR_USART_MODE_NORMAL | US_MR_CHMODE_NORMAL | US_MR_CHRL_8_BIT |
                    US_MR_NBSTOP_1_BIT | US_MR_PAR_NO | US_MR_USCLKS_MCK;
    USART1->US_MR = mode;

    // set baud
    uint32_t baud = 115200;
    USART1->US_BRGR = US_BRGR_CD((SystemCoreClock / baud / 16)) | US_BRGR_FP(0);

#if 0
    //todo; DMA reset

    //NVIC_SetPriority(USART1_IRQn, 11);

    //TODO: set DMA transfer intterupts?

    //TODO: set interrupts?

                if (interrupts & UARTInterrupt::OnRxDone) {
                    USART1->US_IER = US_IER_RXRDY;
                } else {
                    USART1->US_IDR = US_IDR_RXRDY;
                }
                if (interrupts & UARTInterrupt::OnTxDone) {
                    USART1->US_IER = US_IER_TXRDY;
                } else {
                    USART1->US_IDR = US_IDR_TXRDY;
                }

    //NVIC_EnableIRQ(USART1_IRQn);
#endif

	enable();
}

void TinyUSART::enable()
{
	USART1->US_CR = US_CR_TXEN | US_CR_RXEN;
}

void TinyUSART::disable()
{
	USART1->US_CR = US_CR_TXDIS | US_CR_RXDIS;
}

void TinyUSART::enableClock()
{
	uint32_t id_mask = 1 << USART1_IRQn; //peripheral id
	if ((PMC->PMC_PCSR0 & id_mask) != id_mask)
	{
        PMC->PMC_PCER0 = id_mask;
    }

#if 0 //May need to try:
            if ((PMC->PMC_PCSR1 & id_mask) != id_mask) {
                PMC->PMC_PCER1 = id_mask;
            }
#endif
}
