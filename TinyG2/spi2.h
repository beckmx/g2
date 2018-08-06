#ifndef _SPI_H
#define _SPI_H

#ifdef __cplusplus
extern "C" {
#endif

//#include "integer.h"

#define SPI2_MCK_DIV  (SystemCoreClock / 84)  // SPI clock divider to generate baud (based on 84MHz MCK)

// Function Prototypes
void spi2_init(void);
void spi2_test(void);

#ifdef __cplusplus
}
#endif

#endif
