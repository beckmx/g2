#ifndef _SPI_H
#define _SPI_H

#ifdef __cplusplus
extern "C" {
#endif

//#include "integer.h"

// General Definitions
#define SPI2_MCK_DIV  (SystemCoreClock / 84)  // SPI clock divider to generate baud (based on 84MHz MCK)

// Direction (RnW) Bit
#define SPI2_WRITE 0x00
#define SPI2_READ  0x01

// SPI2 Command Set
#define SPI2_CMD_RST_ENC_POS    0x00  // Reset Encoder Positions to Zero
#define SPI2_CMD_START_TOOL_TIP 0x01  // Start Tool Tip Command
#define SPI2_CMD_SND_MTR_POS    0x02  // Send Motor Positions
#define SPI2_CMD_REQ_ENC_POS    0x03  // Request Encoder Positions

#define SPI2_CMD_RD_ENC_POS     0x40  // Read Encoder Position
#define SPI2_CMD_SET_USER_IO    0x41  // Set User IO
#define SPI2_CMD_CLR_USER_IO    0x42  // Clear User IO
#define SPI2_CMD_RD_USER_IO     0x43  // Read User IO
#define SPI2_CMD_SET_USER_LED   0x44  // Set User LED
#define SPI2_CMD_CLR_USER_LED   0x45  // Clear User LED
#define SPI2_CMD_RD_ITR_LOOP    0x46  // Read Interlock Loop
#define SPI2_CMD_SET_SPIN_LED   0x47  // Set Spindle LEDs
#define SPI2_CMD_SET_EPS        0x48  // Set Epsilon
#define SPI2_CMD_RD_ESC_CURR    0x49  // Read ESC Current
#define SPI2_CMD_FW_VER         0x4A  // Firmware Version

#define SPI2_CMD_NULL           0xFF  // NULL command (placeholder for slave requests)

#define SPI2_STS_OK             0x00  // OK Status
#define SPI2_STS_ERR            0x01  // Error Status
#define SPI2_STS_HALT           0x02  // Halt

// Function Prototypes
void spi2_init(void);
uint8_t spi2_cmd(bool, uint8_t, uint8_t, uint8_t*, uint16_t);
uint8_t spi2_slave_handler(void);
void spi2_test(void);

#ifdef __cplusplus
}
#endif

#endif
