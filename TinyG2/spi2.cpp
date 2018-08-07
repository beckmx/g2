#include "tinyg2.h"					// #1 There are some dependencies
#include "config.h"					// #2
#include "hardware.h"
#include "spi2.h"
#include "motate/utility/SamSPI.h"
#include "motate/MotateTimers.h"

#include <stdio.h>
#include <memory>

// Create a SPI instance using CS1 pin for select
std::shared_ptr<Motate::SPI<kSocket2_SPISlaveSelectPinNumber>> spi2;

// spi2_init: Initializes the SPI2 subsystem
void spi2_init() {

  // Pin direction, type, debounce set in hardware.h

  // Set up interrupt on falling edge of SPI_CS2 pin
  spi2_int_pin.setInterrupts(kPinInterruptOnFallingEdge);


}

// spi2_cmd: Start a SPI master transfer (read/write) using the provided buffer
uint8_t spi2_cmd(bool slave_req, uint8_t rnw, uint8_t cmd, uint8_t *data_buf, uint16_t num_data) {

  uint8_t cmd_byte, status = 0xFF;

  // Error checking
  if ((num_data > 0) && (!data_buf)) {
    fprintf_P(stderr, PSTR("\nERROR: Invalid data buffer in spi2_cmd\n"));
    return SPI2_STS_HALT;
  }
  if (rnw > 1) {
    fprintf_P(stderr, PSTR("\nERROR: Read Not Write bit out of range in spi2_cmd (0 - 1)\n"));
    return SPI2_STS_HALT;
  }
  //TODO Check if cmd is invalid

  // Generate command byte by inserting RnW bit to MSB
  cmd_byte = cmd | (rnw << 7);

  // Write out command byte (slave request, skip this)
  if (!slave_req) {
    spi2->write(cmd_byte, false);
    spi2->flush();  //TODO: Investigate why flush needed
  }

  // Process data bytes if available
  if (rnw && (num_data > 0)) {
    spi2->read(data_buf, num_data, false);  // No-op = 0x00 (default for this function)
  } else if (num_data > 0) {
    spi2->write(data_buf, num_data, false);
  }

  // Read and return status byte
  status = spi2->read(true, 0xFF);  // Send dummy 0xFF byte on MOSI
  return status;
}

void spi2_test() {

  uint8_t buf[16] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  // Initialize the SPI peripheral and set to this channel
  spi2.reset(new Motate::SPI<kSocket2_SPISlaveSelectPinNumber>(SPI2_MCK_DIV));

  spi2->setChannel();

  // Let's write some DEAD BEEF out on the wire...
  //spi2->write(buf, 4, true);
  //spi2->write(buf, 4, true);

  // Reads, ignore what slave sent (Aardvark test)
  //spi2->read(true);
  //spi2->read(buf, 4);

  // Try a few sample commands (0x00, 0x01, 0x03)
  spi2_cmd(false, SPI2_WRITE, SPI2_CMD_RST_ENC_POS, buf, 0);
  spi2_cmd(false, SPI2_WRITE, SPI2_CMD_START_TOOL_TIP, buf, 0);
  spi2_cmd(false, SPI2_READ, SPI2_CMD_REQ_ENC_POS, buf, 16);
}

// SPI2 Slave ISR
void Pin<kSocket3_SPISlaveSelectPinNumber>::interrupt() {

  // TEST: Toggle Coolant Enable, status LED (D15)
  coolant_enable_pin.toggle();
}
