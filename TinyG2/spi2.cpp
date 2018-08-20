#include "tinyg2.h"					// #1 There are some dependencies
#include "config.h"					// #2
#include "hardware.h"
#include "spi2.h"
#include "motate/utility/SamSPI.h"
#include "motate/MotateTimers.h"

#include <stdio.h>
#include <memory>

// SPI2 Slave Interrupt Flag
static volatile bool spi2_slave_int = false;

// Create a SPI instance using CS1 pin for select
std::shared_ptr<Motate::SPI<kSocket2_SPISlaveSelectPinNumber>> spi2;

// spi2_init: Initializes the SPI2 subsystem
void spi2_init() {

  // Pin direction, type, debounce set in hardware.h

  // Initialize the SPI peripheral and set to this channel
  spi2.reset(new Motate::SPI<kSocket2_SPISlaveSelectPinNumber>(SPI2_MCK_DIV));

  // Add delays for 1) SS low to SCLK and 2) between transfers for SPI slave
  spi2->setBSDelay(SPI2_DLYBS_US);
  spi2->setBCTDelay(SPI2_DLYBCT_US);

  // Set up highest priority interrupt on falling edge of SPI_CS2 pin
  spi2_int_pin.setInterrupts(kPinInterruptOnFallingEdge|kPinInterruptPriorityHighest);

  // Clear Slave Interrupt Flag
  spi2_slave_int = false;

}

// spi2_cmd: Start a SPI master transfer (read/write) using the provided buffer
uint8_t spi2_cmd(bool slave_req, uint8_t rnw, uint8_t cmd_byte, uint8_t *data_buf, uint16_t num_data) {

  // Error checking
  if ((num_data > 0) && (!data_buf)) {
    fprintf_P(stderr, PSTR("\nERROR: Invalid data buffer in spi2_cmd\n"));
    spi2->read(true); // Toss, release the line
    return SPI2_STS_HALT;
  }
  if (rnw > 1) {
    fprintf_P(stderr, PSTR("\nERROR: Read Not Write bit out of range in spi2_cmd (0 - 1)\n"));
    spi2->read(true); // Toss, release the line
    return SPI2_STS_HALT;
  }
  //TODO Check if cmd_byte is invalid

  // Write out command byte (slave request, skip this)
  if (!slave_req) {
    spi2->write(cmd_byte, true);
    while(!spi2->is_tx_empty());  // Wait for TXEMPTY to flush - TODO timeout
    while(spi2->is_rx_ready()) {  // Clear unused RX data without invoking read
      spi2->read();
    }
    delay(1);                     //TEMP Delay 1ms
  }

  // Process data bytes if available
  if (rnw && (num_data > 0)) {
    spi2->read(data_buf, num_data, true);   // Read RX data (performs dummy writes, doesn't count if RDRF = 0)
    delay(1);                               //TEMP Delay 1ms
  } else if (num_data > 0) {
    spi2->write(data_buf, num_data, true);
    while(!spi2->is_tx_empty());            // Wait for TXEMPTY to flush - TODO timeout
    while(spi2->is_rx_ready()) {            // Clear unused RX data without invoking read
      spi2->read();
    }
    delay(1);                               //TEMP Delay 1ms
  }

  // Read and return status byte
  while ((spi2->read(true)) < 0); // Waits until RX ready to read (performs dummy writes) - TODO add timeout
  delay(1); //TEMP Delay 1ms

  // Flush the system in case leftovers in buffers
  spi2->flush();

  return SPI2_STS_OK;
}

// spi2_slave_handler: Processes SPI2 slave interrupts and executes commands
uint8_t spi2_slave_handler() {

  uint8_t cmd, status;
  int16_t ret;

  uint8_t buf[16] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
                     0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x5A}; //TEMP

  // SPI2 Slave Interrupt received
  if (spi2_slave_int) {

    delay(1); //TEMP Delay 1ms

    // Clear flag
    spi2_slave_int = false;

    // Disable Interrupts
	  //__disable_irq();

    // Enable SPI2 slave select
    spi2->setChannel();

    // Read command
    do {
      ret = spi2->read(true);
    } while (ret < 0); // Waits until RX ready to read (performs dummy writes) - TODO add timeout

    // Convert return to command
    cmd = (uint8_t)(ret & 0x00FF);

    switch (cmd) {

      // Send motor positions (slave requested write)
      case SPI2_CMD_SND_MTR_POS:

        //TODO implement

        spi2_cmd(true, SPI2_WRITE, SPI2_CMD_NULL, buf, 16); //TEMP

        status = SPI2_STS_OK;
        break;

      // Else, error (no other valid slave commands)
      default:

        fprintf_P(stderr, PSTR("\nERROR: Invalid command from SPI2 slave\n"));
        spi2->read(true); // Toss, release the line
        status = SPI2_STS_HALT;
        break;
    }

    // Enable Interrupts
    //__enable_irq();

  } else {
    status = SPI2_STS_OK;
  }

  return status;
}

void spi2_test() {

  uint8_t buf[16] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  spi2->setChannel();

  // Bogus command
  spi2_cmd(false, SPI2_WRITE, 0x00, buf, 0);

  // Try a few sample commands (0x01, 0x02, 0x04)
  spi2_cmd(false, SPI2_WRITE, SPI2_CMD_RST_ENC_POS, buf, 0);
  spi2_cmd(false, SPI2_WRITE, SPI2_CMD_START_TOOL_TIP, buf, 0);
  spi2_cmd(false, SPI2_READ, SPI2_CMD_REQ_ENC_POS, buf, 16);

  // Command 0x03 triggered by SPI2 slave interrupt request
}

// SPI2 Slave ISR
void Pin<kSocket3_SPISlaveSelectPinNumber>::interrupt() {

  // Set Slave Interrupt Flag
  spi2_slave_int = true;

  // TEST: Toggle Coolant Enable, status LED (D15)
  //coolant_enable_pin.toggle();
}
