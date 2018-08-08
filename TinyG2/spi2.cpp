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

  // Set up highest priority interrupt on falling edge of SPI_CS2 pin
  spi2_int_pin.setInterrupts(kPinInterruptOnFallingEdge|kPinInterruptPriorityHighest);

  // Clear Slave Interrupt Flag
  spi2_slave_int = false;

}

// spi2_safe_rd: waits until transfer done + read data ready then returns valid byte
uint8_t spi2_safe_rd(bool lastXfer) {

  //uint8_t tries = 0xFF;
  uint8_t read_byte;

  // WORKAROUND: TX buffer not empty, will cause next read to fail
  if (!(spi2->is_tx_empty())) {
    spi2->read(true, 0xFF);
  }

  /*while ((read_byte = spi2->read(lastXfer)) < 0);*/

  //return read_byte;
  return spi2->read(lastXfer);
}

// spi2_safe_rd_multi: waits until transfer done + read data ready then returns multiple valid bytes
void spi2_safe_rd_multi(uint8_t *buf, uint16_t n, bool lastXfer) {

  uint8_t read_byte;

  // WORKAROUND: TX buffer not empty, will cause next read to fail
  if (!(spi2->is_tx_empty())) {
    spi2->read(true, 0xFF);
  }

  //while ((read_byte = spi2->read(false)) < 0);

  // Set first slot in buffer to valid byte
  //buf[0] = read_byte;

  // Read in the remaining bytes
  //spi2->read(&buf[1], (n - 1), lastXfer);
  spi2->read(buf, n, lastXfer);
}


// spi2_cmd: Start a SPI master transfer (read/write) using the provided buffer
uint8_t spi2_cmd(bool slave_req, uint8_t rnw, uint8_t cmd, uint8_t *data_buf, uint16_t num_data) {

  uint8_t cmd_byte, status;

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
  //TODO Check if cmd is invalid

  // Generate and write out command byte (slave request, skip this)
  if (!slave_req) {
    cmd_byte = cmd | (rnw << 7);
    spi2->write(cmd_byte, false);
    spi2->flush();  //TODO Understand why needed
  }

  // Process data bytes if available
  if (rnw && (num_data > 0)) {
    //spi2_safe_rd_multi(data_buf, num_data, false);
    spi2->read(data_buf, num_data, true);
  } else if (num_data > 0) {
    spi2->write(data_buf, num_data, false);
    if (!(spi2->is_tx_empty())) { // WORKAROUND: TX buffer not empty, next read will fail
      spi2->read(true, 0xFF);
    }
  }

  // Read and return status byte
  status = spi2->read(true);

  return status;
}

// spi2_slave_handler: Processes SPI2 slave interrupts and executes commands
uint8_t spi2_slave_handler() {

  uint8_t cmd, status;

  uint8_t buf[16] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  // SPI2 Slave Interrupt received
  if (spi2_slave_int) {

    // Clear flag
    spi2_slave_int = false;

    // Disable Interrupts
	  //__disable_irq();

    // Enable SPI2 slave select
    spi2->setChannel();

    // Read command
    //cmd = spi2_safe_rd(false);

    // Execute command from master
    spi2_cmd(false, SPI2_WRITE, SPI2_CMD_RST_ENC_POS, buf, 0);
    spi2_cmd(false, SPI2_WRITE, SPI2_CMD_START_TOOL_TIP, buf, 0);

    if (!(spi2->is_tx_empty())) { // WORKAROUND: TX buffer not empty, next read will fail
      spi2->read(true, 0xFF);
    }

    cmd = spi2->read(false);
    spi2_cmd(true, SPI2_WRITE, SPI2_CMD_NULL, buf, 16); //TODO Temporary

    //spi2_cmd(false, SPI2_READ, SPI2_CMD_REQ_ENC_POS, buf, 16);

    while(1);

    switch (cmd) {

      // Send motor positions (slave requested write)
      case SPI2_CMD_SND_MTR_POS:

        //TODO implement
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

  //spi2->setChannel();

  // Try a few sample commands (0x00, 0x01, 0x03)
  //spi2_cmd(false, SPI2_WRITE, SPI2_CMD_RST_ENC_POS, buf, 0);
  //spi2_cmd(false, SPI2_WRITE, SPI2_CMD_START_TOOL_TIP, buf, 0);
  //spi2_cmd(false, SPI2_READ, SPI2_CMD_REQ_ENC_POS, buf, 16);

  // Command 0x02
  //spi2_safe_rd(false);
  //spi2_cmd(true, SPI2_WRITE, SPI2_CMD_NULL, buf, 16);

  //TESTING - Fixes Write then Read Issue
  //while ((cmd = spi2->read(true, 0xFF)) < 0);
  //spi2->write(buf, 16, false);
  //spi2->read(true, 0xFF);
  //while ((status = spi2->read(true, 0xFF)) < 0);
}

// SPI2 Slave ISR
void Pin<kSocket3_SPISlaveSelectPinNumber>::interrupt() {

  // Set Slave Interrupt Flag
  spi2_slave_int = true;

  // TEST: Toggle Coolant Enable, status LED (D15)
  //coolant_enable_pin.toggle();
}
