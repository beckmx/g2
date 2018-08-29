#include "tinyg2.h"					// #1 There are some dependencies
#include "config.h"					// #2
#include "canonical_machine.h"
#include "hardware.h"
#include "spi2.h"
#include "motate/utility/SamSPI.h"
#include "motate/MotateTimers.h"
#include "text_parser.h"

#include <stdio.h>
#include <memory>

// Buffer for passing data to SPI2 command processor
uint8_t buf[SPI2_BUF_SIZE] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
                              0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// SPI2 Slave Interrupt Flag
static volatile bool spi2_slave_int = false;

// Create a SPI instance using CS1 pin for select
std::shared_ptr<Motate::SPI<kSocket2_SPISlaveSelectPinNumber>> spi2;

//TEMP: Calibrated delay function
void delay_us(uint32_t us) {
  for(uint32_t i = 0; i < us * 5; i++); // Calibrated empirically
}
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

  int16_t ret;
  uint8_t sts_byte;

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

  // Select the SS for the SPI2 slave
  spi2->setChannel();

  // Write out command byte (slave request, skip this)
  if (!slave_req) {
    spi2->write(cmd_byte, true);
    while(!spi2->is_tx_empty());  // Wait for TXEMPTY to flush - TODO timeout
    while(spi2->is_rx_ready()) {  // Clear unused RX data without invoking read
      spi2->read();
    }
    // Request Encoder Positions command requires time for SPI2 to prep data - TODO fix performance
    if (cmd_byte == SPI2_CMD_REQ_ENC_POS) {
      delay(4);
    } else {
      delay_us(25);
    }
  }

  // Process data bytes if available
  if (rnw && (num_data > 0)) {
    spi2->read(data_buf, num_data, true);   // Read RX data (performs dummy writes, doesn't count if RDRF = 0)
    delay_us(25);
  } else if (num_data > 0) {
    spi2->write(data_buf, num_data, true);
    while(!spi2->is_tx_empty());            // Wait for TXEMPTY to flush - TODO timeout
    while(spi2->is_rx_ready()) {            // Clear unused RX data without invoking read
      spi2->read();
    }
    delay_us(25);
  }

  // Read the status
  while ((ret = spi2->read(true)) < 0) {  // Waits until RX ready to read (performs dummy writes) - TODO add timeout
    delay_us(25);
  }

  // Convert return to status byte
  sts_byte = (uint8_t)(ret & 0x00FF);

  // Flush the system in case leftovers in buffers
  spi2->flush();

  // Return the status code
  return sts_byte;
}

// spi2_slave_handler: Processes SPI2 slave interrupts and executes commands
uint8_t spi2_slave_handler() {

  uint8_t cmd, status;
  int16_t ret;

  //TEMP Generate random data for testing
  for (int i = 0; i < SPI2_BUF_SIZE; i++) {
    buf[i] = (i + 10) * 2;
  }

  // SPI2 Slave Interrupt received
  if (spi2_slave_int) {

    delay_us(25);

    // Clear flag
    spi2_slave_int = false;

    // Disable Interrupts
	  //__disable_irq();

    // Enable SPI2 slave select
    spi2->setChannel();

    // Read command
    while ((ret = spi2->read(true)) < 0){ // Waits until RX ready to read (performs dummy writes) - TODO add timeout
      delay_us(25);
    }

    // Convert return to command
    cmd = (uint8_t)(ret & 0x00FF);

    switch (cmd) {

      // Send motor positions (slave requested write)
      case SPI2_CMD_SND_MTR_POS:

        status = spi2_send_motor_positions();
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

// spi2_reset_encoder_positions: reset encoder positions (command 0x01)
uint8_t spi2_reset_encoder_positions() {
  return (spi2_cmd(false, SPI2_WRITE, SPI2_CMD_RST_ENC_POS, NULL, 0));
}

// spi2_start_tool_tip: start tool tip command (command 0x02)
uint8_t spi2_start_tool_tip() {
  return (spi2_cmd(false, SPI2_WRITE, SPI2_CMD_START_TOOL_TIP, NULL, 0));
}

// spi2_send_motor_positions: send motor positions (command 0x03)
uint8_t spi2_send_motor_positions() {

  float f;
  uint32_t u;

  // Get each motor position, convert to uint32_t and store bytes into buffer
  for (uint8_t axis = AXIS_X; axis < SPI2_NUM_AXES; axis++) {

    // Convert from float to unsigned 32-bit integer
    f = cm_get_g28_position(axis);
    u = FLOAT_TO_U32(f);

    // Break 32-bits into separate bytes
    buf[axis*4] = (uint8_t)((u >> 24) & 0xFF);
    buf[axis*4+1] = (uint8_t)((u >> 16) & 0xFF);
    buf[axis*4+2] = (uint8_t)((u >> 8) & 0xFF);
    buf[axis*4+3] = (uint8_t)(u & 0xFF);

  }

  // Write out buffer with motor position data to SPI slave and return status
  return(spi2_cmd(true, SPI2_WRITE, SPI2_CMD_NULL, buf, (SPI2_NUM_AXES*4)));
}

// spi2_request_encoder_positions: request encoder positions (command 0x04)
uint8_t spi2_request_encoder_positions() {

  uint8_t st;
  uint32_t u;

  // Get the encoder position data as one 16-byte transfer
	st = spi2_cmd(false, SPI2_READ, SPI2_CMD_REQ_ENC_POS, buf, (SPI2_NUM_AXES*4));

  // Convert the data in the buffer to their approriate array values
  for (uint8_t axis = AXIS_X; axis < SPI2_NUM_AXES; axis++) {
    u = ((buf[axis*4] << 24) + (buf[axis*4+1] << 16) + (buf[axis*4+2] << 8) + (buf[axis*4+3]));
    spi2_encoder_pos[axis] = U32_TO_FLOAT(u);
  }

  return st;
}

// spi2_test: unit testing for SPI2 interface
void spi2_test() {

  spi2->setChannel();

  // Bogus command
  spi2_cmd(false, SPI2_WRITE, 0x00, buf, 0);

  // Try a few sample commands (0x01, 0x02, 0x04)
  spi2_cmd(false, SPI2_WRITE, SPI2_CMD_RST_ENC_POS, buf, 0);
  spi2_cmd(false, SPI2_WRITE, SPI2_CMD_START_TOOL_TIP, buf, 0);
  spi2_cmd(false, SPI2_READ, SPI2_CMD_REQ_ENC_POS, buf, (SPI2_NUM_AXES*4));

  // Random commands
  spi2_cmd(false, SPI2_WRITE, SPI2_CMD_START_TOOL_TIP, buf, 0);
  spi2_cmd(false, SPI2_WRITE, SPI2_CMD_RST_ENC_POS, buf, 0);
  spi2_cmd(false, SPI2_WRITE, SPI2_CMD_RST_ENC_POS, buf, 0);
  spi2_cmd(false, SPI2_WRITE, 0x00, buf, 0);
  spi2_cmd(false, SPI2_READ, SPI2_CMD_REQ_ENC_POS, buf, (SPI2_NUM_AXES*4));
  spi2_cmd(false, SPI2_WRITE, SPI2_CMD_RST_ENC_POS, buf, 0);

  // Command 0x03 triggered by SPI2 slave interrupt request
}

// SPI2 Slave ISR
void Pin<kSocket3_SPISlaveSelectPinNumber>::interrupt() {

  // Set Slave Interrupt Flag
  spi2_slave_int = true;

}


/////////////////////////////////////////////////////////////
// Begin SW commands (mostly wrappers for SPI2 functions)
/////////////////////////////////////////////////////////////

//
// Commands:
//
//  cmd1 - Reset Encoder Positions to Zero
//  cmd2 - Start Tool Tip Command
//  cmd4 - Request Encoder Positions
//

// Encoder data global variable (for JSON commands)
float spi2_encoder_pos[SPI2_NUM_AXES] = {0.0,0.0,0.0,0.0};

// spi2_cmd_helper: helper function to return proper status
stat_t spi2_cmd_helper(uint8_t sts_byte) {

  stat_t status;

  // Convert status byte to TinyG status code
  // (0x00 over SPI2 not in use, potential false OK status) - TODO use TinyG status only
  switch (sts_byte) {
    case SPI2_STS_OK: status = STAT_OK; break;
    default: status = STAT_ERROR; break; // TODO - handle other non-OK statuses
  }

  return status;
}

stat_t spi2_cmd1_set(nvObj_t *nv) {
  return (spi2_cmd_helper(spi2_reset_encoder_positions()));
}

stat_t spi2_cmd2_set(nvObj_t *nv) {
	return (spi2_cmd_helper(spi2_start_tool_tip()));
}

stat_t spi2_cmd4_set(nvObj_t *nv) {
  return (spi2_cmd_helper(spi2_request_encoder_positions()));
}

// Print functions (text-mode only)
#ifdef __TEXT_MODE

static const char msg_units0[] PROGMEM = " in";	// used by generic print functions
static const char msg_units1[] PROGMEM = " mm";
static const char msg_units2[] PROGMEM = " deg";
static const char *const msg_units[] PROGMEM = { msg_units0, msg_units1, msg_units2 };

static const char fmt_spi2_cmd1[] PROGMEM = "Reset Encoder Positions to Zero\n";
static const char fmt_spi2_cmd2[] PROGMEM = "Start Tool Tip Command\n";
static const char fmt_spi2_cmd4[] PROGMEM = "%c Encoder Position:%15.3f%s\n";

static int8_t _get_axis(const index_t index)
{
	char_t *ptr;
	char_t tmp[TOKEN_LEN+1];
	char_t axes[] = {"xyza"};

	strncpy_P(tmp, cfgArray[index].token, TOKEN_LEN);	// kind of a hack. Looks for an axis
	if ((ptr = strchr(axes, tmp[0])) == NULL) {			// character in the 0 and 3 positions
		if ((ptr = strchr(axes, tmp[3])) == NULL) {		// to accommodate 'xam' and 'g54x' styles
			return (-1);
		}
	}
	return (ptr - axes);
}

static void _print_enc_pos(nvObj_t *nv, const char *format, uint8_t units)
{
	char axes[] = {"XYZA"};
	uint8_t axis = _get_axis(nv->index);
	fprintf_P(stderr, format, axes[axis], nv->value, GET_TEXT_ITEM(msg_units, units));
}

void spi2_cmd1_print(nvObj_t *nv) { text_print_nul(nv, fmt_spi2_cmd1);}
void spi2_cmd2_print(nvObj_t *nv) { text_print_nul(nv, fmt_spi2_cmd2);}
void spi2_cmd4_print(nvObj_t *nv) { _print_enc_pos(nv, fmt_spi2_cmd4, cm_get_units_mode(MODEL));}
#endif

/////////////////////////////////////////////////////////////
// End SW commands
/////////////////////////////////////////////////////////////
