#ifndef _SPI_H
#define _SPI_H

#ifdef __cplusplus
extern "C" {
#endif

// Structures and Type Definitions
struct spi2_fw_type
{
   uint8_t major, minor, rev;
};

typedef enum {
    SPI2_SPD_IDX = 0,
    SPI2_SPD_R,
    SPI2_SPD_G,
    SPI2_SPD_B,
    SPI2_SPD_W,
} spi2SpdEnum;

// General Definitions
#define SPI2_MCK_DIV      (SystemCoreClock / 16)   // SPI clock divider to generate baud (based on 84MHz MCK)
#define SPI2_DLYBS_NS     357                      // Delay between SS low and SCLK (in ns)
#define SPI2_DLYBCT_NS    381                      // Delay between transfers (in ns)
#define SPI2_NUM_AXES     4                        // Number of axes (XYZA) - DO NOT use AXES for SPI2!
#define SPI2_BUF_SIZE     (SPI2_NUM_AXES*4)        // Buffer size for passing data
#define SPI2_NUM_RETRIES  3                        // Number of retries on error condition
#define SPI2_TIMEOUT      100                      // SPI2 bus timeout (in ms)

// SPI2 Command Set
#define SPI2_CMD_RST_ENC_POS    0x01  // Reset Encoder Positions to Zero
#define SPI2_CMD_START_TOOL_TIP 0x02  // Start Tool Tip Command
#define SPI2_CMD_SND_MTR_POS    0x03  // Send Motor Positions
#define SPI2_CMD_REQ_ENC_POS    0x04  // Request Encoder Positions

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
#define SPI2_CMD_RST_ESC_VAL    0x4A  // Reset Min/Max/Mean ESC Current
#define SPI2_CMD_RD_ESC_VAL     0x4B  // Read Min/Max/Mean ESC Current
#define SPI2_CMD_SET_THRES      0x4C  // Set ESC Current Threshold Value, Time
#define SPI2_CMD_FW_VER         0x4D  // Firmware Version

#define SPI2_STS_OK             0x10  // OK Status
#define SPI2_STS_ERR            0x11  // Error Status
#define SPI2_STS_TIMEOUT        0x12  // Timeout Status
#define SPI2_STS_RETRIES        0x13  // Exceeded Retries Status

#define SPI2_STOP_TOOL_TIP      100.0 // WORKAROUND: Unique command to stop tool tip (Z Axis runs 0 to -100 typically)

// Function Prototypes
void spi2_init(void);
uint8_t spi2_cmd(bool, uint8_t, uint8_t*, uint16_t, uint8_t *, uint16_t);
uint8_t spi2_slave_handler(void);
uint8_t spi2_reset_encoder_positions(void);
uint8_t spi2_start_tool_tip(void);
uint8_t spi2_send_motor_positions(void);
uint8_t spi2_request_encoder_positions(void);
uint8_t spi2_read_encoder_position(uint8_t);
uint8_t spi2_set_user_io(uint8_t);
uint8_t spi2_clear_user_io(uint8_t);
uint8_t spi2_read_user_io(uint8_t);
uint8_t spi2_set_user_led(uint8_t);
uint8_t spi2_clear_user_led(uint8_t);
uint8_t spi2_read_itr_loop(uint8_t);
uint8_t spi2_set_spindle_led(void);
uint8_t spi2_set_epsilon(void);
uint8_t spi2_get_fw_version(void);
void spi2_test(void);

stat_t spi2_cmd_helper(uint8_t);

stat_t spi2_cmd64_get(nvObj_t *);

stat_t spi2_cmd1_set(nvObj_t *);
stat_t spi2_cmd2_set(nvObj_t *);
stat_t spi2_cmd4_set(nvObj_t *);
stat_t spi2_cmd64_set(nvObj_t *);
stat_t spi2_cmd65_set(nvObj_t *);
stat_t spi2_cmd66_set(nvObj_t *);
stat_t spi2_cmd67_set(nvObj_t *);
stat_t spi2_cmd68_set(nvObj_t *);
stat_t spi2_cmd69_set(nvObj_t *);
stat_t spi2_cmd70_set(nvObj_t *);
stat_t spi2_cmd71_set(nvObj_t *);
stat_t spi2_cmd72_set(nvObj_t *);
stat_t spi2_cmd73_set(nvObj_t *);
stat_t spi2_cmd74_set(nvObj_t *);
stat_t spi2_cmd75_set(nvObj_t *);
stat_t spi2_cmd76_set(nvObj_t *);
stat_t spi2_cmd77_set(nvObj_t *);

#ifdef __TEXT_MODE
void spi2_cmd1_print(nvObj_t *);
void spi2_cmd2_print(nvObj_t *);
void spi2_cmd4_print(nvObj_t *);
void spi2_cmd64_print(nvObj_t *);
void spi2_cmd65_print(nvObj_t *);
void spi2_cmd66_print(nvObj_t *);
void spi2_cmd67_print(nvObj_t *);
void spi2_cmd68_print(nvObj_t *);
void spi2_cmd69_print(nvObj_t *);
void spi2_cmd70_print(nvObj_t *);
void spi2_cmd73_print(nvObj_t *);
void spi2_cmd74_print(nvObj_t *);
void spi2_cmd75_print(nvObj_t *);
void spi2_cmd77_print(nvObj_t *);
#else
#define spi2_cmd1_print tx_print_stub
#define spi2_cmd2_print tx_print_stub
#define spi2_cmd4_print tx_print_stub
#define spi2_cmd64_print tx_print_stub
#define spi2_cmd65_print tx_print_stub
#define spi2_cmd66_print tx_print_stub
#define spi2_cmd67_print tx_print_stub
#define spi2_cmd68_print tx_print_stub
#define spi2_cmd69_print tx_print_stub
#define spi2_cmd70_print tx_print_stub
#define spi2_cmd73_print tx_print_stub
#define spi2_cmd74_print tx_print_stub
#define spi2_cmd75_print tx_print_stub
#define spi2_cmd77_print tx_print_stub
#endif

#ifdef __cplusplus
}
#endif

#endif
