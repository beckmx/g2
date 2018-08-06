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
//ALT Motate::SPI<kSocket2_SPISlaveSelectPinNumber> spi2;

// spi2_init: Initializes the SPI2 subsystem
void spi2_init() {

  // Pin direction, type, debounce set in hardware.h

  // Set up interrupt on falling edge of SPI_CS2 pin
  spi2_int_pin.setInterrupts(kPinInterruptOnFallingEdge);

}

void spi2_test() {

  uint8_t buf[4] = {0xDE, 0xAD, 0xBE, 0xEF};

  // Initialize the SPI peripheral and set to this channel
  spi2.reset(new Motate::SPI<kSocket2_SPISlaveSelectPinNumber>(SPI2_MCK_DIV));
  //ALT spi2.init(SPI2_MCK_DIV, kSPI8Bit | kSPIMode0);

  spi2->setChannel();

  // Let's write some DEAD BEEF out on the wire...
  //spi2->write(buf, 4, true);
  spi2->write(buf, 4, true);

  // Reads, ignore what slave sent (Aardvark test)
  //spi2->read(true);
  //spi2->read(buf, 4);
}

// SPI2 Slave ISR
void Pin<kSocket3_SPISlaveSelectPinNumber>::interrupt() {

  // TEST: Toggle Coolant Enable, status LED (D15)
  coolant_enable_pin.toggle();
}
