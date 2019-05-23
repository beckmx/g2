# TinyG2

This is the Bantam Tools V3 development fork of Synthetos's g2 firmware.

## Toolchain

This project requires gcc 4.8.

You can install the toolchain using the following command:

```
make toolchain
```

## Compiling Firmware

The firmware is contained in the `TinyG2/` directory.

When compiling the TinyG firmware, you need to specify a platform:

```
make PLATFORM=V3
make PLATFORM=G2v9i
```

You must build the firmware from the `TinyG2/` directory:

```
cd TinyG2 && make PLATFORM=V3
```

Note: For the V3, you may run the makefile with either `V3` or `v3` as the `PLATFORM`.

### Firmware Binaries

Firmware binaries can be found in the `TinyG2/bin` folder. Output is organized by `PLATFORM`.

The build puts both an ELF and a binary. ELF is the executable linker format and has symbols for debugging. The .bin is the binary image that we can supply to Atmel Studio or BOSSA.

## Flashing the TinyG2 using Atmel Studio

Using Atmel Studio, the TinyG can be programmed using the binary (.bin) file in the `TinyG2/bin` folder for your `PLATFORM`.  This method requires a Segger J-Link with an ARM 20-pin to JTAG 10-pin adapter, such as the [Olimex ARM-JTAG-20-10](https://www.digikey.com/products/en?keywords=ARM-JTAG-20-10%09).

Connect the J-Link with adapter to connector **J6** on the TinyG as shown below.  Power up the TinyG and open up Atmel Studio.

![J-Link connected to TinyG](https://github.com/bantamtools/g2/blob/msx_dev/Resources/screenshots/segger_tinyg.jpg "J-Link connected to TinyG")

In Atmel Studio, go to **Tools > Device Programming**.  There is no need to open a specific project.

In the Device Programming window, select the following parameters and hit **Apply**:
* Tool: **J-Link**
* Device: **ATSAM3X8C**
* Interface: **JTAG**

![Device Programming](https://github.com/bantamtools/g2/blob/msx_dev/Resources/screenshots/dev_prog.png "Device Programming")

You may hit **Read** next to the Device Signature or Target Voltage to verify your J-Link is properly connected to the TinyG.

Hit **Memories** on the left-hand side of the window.  Under **Flash**, browse to the .bin file you generated.  Be sure to choose **Binary (.bin, .dat) (\*.bin;\*.dat)** as the file type when browsing for your file.

![Memories](https://github.com/bantamtools/g2/blob/msx_dev/Resources/screenshots/memories.png "Memories")

**NOTE**: For Linux users, you will have to copy the .bin file over to Windows, as Atmel Studio is a Windows-only application.

Hit **Program** to write your TinyG binary to flash and have it verified.

## Alternate: Flashing the TinyG2 using BOSSA

**NOTE**: The Atmel Studio method is recommended over BOSSA because it is faster and less error-prone.

To get TinyG2 to reboot into SAM-BA mode, you can issue the json command {"boot":1}.  You can also open & close the serial port at 1200 baud.  The easiest way to do the latter is with the command stty -f /dev/tty.usbserial* 1200.

When in SAM-BA mode, we update firmware with the open source utility BOSSA (the SAM-BA version of avrdude).  Our branch of it is [here](https://github.com/omco/bossa/tree/arduino), though the open-source version can be used for standalone flashing.

To flash the program using BOSSA, use the following command template:

```
bossac -p tty.usbserialXX -e -w -v -b -R path/to/firmware.bin
```

Example:

```
bossac -p tty.usbmodem1411 -e -w -v -b -R g2/TinyG2/bin/V3/V3.bin
```

* `-p` specifies the serial port.  It is likely to be `/dev/tty.usbserialXX`.
	* Important: `-p` prepends "`/dev/`", so if you supply the `/dev/`, it won't find the serial port.
* `-e` is erase
* `-w` is write,
* `-v` is verify (aka read back and check that it's valid).
* `-b` sets the machine to boot into firmware next time it reboots
* `-R` tells it to reboot.
* The last argument is the filename of the firmware, in .bin format.

## Verifying the TinyG Firmware Version

To verify that the correct TinyG version is installed (based on versions provided in https://github.com/bantamtools/g2/releases), connect the TinyG to your computer via USB.  Power up the TinyG and open the Bantam Tools software application.

When the application starts, you'll notice several numbers in the bottom left corner of your screen.  The second number (typically with the format `300.xx`) designates the current firmware build running on the TinyG.

![Bantam Tools Application](https://github.com/bantamtools/g2/blob/msx_dev/Resources/screenshots/bt_app.png "Bantam Tools Application")

**NOTE**: To update the TinyG firmware version for future versions and custom test builds, edit the `TINYG_FIRMWARE_BUILD` variable found in `TinyG2/tinyg2.h` file of this repository and rebuild the firmware.

### Clean Build

To clean, run `make clean` at the top-level. By default, this will run a clean build for the `G2v9i` platform.

If you need to clean another platform, specify it as you would during compilation:

```
cd TinyG2 && make PLATFORM=V3 clean
cd TinyG2 && make PLATFORM=G2v9i clean
```

## Debugging

You can debug the TinyG through JTAG using a SAM-ICE adapter and the J-Link tools.

### Installing JLink Tools

You will need to download and install the J-Link software package [from here](https://www.segger.com/downloads/jlink/). This will provide `JLinkGDBServer`, which we use for talking to the SAMA-ICE adapter.

### Starting the JLinkGDBServer

You can start the JLinkGDBServer from the command line:

```
JLinkGDBServer -USB -device ATSAM3X8C -endian little -if JTAG -speed auto -noir
```

### Connecting to the Device

Once the server is running, you can connect to the device using `arm-none-eabi-gdb`.

In the `TinyG/` directory, run `arm-none-eabi-gdb`. `gdb` will mention that it connected to JLinkGDBServer.

We recommend running `arm-none-eabi-gdb` inside of the `TinyG/` folder so that the `.gdbinit` file is detected.

### Make Debug Command

You can build the code and open a debug terminal by adding `debug` to the build command:

```
make PLATFORM=V3 debug
```

This will automatically connect to `gdb`. The `JLinkGDBServer` needs to be running.

## Further Reading

* [g2 wiki](https://github.com/bantamtools/g2/wiki)
* [Bantam: Intro to V2 Firmware](https://sites.google.com/a/othermachine.co/software/dev-instructions-and-resources/intro-to-firmware?pli=1)
* [Bantam: Dev Instructions & Resources](https://sites.google.com/a/othermachine.co/software/dev-instructions-and-resources/)
