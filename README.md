# TinyG2

This is the Bantam Tools V3 development fork of Synthetos's g2 firmware.

## Compiling Firmware using Atmel Studio

This repository contains an Atmel Studio project **TinyG2.atsln** which can be found in the `TinyG2/` directory.  The project solution file can be used to compile the TinyG firmware in the Atmel Studio IDE without the need to download GCC or compile from source.

Open the **TinyG2.atsln** project using Atmel Studio.

Make sure that the **V3** platform is selected in the top toolbar next to the debug functions as shown below:

![Platform Selection](https://github.com/bantamtools/g2/blob/msx_dev/Resources/screenshots/v3_platform.png "Platform Selection")

To compile the entire TinyG solution, choose **Build > Build Solution** (or **Build > Rebuild Solution** to build from scratch).  This will begin the compile process using the custom Makefile included in the `TinyG2/` directory.

**NOTE**: The compile process may take several minutes for a clean build due to downloading and extracting GCC.

A sample build output is shown below:

```
------ Build started: Project: TinyG2, Configuration: V3 ARM ------
Build started.
Project "TinyG2.cppproj" (default targets):
Target "PreBuildEvent" skipped, due to false condition; ('$(PreBuildEvent)'!='') was evaluated as (''!='').
Target "CoreBuild" in file "C:\Program Files (x86)\Atmel\Studio\7.0\Vs\Compiler.targets" from project "C:\git\bt\sam_tinyg_test\TinyG2\TinyG2.cppproj" (target "Build" depends on it):
	Task "RunCompilerTask"
		Shell Utils Path C:\Program Files (x86)\Atmel\Studio\7.0\shellUtils
		C:\Program Files (x86)\Atmel\Studio\7.0\shellUtils\make.exe -C "C:\git\bt\sam_tinyg_test\TinyG2" -f "Makefile" PLATFORM=V3 
		make: Entering directory 'C:/git/bt/sam_tinyg_test/TinyG2'
		"Found that we're in Atmel Studio"
		The system cannot find the path specified.
		Compiling c ./fix16.c
		    -> build/V3/./fix16.o
		Compiling c fatfs/ff.c
		    -> build/V3/fatfs/ff.o
		Compiling c CMSIS/Device/ATMEL/sam3xa/source/system_sam3xa.c
		    -> build/V3/CMSIS/Device/ATMEL/sam3xa/source/system_sam3xa.o
		Compiling c CMSIS/Device/ATMEL/sam3xa/source/gcc/startup_sam3xa.c
		    -> build/V3/CMSIS/Device/ATMEL/sam3xa/source/gcc/startup_sam3xa.o
		Compiling c platform/atmel_sam/cortex_handlers.c
		    -> build/V3/platform/atmel_sam/cortex_handlers.o
		Compiling c platform/atmel_sam/hooks.c
		    -> build/V3/platform/atmel_sam/hooks.o
		Compiling cpp motate/SamSPI.cpp
		    -> build/V3/motate/SamSPI.o
		Compiling cpp motate/SamPins.cpp
		    -> build/V3/motate/SamPins.o
		Compiling cpp motate/SamTimers.cpp
		    -> build/V3/motate/SamTimers.o
		Compiling cpp motate/SamUSB.cpp
		    -> build/V3/motate/SamUSB.o
		Compiling cpp motate/AvrUSB.cpp
		    -> build/V3/motate/AvrUSB.o
		Compiling cpp ./pwm.cpp
		    -> build/V3/./pwm.o
		Compiling cpp ./test.cpp
		    -> build/V3/./test.o
		Compiling cpp ./plan_exec.cpp
		    -> build/V3/./plan_exec.o
		Compiling cpp ./plan_zoid.cpp
		    -> build/V3/./plan_zoid.o
		Compiling cpp ./main.cpp
		    -> build/V3/./main.o
		Compiling cpp ./help.cpp
		    -> build/V3/./help.o
		Compiling cpp ./gcode_parser.cpp
		    -> build/V3/./gcode_parser.o
		Compiling cpp ./persistence.cpp
		    -> build/V3/./persistence.o
		Compiling cpp ./encoder.cpp
		    -> build/V3/./encoder.o
		Compiling cpp ./canonical_machine.cpp
		    -> build/V3/./canonical_machine.o
		Compiling cpp ./spindle.cpp
		    -> build/V3/./spindle.o
		Compiling cpp ./config.cpp
		    -> build/V3/./config.o
		Compiling cpp ./report.cpp
		    -> build/V3/./report.o
		Compiling cpp ./cycle_jogging.cpp
		    -> build/V3/./cycle_jogging.o
		Compiling cpp ./plan_arc.cpp
		    -> build/V3/./plan_arc.o
		Compiling cpp ./kinematics.cpp
		    -> build/V3/./kinematics.o
		Compiling cpp ./xio.cpp
		    -> build/V3/./xio.o
		Compiling cpp ./config_app.cpp
		    -> build/V3/./config_app.o
		Compiling cpp ./controller.cpp
		    -> build/V3/./controller.o
		Compiling cpp ./json_parser.cpp
		    -> build/V3/./json_parser.o
		Compiling cpp ./plan_line.cpp
		    -> build/V3/./plan_line.o
		Compiling cpp ./stepper.cpp
		    -> build/V3/./stepper.o
		Compiling cpp ./cycle_probing.cpp
		    -> build/V3/./cycle_probing.o
		Compiling cpp ./hardware.cpp
		    -> build/V3/./hardware.o
		Compiling cpp ./planner.cpp
		    -> build/V3/./planner.o
		Compiling cpp ./switch.cpp
		    -> build/V3/./switch.o
		Compiling cpp ./spi2.cpp
		    -> build/V3/./spi2.o
		Compiling cpp ./text_parser.cpp
		    -> build/V3/./text_parser.o
		Compiling cpp ./cycle_homing.cpp
		    -> build/V3/./cycle_homing.o
		Compiling cpp ./util.cpp
		    -> build/V3/./util.o
		Compiling cpp fatfs/diskio.cpp
		    -> build/V3/fatfs/diskio.o
		Compiling cpp platform/atmel_sam/Reset.cpp
		    -> build/V3/platform/atmel_sam/Reset.o
		Compiling cpp platform/atmel_sam/UniqueId.cpp
		    -> build/V3/platform/atmel_sam/UniqueId.o
		Compiling cpp platform/atmel_sam/syscalls_sam3.cpp
		    -> build/V3/platform/atmel_sam/syscalls_sam3.o
		Linking bin/V3/V3.elf
		Using linker script: C:/git/bt/sam_tinyg_test/TinyG2/platform/atmel_sam/gcc_flash.ld
C:\git\bt\sam_tinyg_test\TinyG2\build\V3\CMSIS\Device\ATMEL\sam3xa\source\gcc\startup_sam3xa.o(1,1): warning: undefined reference to `EMAC_Handler'
		Exporting symbols bin/V3/V3.elf.txt
		Making binary bin/V3/V3.bin
		4+0 records in
		4+0 records out
		4 bytes copied, 0.0211035 s, 0.2 kB/s
		4+0 records in
		4+0 records out
		4 bytes copied, 0.0310835 s, 0.1 kB/s
		--- SIZE INFO ---
		   text	   data	    bss	    dec	    hex	filename
		 177944	      0	  14792	 192736	  2f0e0	bin/V3/V3.elf
		cp bin/V3/V3.elf TinyG2.elf
		cp bin/V3/V3.map TinyG2.map
		make: Leaving directory 'C:/git/bt/sam_tinyg_test/TinyG2'
	Done executing task "RunCompilerTask".
	Task "RunOutputFileVerifyTask"
		
		Display Output File Size Skipped due to : Output File not found
	Done executing task "RunOutputFileVerifyTask".
Done building target "CoreBuild" in project "TinyG2.cppproj".
Target "PostBuildEvent" skipped, due to false condition; ('$(PostBuildEvent)' != '') was evaluated as ('' != '').
Target "Build" in file "C:\Program Files (x86)\Atmel\Studio\7.0\Vs\Avr.common.targets" from project "C:\git\bt\sam_tinyg_test\TinyG2\TinyG2.cppproj" (entry point):
Done building target "Build" in project "TinyG2.cppproj".
Done building project "TinyG2.cppproj".

Build succeeded.
========== Build: 1 succeeded or up-to-date, 0 failed, 0 skipped ==========
```

Once the build completes, you should receive only one warning in the Error List window:

`undefined reference to EMAC_Handler`

The build has completed sucessfully and you can now flash the TinyG with firmware.

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

## Alternate: Compiling Firmware from Source

NOTE: The Atmel Studio method is recommended over compiling from source because it is faster and enables debugging through the IDE.

### Toolchain

This project requires gcc 4.8.

You can install the toolchain using the following command:

```
make toolchain
```
### Build

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

### Clean Build

To clean, run `make clean` at the top-level. By default, this will run a clean build for the `G2v9i` platform.

If you need to clean another platform, specify it as you would during compilation:

```
cd TinyG2 && make PLATFORM=V3 clean
cd TinyG2 && make PLATFORM=G2v9i clean
```

### Firmware Binaries

Firmware binaries can be found in the `TinyG2/bin` folder. Output is organized by `PLATFORM`.

The build puts both an ELF and a binary. ELF is the executable linker format and has symbols for debugging. The .bin is the binary image that we can supply to Atmel Studio or BOSSA.

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
