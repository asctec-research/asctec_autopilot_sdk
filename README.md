# AscTec AutoPilot SDK
SDK for the embedded high-level microcontroller on all AutoPilot systems (AscTec Hummingbird, AscTec Pelican, AscTec Firefly).

## Prerequisites
First we need to install the [GCC ARM Embedded](http://launchpad.net/gcc-arm-embedded) and a programming/debug interface ([OpenOCD](http://openocd.org/)).

__Debian/Ubuntu__

Debian-based distros can use:

    sudo apt-get install gcc-arm-none-eabi openocd
    
You may also need to add a udev rule to gain access to the JTAG adapter with your user via:

    echo 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="baf8", MODE="0660", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-jtag.rules
    
Make sure your user is in the `plugdev` group and to restart your computer after adding this rule.
    
__Windows__

* For the compiler, please use the installer or corresponding ZIP file, e.g. from [2015-q3 release](http://launchpad.net/gcc-arm-embedded/4.9/4.9-2015-q3-update)
* Prebuilt binaries of OpenOCD can be found [here](http://github.com/gnuarmeclipse/openocd/releases)  
* Install WinUSB drivers for the JTAG adapter.
    * Make sure the JTAG adapter is attached to your computer.
    * Download Zadig for Windows [here](http://zadig.akeo.ie/). This tool simplifies driver installation.
    * Start Zadig and select _Options => List All Devices_
    * In the dropdown list select `USB-JTAG (Interface 0)`
    * The target driver should be _WinUSB_, click on _Replace Driver_ and wait for the operation to finish.


__Eclipse__

For easy development and debugging, the Eclipse IDE is highly recommended. The tested version is Mars.  
Please fetch the corresponding version for your system from the [Eclipse Mars download page](http://www.eclipse.org/downloads/packages/release/Mars/2).  
Select the "Eclipse IDE for C/C++ Developers" package.

__GNU ARM Eclipse Plug-In__

The [GNU ARM Eclipse](http://gnuarmeclipse.github.io/) plug-in for Eclipse is required to build and debug the trinity SDK.  
* Go to _Help => Install New Software_ in Eclipse and enter this update site `http://gnuarmeclipse.sourceforge.net/updates`.
* Select all items of the "GNU ARM C/C++ Cross Development Tools" and continue with their installation.
* Accept eventual warnings about unsigned content.
* Restart Eclipse after prompted to do so

## Setup
__Debian/Ubuntu__

No further setup is required.

__Windows__

* Open the preferences by going to _Window => Preferences_
* Go to _C/C++ => Build => Global Tools Paths_
    * Set the _Toolchain folder_ to the _bin_ folder of the location where you have extracted the embedded GCC.
* Go to _Run/Debug => OpenOCD_
    * Set the _Folder_ to the location where you have extracted OpenOCD (bin subfolder). Set the _Executable_ accordingly.

## Compiling

__Eclipse__

Select the asctec_autopilot_sdk project and select _Project => Build All_.

__Command line__

Go to the asctec_autopilot_sdk root folder and type `make all`.

__Result__

After the build process is finished the program files can be found at _release/run_. Debug files are built at _debug/run_.

## Flashing

It is a known issue that the first flash/debug operation after the JTAG adapter was powered always fails. Simply execute the corresponding flash/debug operation again to proceed.

__Eclipse__

* Connect your programmer to the UAV and turn it on.
* Go to _Run => Run Configurations..._
    * Under _C/C++ Applications_ select _AP SDK Flash Release_ to flash new code.
    
__Command line__

The flash process can also be started from a command line via `make flash`.    

## Debugging

__Eclipse__

* Connect your programmer to the UAV and turn it on.
* Go to _Run => Debug Configurations..._
    * Under _GDB OpenOCD Debugging_ select _AP SDK Debug_ to start debugging.
    * Eclipse will ask you to switch to the debugging perspective, click yes.

