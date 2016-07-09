#Arduino-core for leaflift platform

# Setup #
1. Install USB drivers [CP210x USB to UART Bridge VCP Drivers](https://www.silabs.com/products/mcu/Pages/USBtoUARTBridgeVCPDrivers.aspx)
2. Copy libraries to ~/Documents/Arduino/library/ 


# Supported Boards #
* NodeMCU 1.0 (ESP8266-12E)
* Adafruit HUZZAH (ESP8266-12E)



# Uploading to boards - Step by step  #

## NodeMCU 1.0 (ESP-12E Module) ##
1. Select board from Tools > Board:
2. Click ( > ) to upload, board will reset when done

##  Adafruit HUZZAH ESP8266 ##
1. Select board from Tools > Board:
2. Put the Board in program mode on bootloader 
*(hold GPIO0 button then bress reset holding GPIO0 till after reset is released)*
3. Click ( > ) to upload, board will reset when done


# Tools for graphics #
[Tool for creating PROGMEM images](http://javl.github.io/image2cpp/)

# Other resources #
* https://github.com/gnulabis/UTFT-ESP8266
* https://github.com/esp8266/Arduino
* https://github.com/nodemcu/nodemcu-flasher

```
#!text
ESP8266 boot details

reset causes:
        0:
        1: normal boot
        2: reset pin
        3: software reset
        4: watchdog reset

    boot device:
        0:
        1: ram
        3: flash - See more at: http://www.esp8266.com/viewtopic.php?p=2096#p2112


struct bootflags
{
    unsigned char raw_rst_cause : 4;
    unsigned char raw_bootdevice : 4;
    unsigned char raw_bootmode : 4;

    unsigned char rst_normal_boot : 1;
    unsigned char rst_reset_pin : 1;
    unsigned char rst_watchdog : 1;

    unsigned char bootdevice_ram : 1;
    unsigned char bootdevice_flash : 1;
};
struct bootflags bootmode_detect(void) {
    int reset_reason, bootmode;
    asm (
        "movi %0, 0x60000600\n\t"
        "movi %1, 0x60000200\n\t"
        "l32i %0, %0, 0x114\n\t"
        "l32i %1, %1, 0x118\n\t"
        : "+r" (reset_reason), "+r" (bootmode) /* Outputs */
        : /* Inputs (none) */
        : "memory" /* Clobbered */
    );

struct bootflags flags;

    flags.raw_rst_cause = (reset_reason&0xF);
    flags.raw_bootdevice = ((bootmode>>0x10)&0x7);
    flags.raw_bootmode = ((bootmode>>0x1D)&0x7);

    flags.rst_normal_boot = flags.raw_rst_cause == 0x1;
    flags.rst_reset_pin = flags.raw_rst_cause == 0x2;
    flags.rst_watchdog = flags.raw_rst_cause == 0x4;

    flags.bootdevice_ram = flags.raw_bootdevice == 0x1;
    flags.bootdevice_flash = flags.raw_bootdevice == 0x3;

    return flags;

} - See more at: http://www.esp8266.com/viewtopic.php?p=2096#p2112


```

##API Status ##
http://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF51822#Downloads

## Openshift Overview ##
* https://developers.openshift.com/managing-your-applications/


## Nordicware BLE ##
* http://www.waveshare.com/wiki/NRF51822_Eval_Kit
* http://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF51822#Downloads
* http://infocenter.nordicsemi.com/pdf/nRF51822_PB_v2.5.pdf
* https://github.com/rogerclarkmelbourne/nRF51822-Arduino

## Data Sheets ##

[Provisioned Devices](https://docs.google.com/spreadsheets/d/1kAXJa40VPameiitzwAIrNPWsr5kDdviN59xv5x8yYJM/edit#gid=0)

[Recorded Data](https://docs.google.com/spreadsheets/d/1Yr3d8pXQllWeyCImAO6XK0u_NgsKmbjGjc9SEn9wM3I/edit#gid=87589501)