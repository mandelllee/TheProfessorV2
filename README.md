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