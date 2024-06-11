# TashPad

An adapter to make a Super Famicom/NES controller appear to an ADB Macintosh as a Gravis GamePad, targeting the PIC12F1501 (8 pins, ~$0.96) microcontroller.

## Technical Details

The firmware translates the buttons by position, not by color.  The Select and Start buttons emulate the toggle switch on the Gravis GamePad, with Select moving it toward the directional pad and Start moving it toward the buttons.  The SFC controller's L and R buttons duplicate the functionality of the Y and X buttons, respectively.

The ADB protocol used by the Gravis Mac GamePad is documented [here](https://github.com/lampmerchant/tashnotes/blob/main/macintosh/adb/protocols/gravis_mac_gamepad.md).  The protocol used by the SFC controller is documented [here](https://gamesx.com/controldata/snesdat.htm).

### Building Firmware

Building the firmware requires Microchip MPASM, which is included with their development environment, MPLAB. Note that you must use MPLAB X version 5.35 or earlier or MPLAB 8 as later versions of MPLAB X have removed MPASM.
