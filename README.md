	CubieTruck changes to download firmware and enable Bluetooth

The AP6210 is the wifi/bluetooth IC on the CubieTruck.  The bluetooth portion
of this IC emulates or is a BCM20710.  This requires a firmware download.

Broadcom supplies the firmware and a download program.  With kernel 3.4 this
download process was less than reliable.  I modified this program and added
a pre-download script which made the process reliable.


The download program asserts RTS when starting waits for CTS before each send
to the BCM20710.  It also sets the line discipline to TTY before downloading
the firmware.


The pre-download script sets the BT_WAKE pin on the AP6210 (probably irrelevant)
and toggles the BT_REST pin.  This toggling resets the BCM20710.  For kernel 3.4
this script requires that the script.bin defines gpio pins 68 and 69.

[gpio_para]
gpio_used = 1
gpio_num = 69
...
gpio_pin_68 = port:PH18<0><default><default><0>
gpio_pin_69 = port:PH24<0><default><default><0>


With kernel 3.19 the process failed and has now been changed so it works
reliably with both kernels 3.4 and 3.19.  The change is that the pre-download
script is now called after the serial port is opened.  This is done by a 
"system" call in the download program and has the unfortunate side-effect that
the two steps (reset and download) are inextricably tied together and the
standard download program can never be used.  It appears that CTS on the 
BCM20710 must be asserted (held low) at the end of the reset pulse to get the
device to respond to serial communication.  The relevant UART on the A20 SoC
is uart2 and its RTS output connects to the BCM20710 CTS.

My assumption here is that the old kernel left RTS asserted by default whereas
the new kernel negates it whenever the port is not open.

Another difference between kernels 3.4 and 3.19 is that uart2 is /dev/ttyS1 with
kernel 3.4 and is /dev/ttyS2 with kernel 3.19.  The scripts here determine the
kernel version and react appropriately.


The procedure now is:
a) The /etc/init.d/bluetooth script calls /usr/local/bin/bt.load.
b) bt.load determines which /dev/ttyS? to use and calls the patchram program.
c) patchram opens the serial port and sets the line parameters.
d) patchram calls the bt.init script which resets the BCM20710.
e) patchram then downloads the firmware.


The init.d/bluetooth script now enables hci using ttyS1 or ttyS2 rather than
using the device specified in the /etc/bluetooth/uart file.

The patchram console output is redirected to /tmp/trace.  Without debug (set in
the bt.load script) this output is minimal.  With debug it lists each transmit
and receive.
