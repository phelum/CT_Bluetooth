	CubieTruck changes to download firmware and enable Bluetooth

The AP6210 is the wifi/bluetooth IC on the CubieTruck.  The bluetooth portion
of this IC emulates or is a BCM20710.  This requires a firmware download.

Broadcom supplies the firmware and a download program.  With kernel 3.4 this
download process was less than reliable.  I modified this program and added
a pre-download script which made the process reliable.


The download program asserts RTS when starting and waits for CTS before each send
to the BCM20710.  It also sets the line discipline to TTY before downloading
the firmware.  It also times out and continues if a reply is not received
within a reasonable time.  There is one command in particular (the "start
download" command) where frequently no reply is received.


The pre-download script sets the BT_WAKE pin on the AP6210 (probably irrelevant)
and toggles the BT_REST pin.  This toggling resets the BCM20710.  For kernel 3.4
this script requires that the script.bin defines gpio pins 68 and 69.

<pre>
[gpio_para]
gpio_used = 1
gpio_num = 69
...
gpio_pin_68 = port:PH18&lt;0>&lt;default>&lt;default>&lt;0>
gpio_pin_69 = port:PH24&lt;0>&lt;default>&lt;default>&lt;0>
</pre>


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

An alternative provided here is a "reset" program that can be run before the
download program.  This "reset" program by default uses system access via /dev/mem.
This can be changed to use an external script if desired.

The options are all specified in the bt.load script.  As supplied it will run
the "reset" program which uses /dev/mem to reset the device, then run the "patch"
program to download the firmware.  The bt.load script can be changed so the
"reset" program will use an external script rather than /dev/mem to reset the
device.  Or the bt.load script can be changed so it just calls the "patch"
program and gets it to reset the device as well as download the firmware.


With the 3.4 kernel, the Bluetooth UART is accessed via /dev/ttyS1.  With the 
3.19 kernel and the standard .dtb (sun7i-a20-cubietruck.dtb), the UART is
accessed via /dev/ttyS2.  With the 3.19 kernel and the .dtb supplied in this
repository, the UART is accessed via /dev/ttyS1.  Two files contain references
to the serial port and these might need to be changed.  The files are

<pre>
a) /etc/bluetooth/uart
b) /usr/local/bin/bt.load
</pre>

Both these files reference /dev/ttyS1 and both might need to be changed to
/dev/ttyS2.


<pre>
The procedure (callout method) is:
a) The /etc/init.d/bluetooth script calls /usr/local/bin/bt.load.
b) bt.load determines parameters to use and calls the patchram program.
c) patchram opens the serial port and sets the line parameters.
d) patchram calls the bt.init script which resets the BCM20710.
e) patchram then downloads the firmware.
</pre>

<pre>
The procedure (new method) is:
a) The /etc/init.d/bluetooth script calls /usr/local/bin/bt.load.
b) bt.load determines parameters to use and calls the reset program.
c) The reset program uses /dev/mem or a script to reset the device.
d) bt.load calls the patchram program.
e) patchram opens the serial port and sets the line parameters.
f) patchram then downloads the firmware.
</pre>


The new approach (reset program) means it is almost possible to use the
standard Broadcom patchram program.  But it often hangs waiting for a reply
to the "start download" command.

The patchram console output is redirected to /tmp/trace.  Without debug (set in
the bt.load script) this output is minimal.  With debug it lists each transmit
and receive.

2016-04-11: The UART read procedure has been changed to cater for unwanted null
bytes that sometimes appear when using ttyAMA0 on a Raspberry Pi 3.
