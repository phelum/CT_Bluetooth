/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * This program resets the Bluetooth portion of the AP6210 in a CubieTruck
 * so it enables UART mode and will allow firmware download by the
 * patchram program.
 *
 * - Steven Saunderson (check <http://phelum.net/> for contact details).
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>
#include <sys/termios.h>
#include <sys/ioctl.h>

#define SW_PORT_IO_BASE  0x01c20800
#define PH_CFG2 0x104 / 4
#define PH_CFG3 0x108 / 4
#define PH_DATA 0x10C / 4


int acquire_uart (char *device_id)
{
	int 			uart_fd = -1;
	struct termios	termios;

	if ((uart_fd = open (device_id, O_RDWR | O_NOCTTY )) == -1) {
		fprintf (stderr, "port %s could not be opened, error %d\n",
					device_id, errno);
		exit (-1);
	}

	tcflush   (uart_fd, TCIOFLUSH);
	tcgetattr (uart_fd, &termios);

	termios.c_iflag = 0;
	termios.c_oflag = 0;
	termios.c_cflag = 0;
	termios.c_cflag |= B115200 | CS8 | CREAD | CLOCAL;
	termios.c_lflag = 0;

	cfsetispeed (&termios, B115200);
	cfsetospeed (&termios, B115200);

	tcsetattr (uart_fd, TCSANOW, &termios);
	tcflush   (uart_fd, TCIOFLUSH);

	int		status = TIOCM_RTS;
	ioctl (uart_fd, TIOCMBIS, &status);				// set RTS bit.

    return uart_fd;
} 


int fiddle_pins ()
{
	unsigned int * pc;
	int fd;
	unsigned int addr_start, addr_offset, PageSize, PageMask;
	unsigned char *ucptr;
	unsigned long *ulptr, data;

	PageSize = sysconf (_SC_PAGESIZE);
	PageMask = (~(PageSize-1));
	addr_start = SW_PORT_IO_BASE & PageMask;
	addr_offset = SW_PORT_IO_BASE & ~PageMask;

	fd = open ("/dev/mem", O_RDWR);
	if (fd < 0) {
		perror ("Unable to open /dev/mem");
		return (-1);
	}

	pc = mmap (0, PageSize*2, PROT_READ | PROT_WRITE, MAP_SHARED, fd, addr_start);

	if (pc == MAP_FAILED) {
		perror ("Unable to mmap file");
		printf ("pc:%lx\n", (unsigned long) pc);
		return (-2);
	}

	ucptr = ((unsigned char *) pc) + addr_offset;
	ulptr = (unsigned long *) ucptr;

	data = ulptr [PH_CFG2];
	data &= 0xFFFFF0FF;
	data |= 0x00000100;							// PH18 (BT_REST) is output
	ulptr [PH_CFG2] = data;

	data = ulptr [PH_CFG3];
	data &= 0xFFFFFFF0;
	data |= 0x00000001;							// PH24 (BT_WAKE) is output
	ulptr [PH_CFG3] = data;

	data = ulptr [PH_DATA];
	data &= ~((1 << 18) + (1 << 24));			// REST low, WAKE low
	ulptr [PH_DATA] = data;

	usleep (10000);								// wait 10ms

	data = ulptr [PH_DATA];
	data |= (1 << 18);							// REST high
	ulptr [PH_DATA] = data;

	close (fd);

	return 0;
}


int main (int argc, char *argv [])
{
	int 		uart_fd = -1;
	int			erc = 0;


	if (argc < 2) {
		fprintf (stderr,	"Need serial port device name in tail\n"
							"\n"
							"Tail: <serial port> [init script]\n"
							"serial port is normally /dev/ttyS1\n"
							"init script (e.g. bt.init) is script used to toggle BT pins\n"
							"if no init script, this program will toggle them using /dev/mem\n"
			);
		return 1;
	}

	if (0 > (uart_fd = acquire_uart (argv [1])))
		return 2;

	if (argc < 3)								// use /dev/mem ?
		erc = fiddle_pins ();
	else										// external script ?
		if (0 != (erc = system (argv [2])))
			fprintf (stderr, "Script result = %d\n", erc);

	usleep (30000);								// hold RTS for 30ms

	close (uart_fd);

	return erc;
  }


