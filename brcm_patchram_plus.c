/*******************************************************************************
 *
 *  Copyright (C) 2009-2011 Broadcom Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 *
 *  Modified by Steven Saunderson (phelum.net) for more reliable operation
 *  on A20 Cubietruck.
 *
 *  The download program asserts RTS when starting and waits for CTS before each send
 *  to the BCM20710.  It also sets the line discipline to TTY before downloading
 *  the firmware.
 *  
 *  The pre-download script sets the BT_WAKE pin on the AP6210 (probably irrelevant)
 *  and toggles the BT_REST pin.  This toggling resets the BCM20710.  For kernel 3.4
 *  this script requires that the script.bin defines gpio pins 68 and 69.
 *  
 *  [gpio_para]
 *  gpio_used = 1
 *  gpio_num = 69
 *  ...
 *  gpio_pin_68 = port:PH18<0><default><default><0>
 *  gpio_pin_69 = port:PH24<0><default><default><0>
 *  
 *  With kernel 3.19 the process failed and has now been changed so it works
 *  reliably with both kernels 3.4 and 3.19.  The change is that the pre-download
 *  script is now called after the serial port is opened.  This is done by a 
 *  "system" call in the download program and has the unfortunate side-effect that
 *  the two steps (reset and download) are inextricably tied together and the
 *  standard download program can never be used.  It appears that CTS on the 
 *  BCM20710 must be asserted (held low) at the end of the reset pulse to get the
 *  device to respond to serial communication.  The relevant UART on the A20 SoC
 *  is uart2 and its RTS output connects to the BCM20710 CTS.
 *  
 *  My assumption here is that the old kernel left RTS asserted by default whereas
 *  the new kernel negates it whenever the port is not open.
 *
 ******************************************************************************/

/*****************************************************************************
**                                                                           
**  Name:          brcm_patchram_plus.c
**
**  Description:   This program downloads a patchram files in the HCD format
**                 to Broadcom Bluetooth based silicon and combo chips and
**				   and other utility functions.
**
**                 It can be invoked from the command line in the form
**						<-d> to print a debug log
**						<--patchram patchram_file>
**						<--baudrate baud_rate>
**						<--bd_addr bd_address>
**						<--enable_lpm>
**						<--enable_hci>
**						<--use_baudrate_for_download>
**						<--scopcm=sco_routing,pcm_interface_rate,frame_type,
**							sync_mode,clock_mode,lsb_first,fill_bits,
**							fill_method,fill_num,right_justify>
**
**							Where
**
**							sco_routing is 0 for PCM, 1 for Transport,
**							2 for Codec and 3 for I2S,
**
**							pcm_interface_rate is 0 for 128KBps, 1 for
**							256 KBps, 2 for 512KBps, 3 for 1024KBps,
**							and 4 for 2048Kbps,
**
**							frame_type is 0 for short and 1 for long,
**
**							sync_mode is 0 for slave and 1 for master,
**
**							clock_mode is 0 for slabe and 1 for master,
**
**							lsb_first is 0 for false aand 1 for true,
**
**							fill_bits is the value in decimal for unused bits,
**
**							fill_method is 0 for 0's and 1 for 1's, 2 for
**								signed and 3 for programmable,
**
**							fill_num is the number or bits to fill,
**
**							right_justify is 0 for false and 1 for true
**
**						<--i2s=i2s_enable,is_master,sample_rate,clock_rate>
**
**							Where
**
**							i2s_enable is 0 for disable and 1 for enable,
**
**							is_master is 0 for slave and 1 for master,
**
**							sample_rate is 0 for 8KHz, 1 for 16Khz and
**								2 for 4 KHz,
**
**							clock_rate is 0 for 128KHz, 1 for 256KHz, 3 for
**								1024 KHz and 4 for 2048 KHz.
**
**						<--no2bytes skips waiting for two byte confirmation
**							before starting patchram download. Newer chips
**                          do not generate these two bytes.>
**						<--tosleep=number of microsseconds to sleep before
**							patchram download begins.>
**						uart_device_name
**
**                 For example:
**
**                 brcm_patchram_plus -d --patchram  \
**						BCM2045B2_002.002.011.0348.0349.hcd /dev/ttyHS0
**
**                 It will return 0 for success and a number greater than 0
**                 for any errors.
**
**                 For Android, this program invoked using a 
**                 "system(2)" call from the beginning of the bt_enable
**                 function inside the file 
**                 system/bluetooth/bluedroid/bluetooth.c.
**
**                 If the Android system property "ro.bt.bcm_bdaddr_path" is
**                 set, then the bd_addr will be read from this path.
**                 This is overridden by --bd_addr on the command line.
**  
******************************************************************************/

#include <stdio.h>
#include <getopt.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <stdlib.h>

#ifdef ANDROID
#include <termios.h>
#else
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <limits.h>
#endif

#include <string.h>
#include <signal.h>
#include <unistd.h>

#ifdef ANDROID
#include <cutils/properties.h>
#define LOG_TAG "brcm_patchram_plus"
#include <cutils/log.h>
#undef printf
#define printf LOGD
#undef fprintf
#define fprintf(x, ...) \
  { if(x==stderr) LOGE(__VA_ARGS__); else fprintf(x, __VA_ARGS__); }

#endif //ANDROID

#ifndef N_HCI
#define N_HCI	15
#endif

#define HCIUARTSETPROTO		_IOW('U', 200, int)
#define HCIUARTGETPROTO		_IOR('U', 201, int)
#define HCIUARTGETDEVICE	_IOR('U', 202, int)

#define HCI_UART_H4		0
#define HCI_UART_BCSP	1
#define HCI_UART_3WIRE	2
#define HCI_UART_H4DS	3
#define HCI_UART_LL		4

typedef unsigned char uchar;

int uart_fd = -1;
int hcdfile_fd = -1;
int termios_baudrate = 0;
int bdaddr_flag = 0;
int enable_lpm = 0;
int enable_hci = 0;
int use_baudrate_for_download = 0;
int debug = 0;
int scopcm = 0;
int i2s = 0;
int no2bytes = 0;
int tosleep = 0;
int baudrate = 0;
char	script_name [] = ""; // was "bt.init";

struct termios termios;

uchar buffer[1024];

uchar hci_reset[] = { 0x01, 0x03, 0x0c, 0x00 };

uchar hci_download_minidriver[] = { 0x01, 0x2e, 0xfc, 0x00 };

uchar hci_update_baud_rate[] = { 0x01, 0x18, 0xfc, 0x06, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00 };

uchar hci_write_bd_addr[] = { 0x01, 0x01, 0xfc, 0x06,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

uchar hci_write_sleep_mode[] = { 0x01, 0x27, 0xfc, 0x0c,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00,
	0x00, 0x00 };

uchar hci_write_sco_pcm_int[] =
	{ 0x01, 0x1C, 0xFC, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00 };

uchar hci_write_pcm_data_format[] =
	{ 0x01, 0x1e, 0xFC, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00 };

uchar hci_write_i2spcm_interface_param[] =
	{ 0x01, 0x6d, 0xFC, 0x04, 0x00, 0x00, 0x00, 0x00 };

uchar hci_write_uart_clock_setting_48Mhz[] =
	{ 0x01, 0x45, 0xfc, 0x01, 0x01 };

uchar reqd7 [] = { 0x04, 0x0e, 0x04, 0x00, 0x00, 0x00, 0x00 };    // changed after each send


int parse_patchram (char *optarg)
{
	char *p;

	if (!(p = strrchr(optarg, '.'))) {
		fprintf(stderr, "file %s not an HCD file\n", optarg);
		exit(3);
	}

	p++;

	if (strcasecmp("hcd", p) != 0) {
		fprintf(stderr, "file %s not an HCD file\n", optarg);
		exit(4);
	}

	if ((hcdfile_fd = open(optarg, O_RDONLY)) == -1) {
		fprintf(stderr, "file %s could not be opened, error %d\n", optarg, errno);
		exit(5);
	}

	return(0);
}

void BRCM_encode_baud_rate (uint baud_rate, uchar *encoded_baud)
{
	if (baud_rate == 0 || encoded_baud == NULL) {
		fprintf(stderr, "Baudrate not supported!");
		return;
	}

	encoded_baud [3] = (uchar)(baud_rate >> 24);
	encoded_baud [2] = (uchar)(baud_rate >> 16);
	encoded_baud [1] = (uchar)(baud_rate >> 8);
	encoded_baud [0] = (uchar)(baud_rate & 0xFF);
}

typedef struct {
	int baud_rate;
	int termios_value;
} tBaudRates;

tBaudRates baud_rates[] = {
	{ 115200, B115200 },
	{ 230400, B230400 },
	{ 460800, B460800 },
	{ 500000, B500000 },
	{ 576000, B576000 },
	{ 921600, B921600 },
	{ 1000000, B1000000 },
	{ 1152000, B1152000 },
	{ 1500000, B1500000 },
	{ 2000000, B2000000 },
	{ 2500000, B2500000 },
	{ 3000000, B3000000 },
#ifndef __CYGWIN__
	{ 3500000, B3500000 },
	{ 4000000, B4000000 }
#endif
};


int validate_baudrate (int baud_rate, int *value)
{
	unsigned int i;

	for (i = 0; i < (sizeof(baud_rates) / sizeof(tBaudRates)); i++) {
		if (baud_rates[i].baud_rate == baud_rate) {
			*value = baud_rates[i].termios_value;
			return(1);
		}
	}

	return(0);
}


int parse_baudrate (char *optarg)
{
	baudrate = atoi(optarg);

	if (validate_baudrate(baudrate, &termios_baudrate)) {
		BRCM_encode_baud_rate(baudrate, &hci_update_baud_rate[6]);
	} else {
		return(1);
	}

	return(0);
}


int parse_bdaddr (char *optarg)
{
	int bd_addr[6];
	int i;

	sscanf(optarg, "%02X:%02X:%02X:%02X:%02X:%02X",
		&bd_addr[5], &bd_addr[4], &bd_addr[3],
		&bd_addr[2], &bd_addr[1], &bd_addr[0]);

	for (i = 0; i < 6; i++) {
		hci_write_bd_addr[4 + i] = bd_addr[i];
	}

	bdaddr_flag = 1;

	return(0);
}


int parse_enable_lpm (char *optarg)
{
	enable_lpm = 1;
	return(0);
}


int parse_use_baudrate_for_download (char *optarg)
{
	use_baudrate_for_download = 1;
	return(0);
}


int parse_enable_hci (char *optarg)
{
	enable_hci = 1;
	return(0);
}


int parse_scopcm (char *optarg)
{
	int param[10];
	int ret;
	int i;

	ret = sscanf(optarg, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
		&param[0], &param[1], &param[2], &param[3], &param[4],
		&param[5], &param[6], &param[7], &param[8], &param[9]);

	if (ret != 10) {
		return(1);
	}

	scopcm = 1;

	for (i = 0; i < 5; i++) {
		hci_write_sco_pcm_int[4 + i] = param[i];
	}

	for (i = 0; i < 5; i++) {
		hci_write_pcm_data_format[4 + i] = param[5 + i];
	}

	return(0);
}


int parse_i2s (char *optarg)
{
	int param[4];
	int ret;
	int i;

	ret = sscanf(optarg, "%d,%d,%d,%d", &param[0], &param[1], &param[2],
		&param[3]);

	if (ret != 4) {
		return(1);
	}

	i2s = 1;

	for (i = 0; i < 4; i++) {
		hci_write_i2spcm_interface_param[4 + i] = param[i];
	}

	return(0);
}


int parse_no2bytes (char *optarg)
{
	no2bytes = 1;
	return(0);
}


int parse_tosleep (char *optarg)
{
	tosleep = atoi(optarg);

	if (tosleep <= 0) {
		return(1);
	}

	return(0);
}


void usage (char *argv0)
{
	printf("Usage %s:\n", argv0);
	printf("\t<-d> to print a debug log\n");
	printf("\t<--patchram patchram_file>\n");
	printf("\t<--baudrate baud_rate>\n");
	printf("\t<--bd_addr bd_address>\n");
	printf("\t<--enable_lpm>\n");
	printf("\t<--enable_hci>\n");
	printf("\t<--use_baudrate_for_download> - Uses the\n");
	printf("\t\tbaudrate for downloading the firmware\n");
	printf("\t<--scopcm=sco_routing,pcm_interface_rate,frame_type,\n");
	printf("\t\tsync_mode,clock_mode,lsb_first,fill_bits,\n");
	printf("\t\tfill_method,fill_num,right_justify>\n");
	printf("\n\t\tWhere\n");
	printf("\n\t\tsco_routing is 0 for PCM, 1 for Transport,\n");
	printf("\t\t2 for Codec and 3 for I2S,\n");
	printf("\n\t\tpcm_interface_rate is 0 for 128KBps, 1 for\n");
	printf("\t\t256 KBps, 2 for 512KBps, 3 for 1024KBps,\n");
	printf("\t\tand 4 for 2048Kbps,\n");
	printf("\n\t\tframe_type is 0 for short and 1 for long,\n");
	printf("\t\tsync_mode is 0 for slave and 1 for master,\n");
	printf("\n\t\tclock_mode is 0 for slabe and 1 for master,\n");
	printf("\n\t\tlsb_first is 0 for false aand 1 for true,\n");
	printf("\n\t\tfill_bits is the value in decimal for unused bits,\n");
	printf("\n\t\tfill_method is 0 for 0's and 1 for 1's, 2 for\n");
	printf("\t\tsigned and 3 for programmable,\n");
	printf("\n\t\tfill_num is the number or bits to fill,\n");
	printf("\n\t\tright_justify is 0 for false and 1 for true\n");
	printf("\n\t<--i2s=i2s_enable,is_master,sample_rate,clock_rate>\n");
	printf("\n\t\tWhere\n");
	printf("\n\t\ti2s_enable is 0 for disable and 1 for enable,\n");
	printf("\n\t\tis_master is 0 for slave and 1 for master,\n");
	printf("\n\t\tsample_rate is 0 for 8KHz, 1 for 16Khz and\n");
	printf("\t\t2 for 4 KHz,\n");
	printf("\n\t\tclock_rate is 0 for 128KHz, 1 for 256KHz, 3 for\n");
	printf("\t\t1024 KHz and 4 for 2048 KHz.\n\n");
	printf("\t<--no2bytes skips waiting for two byte confirmation\n");
	printf("\t\tbefore starting patchram download. Newer chips\n");
	printf("\t\tdo not generate these two bytes.>\n");
	printf("\t<--tosleep=microseconds>\n");
	printf("\tuart_device_name\n");
}


int parse_cmd_line (int argc, char **argv)
{
	int c;
	int ret = 0;

	typedef int (*PFI)();

	PFI parse[] = { parse_patchram, parse_baudrate,
		parse_bdaddr, parse_enable_lpm, parse_enable_hci,
		parse_use_baudrate_for_download,
		parse_scopcm, parse_i2s, parse_no2bytes, parse_tosleep};

	while (1) {
//		int this_option_optind = optind ? optind : 1;
		int option_index = 0;

		static struct option long_options[] = {
			{"patchram", 1, 0, 0},
			{"baudrate", 1, 0, 0},
			{"bd_addr", 1, 0, 0},
			{"enable_lpm", 0, 0, 0},
			{"enable_hci", 0, 0, 0},
			{"use_baudrate_for_download", 0, 0, 0},
			{"scopcm", 1, 0, 0},
			{"i2s", 1, 0, 0},
			{"no2bytes", 0, 0, 0},
			{"tosleep", 1, 0, 0},
			{0, 0, 0, 0}
		};

		c = getopt_long_only (argc, argv, "d", long_options,
				&option_index);

		if (c == -1) {
			break;
		}

		switch (c) {
			case 0:
				if (debug) {
					printf ("option %s",
						long_options[option_index].name);
					if (optarg)
						printf (" with arg %s", optarg);
					printf ("\n");
				}

				ret = (*parse[option_index])(optarg);

				break;
			case 'd':
				debug = 1;
				break;

			case '?':
				//nobreak
			default:
				usage(argv[0]);
				break;
		}

		if (ret) {
			usage(argv[0]);
			break;
		}
	}

	if (ret) {
		return(1);
	}

	if (optind < argc)
	  {
		if (debug)
			fprintf (stderr, "%s \n", argv [optind]);

		if ((uart_fd = open (argv [optind], O_RDWR | O_NOCTTY )) == -1)
		 {
			fprintf (stderr, "port %s could not be opened, error %d\n",
					argv [optind], errno);
		 }

		if (++optind < argc)						// helper program ??
			strcpy (script_name, argv [optind]);

	  }

	return(0);
}


void dump (uchar *out, int len)
  {
	int i;

	for (i = 0; i < len; i++)
	  {
		if (i && !(i % 16))
			fprintf(stderr, "\n");

		fprintf(stderr, "%02x ", out[i]);
	  }

	fprintf(stderr, "\n");

	return;
  }


void init_uart ()
  {
	tcflush (uart_fd, TCIOFLUSH);
	tcgetattr (uart_fd, &termios);

//		New brutal approach.  Reset all then set what we want.

	termios.c_iflag = 0;
	termios.c_oflag = 0;
	termios.c_cflag = 0;
	termios.c_cflag |= B115200 | CS8 | CREAD | CLOCAL;
	termios.c_lflag = 0;

	cfsetispeed (&termios, B115200);
	cfsetospeed (&termios, B115200);

	tcsetattr (uart_fd, TCSANOW, &termios);
	tcflush   (uart_fd, TCIOFLUSH);

    return;
  } 


int	AssertRTS		(int fd)
  {
	int status;

	status = TIOCM_RTS;
	ioctl (fd, TIOCMBIS, &status);			// set RTS bit.

	return 0;
  }	


int	NegateRTS		(int fd)
  {
	int status;

	status = TIOCM_RTS;
	ioctl (fd, TIOCMBIC, &status);			// clear RTS bit.

	return 0;
  }	


void proc_enable_tty ()
{
    int a = 0;

	if (ioctl (uart_fd, TIOCGETD, &a) < 0) {
		fprintf (stderr, "Can't get line discipline, errno = %d\n", errno);
		return;
	}
	if (debug)
	  fprintf (stderr, "Previous discipline = %d\n", a);

    int i = N_TTY;

	if (ioctl (uart_fd, TIOCSETD, &i) < 0) {
		fprintf (stderr, "Can't set line discipline, errno = %d\n", errno);
		return;
	}

	if (debug)
	  fprintf (stderr, "Done setting line discpline\n");
	if (ioctl (uart_fd, TIOCGETD, &a) < 0) {
		fprintf (stderr, "Can't get line discipline, errno = %d\n", errno);
		return;
	}
	if (debug)
	  fprintf (stderr, "Current discipline = %d\n", a);

//		With kernel 3.19 we can't reset the BCM20710 with RTS negated.
//		So we call the init script (if specified) now to do the job.
//		An alternative approach is to run a "reset" program before
//		this download program.

	AssertRTS (uart_fd);					// our RTS tells other end it's okay to send.
											// also tells the BCM20710 to use uart mode.

	if (script_name [0])
	  {
		int erc;

		if (debug)
		  fprintf (stderr, "Calling init script\n");

		erc = system (script_name);

		if (debug || erc)
		  fprintf (stderr, "Script result = %d\n", erc);
	  }

	return;
}


void wait_cts (int fd)
  {
	int status = TIOCM_RTS;

	ioctl (fd, TIOCMGET, &status);

	if (!(status & TIOCM_CTS))
	  {
		if (debug)
			fprintf (stderr, "Waiting CTS...");

		while (!(status & TIOCM_CTS))
		  {
			usleep (10000);
			ioctl (fd, TIOCMGET, &status);
		  }

		if (debug)
			fprintf (stderr, "done\n");
	  }

	return;
  }


void read_prep (int fd)
  {
	int status;

	AssertRTS (fd);						// our RTS tells other end
										// it's okay to send.
	ioctl (fd, TIOCMGET, &status);

	if (debug)
	  {
		fprintf (stderr, "RTS = %x\n", (status & TIOCM_RTS) > 0);
		fprintf (stderr, "CTS = %x\n", (status & TIOCM_CTS) > 0);
		fprintf (stderr, "DTR = %x\n", (status & TIOCM_DTR) > 0);
		fprintf (stderr, "DCD = %x\n", (status & TIOCM_CAR) > 0);
	  }

//	wait_cts (fd);						// probably futile

	return;
  }


int read_waitfor7 (int fd)
  {
	int oldbytes, bytes = 0, sleeps = 0, sleep_limit = 100;

	ioctl (fd, FIONREAD, &bytes);

	if ((oldbytes = bytes) < 7)
	  {
		if (debug)
			fprintf (stderr, "Waiting for 7 bytes...");
		while (bytes < 7)
		  {
			usleep (1000);

			ioctl (fd, FIONREAD, &bytes);
			if (bytes != oldbytes)
				if (debug)
					fprintf (stderr, "%d ", bytes);

			oldbytes = bytes;

			if (++sleeps < sleep_limit)
				continue;

			if (debug)
				fprintf (stderr, "Timed out awaiting %02x%02x%02x%02x%02x%02x%02x\n",
				reqd7 [0], reqd7 [1], reqd7 [2], reqd7 [3], reqd7 [4], reqd7 [5], reqd7 [6]);

			return 1; 
		  }
	  }

	if (debug)
		fprintf (stderr, "done\n");

	return 0;
  }


int read_event (int fd, uchar *buffer)
  {
	int i = 0;
	int len = 3;
	int count;

	read_prep (fd);
	if (read_waitfor7 (fd) > 0)
		return 1;

	while ((count = read(fd, &buffer[i], len)) < len)
	  {
		i += count;
		len -= count;
	  }

	i += count;
	len = buffer[2];

	while ((count = read(fd, &buffer[i], len)) < len)
	  {
		i += count;
		len -= count;
	  }

	if (debug)
	  {
		count += i;
		fprintf(stderr, "received %d\n", count);
		dump(buffer, count);
	  }
    
    if (memcmp (buffer, reqd7, 7))
	  {
        fprintf (stderr, "Expected %02x%02x%02x%02x%02x%02x%02x\n",
            reqd7 [0], reqd7 [1], reqd7 [2], reqd7 [3], reqd7 [4], reqd7 [5], reqd7 [6]);
        fprintf (stderr, "Rcvd     %02x%02x%02x%02x%02x%02x%02x\n",
            buffer [0], buffer [1], buffer [2], buffer [3], buffer [4], buffer [5], buffer [6]);
		return 2;
      }

	return 0;
  }


void hci_send_cmd (uchar *buf, int len)
  {
	wait_cts (uart_fd);

	if (debug)
	  {
		fprintf(stderr, "writing\n");
		dump(buf, len);
	  }

	write (uart_fd, buf, len);

    reqd7 [0] = 0x04;
    reqd7 [1] = 0x0e;
    reqd7 [2] = 0x04;
    reqd7 [3] = buf [0];
    reqd7 [4] = buf [1];
    reqd7 [5] = buf [2];
    reqd7 [6] = 0x00;

	return;
  }


void proc_reset ()
  {
	int		tries = 0;

	while (1)
	  {
		hci_send_cmd(hci_reset, sizeof(hci_reset));

		if (read_event (uart_fd, buffer) == 0)
			break;

		if (++tries < 4)
			continue;

		fprintf (stderr, "No reply to reset command\n");
		exit (1);
	  }

	if (tosleep)
		usleep (tosleep);

	return;
  }


void proc_patchram ()
{
	int len;

	//	We get a response here with kernel 3.4 but not with 3.19 ??
	//	We must let it time-out to avoid mismatch errors with subsequent replies.

	hci_send_cmd (hci_download_minidriver, sizeof (hci_download_minidriver));
	read_event (uart_fd, buffer);

	//	If error, no use trying again because we just get the same time-out.

	if (!no2bytes) {
		read (uart_fd, &buffer[0], 2);
	}

	usleep (20000);                              // wait before sending firmware

	while (read (hcdfile_fd, &buffer[1], 3) == 3) {
		buffer [0] = 0x01;

		len = buffer [3];

		read (hcdfile_fd, &buffer[4], len);

		hci_send_cmd (buffer, len + 4);

		read_event (uart_fd, buffer);
	}

	if (use_baudrate_for_download) {
		cfsetospeed(&termios, B115200);
		cfsetispeed(&termios, B115200);
		tcsetattr(uart_fd, TCSANOW, &termios);
	}

	proc_reset ();

	return;
}


void proc_baudrate ()
{

	if (baudrate > 3000000) {
		hci_send_cmd(hci_write_uart_clock_setting_48Mhz,
			sizeof(hci_write_uart_clock_setting_48Mhz));

		read_event(uart_fd, buffer);
	}

	hci_send_cmd(hci_update_baud_rate, sizeof(hci_update_baud_rate));

	read_event(uart_fd, buffer);

	cfsetospeed(&termios, termios_baudrate);
	cfsetispeed(&termios, termios_baudrate);
	tcsetattr(uart_fd, TCSANOW, &termios);

	if (debug) {
		fprintf(stderr, "Done setting baudrate\n");
	}
}


void proc_bdaddr ()
{
	hci_send_cmd(hci_write_bd_addr, sizeof(hci_write_bd_addr));

	read_event(uart_fd, buffer);
}


void proc_enable_lpm ()
{
	hci_send_cmd(hci_write_sleep_mode, sizeof(hci_write_sleep_mode));

	read_event(uart_fd, buffer);
}


void proc_scopcm ()
{
	hci_send_cmd(hci_write_sco_pcm_int,
		sizeof(hci_write_sco_pcm_int));

	read_event(uart_fd, buffer);

	hci_send_cmd(hci_write_pcm_data_format,
		sizeof(hci_write_pcm_data_format));

	read_event(uart_fd, buffer);
}


void proc_i2s ()
{
	hci_send_cmd(hci_write_i2spcm_interface_param,
		sizeof(hci_write_i2spcm_interface_param));

	read_event(uart_fd, buffer);
}


void proc_enable_hci ()
{
    int i = N_HCI;
	int proto = HCI_UART_H4;

	if (debug)
	  fprintf(stderr, "Setting line discpline...");
	if (ioctl(uart_fd, TIOCSETD, &i) < 0) {
		fprintf(stderr, "Can't set line discipline, errno = %d\n", errno);
		return;
	}

	if (ioctl(uart_fd, HCIUARTSETPROTO, proto) < 0) {
		fprintf(stderr, "Can't set hci protocol, errno = %d\n", errno);
		return;
	}

	if (debug)
	  fprintf(stderr, "done\n");

	return;
}


#ifdef ANDROID
void read_default_bdaddr ()
{
	int sz;
	int fd;

	char path[PROPERTY_VALUE_MAX];

	char bdaddr[18];
	int len = 17;
	memset(bdaddr, 0, (len + 1) * sizeof(char));

	property_get("ro.bt.bdaddr_path", path, "");
	if (path[0] == 0)
		return;

	fd = open(path, O_RDONLY);
	if (fd < 0) {
		fprintf(stderr, "open(%s) failed: %s (%d)", path, strerror(errno),
				errno);
		return;
	}

	sz = read(fd, bdaddr, len);
	if (sz < 0) {
		fprintf(stderr, "read(%s) failed: %s (%d)", path, strerror(errno),
				errno);
		close(fd);
		return;
	} else if (sz != len) {
		fprintf(stderr, "read(%s) unexpected size %d", path, sz);
		close(fd);
		return;
	}

	if (debug) {
		printf("Read default bdaddr of %s\n", bdaddr);
	}

	parse_bdaddr(bdaddr);
}
#endif


int main (int argc, char **argv)
{
#ifdef ANDROID
	read_default_bdaddr();
#endif

	if (parse_cmd_line (argc, argv)) {
		exit (1);
	}

	if (uart_fd < 0) {
		exit (2);
	}

	init_uart ();

	proc_enable_tty ();		// set discipline, reset device.

//	read_prep (uart_fd);

	proc_reset();

	if (use_baudrate_for_download) {
		if (termios_baudrate) {
			proc_baudrate();
		}
	}

	if (hcdfile_fd > 0) {
		proc_patchram();
	}

	if (termios_baudrate) {
		proc_baudrate();
	}

	if (bdaddr_flag) {
		proc_bdaddr();
	}

	if (enable_lpm) {
		proc_enable_lpm();
	}

	if (scopcm) {
		proc_scopcm();
	}

	if (i2s) {
		proc_i2s();
	}

	if (enable_hci) {
		proc_enable_hci();
	}

	fprintf (stderr, "F/W load done\n");
	exit (0);
}

