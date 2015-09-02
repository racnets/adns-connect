/*
 * SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2007  MontaVista Software, Inc.
 * Copyright (c) 2007  Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * Cross-compile with cross-gcc -I/path/to/cross-kernel/include
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>			//fopen, fprintf, printf
#include <stdlib.h>
#include <string.h>			//memcpy,
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include "adns.h"

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static uint8_t bits = 8;
static uint16_t delay = 75;
static const char *device = "/dev/spidev0.0";
static uint8_t mode = SPI_CPHA | SPI_CPOL;
static uint32_t speed = 500000;
static uint8_t verbose = 0;

adns3080_t adns;

int SPI_read_byte(int fd, uint8_t addr, uint8_t *value) {
	int n = 1;
	struct spi_ioc_transfer tr[2] = {{0},};
    
   	uint8_t tx[1] = {addr};
	uint8_t rx[1] = {0};

	tr[0].tx_buf = (unsigned long)tx;
	tr[0].rx_buf = (unsigned long)NULL;
	tr[0].len = n;
	tr[0].delay_usecs = delay;
	tr[0].speed_hz = speed;
	tr[0].bits_per_word = bits;

	tr[1].tx_buf = (unsigned long)NULL;
	tr[1].rx_buf = (unsigned long)rx;
	tr[1].len = n;
	tr[1].delay_usecs = delay;
	tr[1].speed_hz = speed;
	tr[1].bits_per_word = bits;

	int ret;
	ret = ioctl(fd, SPI_IOC_MESSAGE(2), &tr);
	if (ret < 1) pabort("can't send spi message");

	*value = rx[0];
	
	if (verbose > 1) {
		printf("spi read byte at address: %.2x = %.2x\n", addr, rx[0]);	
	}
	return ret;
}

int SPI_write_byte(int fd, uint8_t addr, uint8_t value) {
	// write
	struct spi_ioc_transfer tr[1] = {{0},};
    
   	uint8_t tx[2] = {addr, value};

	tr[0].tx_buf = (unsigned long)tx;
	tr[0].rx_buf = (unsigned long)NULL;
	tr[0].len = 2;
	tr[0].delay_usecs = delay;
	tr[0].speed_hz = speed;
	tr[0].bits_per_word = bits;

	int ret;
	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) pabort("can't send spi message");

	if (verbose > 1) {
		printf("spi write byte %.2x to address %.2x\n", value, addr);	
	}
	return ret;
}

int ADNS_read_motion_burst(int fd) {
	struct spi_ioc_transfer tr[2] = {{0},};
	uint8_t addr = 0x50;
    
   	uint8_t tx[1] = {addr};
	uint8_t rx[7] = {0};

	tr[0].tx_buf = (unsigned long)tx;
	tr[0].rx_buf = (unsigned long)NULL;
	tr[0].len = 1;
	tr[0].delay_usecs = delay;
	tr[0].speed_hz = speed;
	tr[0].bits_per_word = bits;

	tr[1].tx_buf = (unsigned long)NULL;
	tr[1].rx_buf = (unsigned long)rx;
	tr[1].len = 7;
	tr[1].delay_usecs = delay;
	tr[1].speed_hz = speed;
	tr[1].bits_per_word = bits;

	int ret;
	ret = ioctl(fd, SPI_IOC_MESSAGE(2), &tr);
	if (ret < 1) pabort("can't send spi message");

	adns.motion_val		= rx[0];
	adns.delta_X    	= (int8_t)rx[1];
	adns.delta_Y    	= (int8_t)rx[2];
	adns.squal		= 4*rx[3];
	adns.shutter		= (rx[4] << 8) | rx[5];
	adns.maximum_pixel	= rx[6];

	if (verbose) {
		printf("spi read motion burst from address %.2x\n", addr);
		if (verbose > 1) {
			int i;
			printf("\treceived:");
			for (i = 0; i < tr[1].len; i++) {
				if (!(i % 8)) printf("\n\t\t");
				printf("%.2x ", rx[i]);
			}
			printf("\n");
		}
		printf("\tmotion MOT %d\n", adns.motion.MOT);
		printf("\tmotion OVF %d\n", adns.motion.OVF);
		printf("\tmotion RES %d\n", adns.motion.RES);
		printf("\tdelta_x %d\n", adns.delta_X);
		printf("\tdelta_y %d\n", adns.delta_Y);
		printf("\tSQUAL %d\n", adns.squal);
		printf("\tmaximum_pixel %d\n", adns.maximum_pixel);
		printf("\tshutter %d\n", adns.shutter);
	}
	return ret;
}

int ADNS_read_frame_burst(int fd, uint8_t * frame) {
	int ret;
	if (verbose) printf("get adns raw values\n");

	// read frame period
	uint8_t _valLower;
	uint8_t _valUpper;
	ret = SPI_read_byte(fd, 0x11, &_valUpper);
	if (ret < 1) return ret;
	ret = SPI_read_byte(fd, 0x10, &_valLower);
	if (ret < 1) return ret;
	adns.frame_period 	= (_valUpper << 8) | _valLower;

	// write frame capture register
	ret = SPI_write_byte(fd, 0x80 | 0x13, 0x83);
	if (ret < 1) return ret;
	
	// wait 10us + 3 frame periods
	usleep(3 * adns.frame_period);

	// read pixel dump register
	struct spi_ioc_transfer tr[2] = {{0},};
	uint8_t addr = 0x40;
    
   	uint8_t tx[1] = {addr};
	uint8_t rx[1024] = {};

	tr[0].tx_buf = (unsigned long)tx;
	tr[0].rx_buf = (unsigned long)NULL;
	tr[0].len = 1;
	tr[0].delay_usecs = delay;
	tr[0].speed_hz = speed;
	tr[0].bits_per_word = bits;

	tr[1].tx_buf = (unsigned long)NULL;
	tr[1].rx_buf = (unsigned long)rx;
	tr[1].len = 1024;
	tr[1].delay_usecs = delay;
	tr[1].speed_hz = speed;
	tr[1].bits_per_word = bits;

	ret = ioctl(fd, SPI_IOC_MESSAGE(2), &tr);
	if (ret < 1) pabort("can't send spi message");

	if (verbose) printf("\tread %d bytes\n", ret-1);
	
	// release and copy 6bit pixel values 
	int l=0;
	for (l = 0; l < 900; l++) {
		frame[l] = rx[l] & ((1<<6)-1);
	}
	
	if (verbose > 1) {
		int i;
		printf("\treceived:");
		for (i = 0; i < tr[1].len; i++) {
			if (!(i % 8)) printf("\n\t\t");
				printf("%.2X ", rx[i]);
		}
		printf("\n");
	}
	
	return ret;
}

int ADNS_read_all(int fd) {
	int ret;
        
	if (verbose) printf("read all essential adns values\n");
	
	// read motion, delta_X, delta_Y, squal, shutter, maximum_pixel
	ret = ADNS_read_motion_burst(fd);
	if (ret < 1) return ret;

	// read frame period
	uint8_t _valLower;
	uint8_t _valUpper;
	ret = SPI_read_byte(fd, 0x11, &_valUpper);
	if (ret < 1) return ret;
	ret = SPI_read_byte(fd, 0x10, &_valLower);
	if (ret < 1) return ret;
	adns.frame_period 	= (_valUpper << 8) | _valLower;

	// read product id
	ret = SPI_read_byte(fd, 0x00, &(adns.product_ID));
	if (ret < 1) return ret;
	
	// read revision
	ret = SPI_read_byte(fd, 0x01, &(adns.revision));
	if (ret < 1) return ret;
	
	// read pixel_sum
	ret = SPI_read_byte(fd, 0x06, &(adns.pixel_sum));
	if (ret < 1) return ret;
	
	// read inv_product_ID
	ret = SPI_read_byte(fd, 0x3f, &(adns.inv_product_ID));
	if (ret < 1) return ret;
	
	if (verbose > 1) {
		printf("\n");
		printf("product_ID 0x%x\n", adns.product_ID);
		printf("inverse product_ID 0x%x\n", adns.inv_product_ID);
		printf("revision_ID 0x%x\n", adns.revision);
		printf("pixel_sum %d\n", adns.pixel_sum);
		printf("frame_period %d - %f Hz\n", adns.frame_period, 24E6/adns.frame_period);
	}
	
	return ret;
}

int ADNS_get_FPS_bounds(int fd) {
	int ret;

	if (verbose) printf("get frame period and shutter bounds\n");

	// read frame period max
	uint8_t _valLower;
	uint8_t _valUpper;
	ret = SPI_read_byte(fd, 0x1a, &_valUpper);
	if (ret < 1) return ret;
	ret = SPI_read_byte(fd, 0x19, &_valLower);
	if (ret < 1) return ret;
	adns.frame_period_max = (_valUpper << 8) | _valLower;
        
	// read frame period min
	ret = SPI_read_byte(fd, 0x1c, &_valUpper);
	if (ret < 1) return ret;
	ret = SPI_read_byte(fd, 0x1b, &_valLower);
	if (ret < 1) return ret;
	adns.frame_period_min = (_valUpper << 8) | _valLower;
        
	// read shutter max
	ret = SPI_read_byte(fd, 0x1e, &_valUpper);
	if (ret < 1) return ret;
	ret = SPI_read_byte(fd, 0x1d, &_valLower);
	if (ret < 1) return ret;
	adns.shutter_max = (_valUpper << 8) | _valLower;
        
	if (verbose) {
		printf("\tframe_period_max %d\n", adns.frame_period_max);
		printf("\tframe_period_min %d\n", adns.frame_period_min);
		printf("\tshutter_max %d\n", adns.shutter_max);
	}
	
	return ret;
}

int ADNS_set_FPS_bounds(int fd, int shutter) {
	int ret;
        
	if (verbose) printf("set frame period bounds for max shutter of %d\n", shutter);

	adns.shutter_max	= shutter;
	adns.frame_period_min	= 0x0e7e;
	adns.frame_period_max	= adns.frame_period_min	+ shutter;

	uint8_t fpmaxbl	= adns.frame_period_max;
	uint8_t fpmaxbu	= adns.frame_period_max >> 8;
	uint8_t fpminbl	= adns.frame_period_min;
	uint8_t fpminbu = adns.frame_period_min >> 8;
	uint8_t smaxbl 	= adns.shutter_max;
	uint8_t smaxbu 	= adns.shutter_max >> 8;

	int unsuccessful_change_count = 0;
	do {
		// wait for sensor to be ready
		int busy_count = 0;
		int _verbose = verbose;
		verbose = 0;
		do {
			ADNS_get_ext_conf(fd);
			busy_count++;
//			usleep(100000);
		} while (adns.ext_config.busy && (busy_count < 1000));
		verbose = _verbose;

		if (adns.ext_config.busy) {
			printf("\twarning: sensor busy!\n");
		}

		// disable automatic shutter mode
		// enable fixed frame rate
		ADNS_set_ext_conf(fd, 0x03);

		// set frame period min
		ret = SPI_write_byte(fd, 0x80 | 0x1b, fpminbl);
		if (ret < 1) return ret;
		ret = SPI_write_byte(fd, 0x80 | 0x1c, fpminbu);
		if (ret < 1) return ret;

		// set shutter max
		ret = SPI_write_byte(fd, 0x80 | 0x1d, smaxbl);
		if (ret < 1) return ret;
		ret = SPI_write_byte(fd, 0x80 | 0x1e, smaxbu);
		if (ret < 1) return ret;

		// set frame period maximum
		// - needs to be the last of the 3 registers to be written to
		// - write activates all new values of the 3 registers
		ret = SPI_write_byte(fd, 0x80 | 0x19, fpmaxbl);
		if (ret < 1) return ret;
//		usleep(100000);
		ret = SPI_write_byte(fd, 0x80 | 0x1a, fpmaxbu);
		if (ret < 1) return ret;

		// sensor needs some time to implement the new settings
		usleep(100000);

		// check for new settings to be active
		ADNS_read_motion_burst(fd);
	
		unsuccessful_change_count++;

	} while ((adns.shutter_max != adns.shutter) && (unsuccessful_change_count < 100));
	printf("needed %d try(s) to implement new setting\n", unsuccessful_change_count);

	// enable automatic shutter mode
	// disable fixed frame rate
	ADNS_set_ext_conf(fd, 0x00);

	if (adns.shutter_max != adns.shutter) {
		printf("\twarning: can't implement new setting!\n");
	}

	return ret;
}

int ADNS_get_ext_conf(int fd) {
	int ret;

	if (verbose) printf("get extended configuration\n");

	ret = SPI_read_byte(fd, 0x0b, &(adns.ext_config_val));
	if (ret < 1) return ret;
        
	if (verbose) {
		printf("\tbusy %d\n", adns.ext_config.busy);
		printf("\tNPU %d\n", adns.ext_config.Serial_NPU);
		printf("\tNAGC %d\n", adns.ext_config.NAGC);
		printf("\tfixed FR %d\n", adns.ext_config.Fixed_FR);
	}
	
	return ret;
}

int ADNS_set_ext_conf(int fd, uint8_t config) {
	int ret;
	
	if (verbose) printf("set extended configuration byte: %.2x\n", config);

	ret = SPI_write_byte(fd, 0x80 | 0x0b, config);

	return ret;
}

int ADNS_set_conf(int fd, uint8_t config) {
	int ret;
	
	if (verbose) printf("set configuration byte: %.2x\n", config);

	ret = SPI_write_byte(fd, 0x80 | 0x0a, config);

	return ret;
}

static void parse_opts(int argc, char *argv[])
{
	// reset getopt
	optind=1;
	//forbid error message
	opterr=0;

	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "verbose", 0, 0, 'v' },
			{ "werbose", 0, 0, 'w' },
			{ "speed",   1, 0, 's' },
			{ "delay",   1, 0, 'd' },
			{ "bpw",     1, 0, 'b' },
			{ "loop",    0, 0, 'l' },
			{ "cpha",    0, 0, 'H' },
			{ "cpol",    0, 0, 'O' },
			{ "lsb",     0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire",   0, 0, '3' },
			{ "no-cs",   0, 0, 'N' },
			{ "ready",   0, 0, 'R' },
			{ NULL, 0, 0, 0 },
		};

		int c;
		c = getopt_long(argc, argv, "d:b:D:s:CHlLNORvw3", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'w':
			verbose = 2;
			break;
		case 'v':
			verbose = 1;
			break;
		case 'D':
			device = optarg;
			break;
		case 's':
			speed = atoi(optarg);
			break;
		case 'd':
			delay = atoi(optarg);
			break;
		case 'b':
			bits = atoi(optarg);
			break;
		case 'l':
			mode |= SPI_LOOP;
			break;
		case 'H':
			mode |= SPI_CPHA;
			break;
		case 'O':
			mode |= SPI_CPOL;
			break;
		case 'L':
			mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			mode |= SPI_CS_HIGH;
			break;
		case '3':
			mode |= SPI_3WIRE;
			break;
		case 'N':
			mode |= SPI_NO_CS;
			break;
		case 'R':
			mode |= SPI_READY;
			break;
		default:;
		}
	}
}

int init_SPI(int* file, int argc, char *argv[]) {
	int ret;
	int fd;

	parse_opts(argc, argv); 

	fd = open(device, O_RDWR);
	if (fd < 0)
		pabort("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	if (verbose > 1) {
		printf("spi mode: %d\n", mode);
		printf("bits per word: %d\n", bits);
		printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
	}
	
	*file = fd;
	return ret;
}
