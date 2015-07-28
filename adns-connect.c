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

#include "i2c.h"

#define I2C_SLAVE_ADDRESS	0x18

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static uint8_t automatic = 0;
static uint8_t bits = 8;
static uint16_t delay = 75;
static const char *device = "/dev/spidev0.0";
static const char *socket = NULL;
static const char *file = NULL;
static uint8_t i2c_log = 0;
static const char *i2c_dev = "/dev/i2c-0";
static uint8_t manual = 0;
static uint8_t mode;
static uint16_t readAddr;
static uint8_t run;
static uint16_t shutter = 0;
static uint32_t speed = 500000;
static double time = 0;
static uint8_t verbose = 0;
static uint8_t grab = 0;

typedef struct {
	uint8_t product_ID;
	uint8_t inv_product_ID;
	uint8_t revision;
    union {
		struct {
			uint8_t RES:1;
			uint8_t res3:3;
			uint8_t OVF:1;
			uint8_t res2:2;
			uint8_t MOT:1;
		} motion;
		uint8_t motion_val;
	};
    union {
		struct {
			uint8_t Fixed_FR:1;
			uint8_t NAGC:1;
			uint8_t Serial_NPU:1;
			uint8_t res:4;
			uint8_t busy:1;
		} ext_config;
		uint8_t ext_config_val;
	};
	int8_t delta_X;
	int8_t delta_Y;
	uint16_t squal;	
	uint8_t pixel_sum;
	uint8_t maximum_pixel;
	uint16_t shutter;
	uint16_t frame_period;
	uint16_t frame_period_max;
	uint16_t frame_period_min;
	uint16_t shutter_max;
} adns3080_t;

adns3080_t adns;

double getTime() {
	struct timeval tp;
	gettimeofday( &tp, NULL );
	return tp.tv_sec + tp.tv_usec/1E6;
}

void SPI_read_write(int fd, uint8_t* tx, uint8_t* rx, int n) {
	int ret;

    struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = n,
        .delay_usecs = delay,
        .speed_hz = speed,
        .bits_per_word = bits,
	};

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) pabort("can't send spi message");

	if (verbose) {
		printf("spi read/wrote %d of %d bytes\n",ret,n);	
		if (verbose > 1) {
			int i;
			printf("send:");
			for (i = 0; i < n; i++) {
				if (!(i % 8)) puts("");
				printf("%.2X ", tx[i]);
			}
			printf("\n");
			
			printf("received:");
			for (i = 0; i < n; i++) {
				if (!(i % 8)) puts("");
				printf("%.2X ", rx[i]);
			}
			printf("\n");
		}
	}
	
}

void read_ADNS_raw(int fd, uint8_t * frame) {

	if (verbose) printf("get adns raw values\n");

	// write frame capture register
	uint8_t tx[1024] = {0x93, 0x83, };
	uint8_t rx[sizeof(tx)] = {0,};
	SPI_read_write(fd, tx, rx, 2);

	// wait 10us + 3 frame periods
	usleep(3 * adns.frame_period);

	// read frame capture register
	// dummy action - otherwise the first pixel get lost
	// resets the internal counter right
	tx[0] = 0x13;
	tx[1] = 0x00;
	SPI_read_write(fd, tx, rx, 2);
	// read pixel dump address
	tx[0] = 0x40;
	tx[1] = 0x00;
	SPI_read_write(fd, tx, rx, 1024);
	
	// search for frame start pixel
	int i=0;
	for (i = 0; i < 1024; i++) {
		if (rx[i] >= 0xC0) break;
	}
	// release and copy 6bit pixel value 
	int l=0;
	for (l = 0; l < 900; l++) {
		frame[l] = rx[i+l] & ((1<<6)-1);
	}
	
	for (i = 0; i < 900; i++) {
		if (!(i % 8)) puts("");
		printf("%.2X ", frame[i]);
	}
	printf("\n");

}

void read_ADNS(int fd) {
	int ret;
        
	uint8_t tx[] = {
		0x00, 0x00, 0x01, 0x00, 
		0x02, 0x00, 0x03, 0x00, 
		0x04, 0x00, 0x05, 0x00,
		0x06, 0x00, 0x07, 0x00,
		0x0f, 0x00, 0x0e, 0x00,
		0x11, 0x00, 0x10, 0x00,
		0x3f, 0x00, 0x00, 0x00
	};

	uint8_t rx[28] = {0, };
       
    struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 28,
        .delay_usecs = delay,
        .speed_hz = speed,
        .bits_per_word = bits,
	};

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) pabort("can't send spi message");

	adns.product_ID 	= rx[3];
	adns.revision   	= rx[5];
	adns.motion_val		= rx[7];
	adns.delta_X    	= (int8_t)rx[9];
	adns.delta_Y    	= (int8_t)rx[11];
	adns.squal			= 4*rx[13];
	adns.pixel_sum  	= rx[15];
	adns.maximum_pixel	= rx[17];
	adns.shutter		= (rx[19] << 8) | rx[21];
	adns.frame_period 	= (rx[23] << 8) | rx[25];
	adns.inv_product_ID = rx[27];

	if (verbose) {
		printf("get adns values\n");
		if (verbose > 1) {
			for (ret = 0; ret < ARRAY_SIZE(rx); ret++) {
				if (!(ret % 8)) puts("");
				printf("%.2X ", rx[ret]);
			}
		}
		puts("");
		printf("\n");
		printf("product_ID 0x%x\n", adns.product_ID);
		printf("inverse product_ID 0x%x\n", adns.inv_product_ID);
		printf("revision_ID 0x%x\n", adns.revision);
		printf("motion MOT %d\n", adns.motion.MOT);
		printf("motion OVF %d\n", adns.motion.OVF);
		printf("motion RES %d\n", adns.motion.RES);
		printf("delta_x %d\n", adns.delta_X);
		printf("delta_y %d\n", adns.delta_Y);
		printf("SQUAL %d\n", adns.squal);
		printf("pixel_sum %d\n", adns.pixel_sum);
		printf("maximum_pixel %d\n", adns.maximum_pixel);
		printf("shutter %d\n", adns.shutter);
		printf("frame_period %d - %f Hz\n", adns.frame_period, 24E6/adns.frame_period);
	}
}

void get_ADNS_FPS_bounds(int fd) {
	int ret;
        
	uint8_t tx[] = {
		0x1a, 0x00, 0x19, 0x00, 
		0x1c, 0x00, 0x1b, 0x00, 
		0x1e, 0x00,	0x1d, 0x00,
		0x00, 0x00, 0x00, 0x00
	};

	uint8_t rx[16] = {0, };
       
    struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 16,
        .delay_usecs = delay,
        .speed_hz = speed,
        .bits_per_word = bits,
	};

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) pabort("can't send spi message");

	adns.frame_period_max	= (rx[3] << 8) | rx[5];
	adns.frame_period_min	= (rx[7] << 8) | rx[9];
	adns.shutter_max		= (rx[11] << 8) | rx[13];

	if (verbose) {
		printf("get frame period and shutter bounds\n");
		if (verbose > 1) {
			for (ret = 0; ret < ARRAY_SIZE(rx); ret++) {
				if (!(ret % 8)) puts("");
				printf("%.2X ", rx[ret]);
			}
		}
		puts("");
		printf("\n");
		printf("frame_period_max %d\n", adns.frame_period_max);
		printf("frame_period_min %d\n", adns.frame_period_min);
		printf("shutter_max %d\n", adns.shutter_max);
	}
}

void get_ADNS_ext_conf(int fd) {
	int ret;
        
	uint8_t tx[] = {
		0x0b, 0x00, 0x10, 0x00
	};

	uint8_t rx[4] = {0, };
       
    struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 4,
        .delay_usecs = delay,
        .speed_hz = speed,
        .bits_per_word = bits,
	};

    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) pabort("can't send spi message");

	adns.ext_config_val		= rx[3];

	if (verbose) {
		printf("get extended configuration\n");
		if (verbose > 1) {
			for (ret = 0; ret < ARRAY_SIZE(rx); ret++) {
				if (!(ret % 8)) puts("");
				printf("%.2X ", rx[ret]);
			}
		}
		puts("");
		printf("\n");
		printf("busy %d\n", adns.ext_config.busy);
		printf("NPU %d\n", adns.ext_config.Serial_NPU);
		printf("NAGC %d\n", adns.ext_config.NAGC);
		printf("fixed FR %d\n", adns.ext_config.Fixed_FR);
	}
}

void set_ADNS_FPS_bounds(int fd, int shutter) {
	int ret;
        
	adns.shutter_max		= shutter;
	adns.frame_period_min	= 3200;
	adns.frame_period_max	= 3200 + shutter;

	uint8_t fpmaxbl	= adns.frame_period_max;
	uint8_t fpmaxbu	= adns.frame_period_max >> 8;
	uint8_t fpminbl	= adns.frame_period_min;
	uint8_t fpminbu = adns.frame_period_min >> 8;
	uint8_t smaxbl 	= adns.shutter_max;
	uint8_t smaxbu 	= adns.shutter_max >> 8;

	uint8_t tx[] = {
		0x99, fpmaxbl, 0x9a, fpmaxbu,
		0x9b, fpminbl, 0x9c, fpminbu,
		0x9d, smaxbl,  0x9e, smaxbu
	};

	uint8_t rx[12] = {0, };
       
    struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 12,
        .delay_usecs = delay,
        .speed_hz = speed,
        .bits_per_word = bits,
	};

	do {
		get_ADNS_ext_conf(fd);
	} while (adns.ext_config.busy);
	
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) pabort("can't send spi message");
}

void set_ADNS_conf(int fd, uint8_t config) {
	uint8_t tx[] = {0x8a, config};
	uint8_t rx[sizeof(tx)] = {0,};

	SPI_read_write(fd, rx, rx, sizeof(tx));
}

void set_ADNS_ext_conf(int fd, uint8_t config) {
	uint8_t tx[] = {0x8b, config};
	uint8_t rx[sizeof(tx)] = {0,};

	SPI_read_write(fd, rx, rx, sizeof(tx));
}

int init_SPI(int* file) {
	int ret;
	int fd;
	
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

static void print_usage(const char *prog)
{
	printf("Usage: %s [-afimStvbVdDhlLsO3]\n", prog);
	puts(" ADNS specific\n"
	     "  -a --auto     set auto frame and shutter period\n"
	     "  -f --file     log file to write to\n"
	     "  -g --grab     grab frame\n"
	     "  -i --i2c      additional i2c sensor\n"
	     "  -k --socket   write using socket\n"
	     "  -m --manual   set fixed frame and shutter period\n"
	     "  -r --run      run\n"
//	     "  -r --read     read from address\n"
	     "  -S --shutter  set shutter period\n"
	     "  -t --time     run time\n"
	     "  -v --verbose  be verbose\n"
	     "  -w --werbose  be wery verbose\n"
	     " SPI specific\n"
	     "  -b --bpw      bits per word \n"
	     "  -C --cs-high  chip select active high\n"
	     "  -d --delay    delay (usec)\n"
	     "  -D --device   device to use (default /dev/spidev0.0)\n"
	     "  -H --cpha     clock phase\n"
	     "  -l --loop     loopback\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -O --cpol     clock polarity\n"
	     "  -3 --3wire    SI/SO signals shared\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "shutter", 1, 0, 'S' },
			{ "file",    1, 0, 'f' },
			{ "grab",    0, 0, 'g' },
			{ "i2c",     1, 0, 'i' },
			{ "socket",  1, 0, 'k' },
			//~ { "read",    1, 0, 'r' },
			{ "run",     0, 0, 'r' },
			{ "time",    1, 0, 't' },
			{ "verbose", 0, 0, 'v' },
			{ "werbose", 0, 0, 'w' },
			{ "manual",  0, 0, 'm' },
			{ "auto",    0, 0, 'a' },
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

		c = getopt_long(argc, argv, "b:d:D:f:i:k:s:S:t:aCgHlLmNOrRvw3", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		//~ case 'r':
			//~ readAddr = atoi(optarg);
			//~ if (!readAddr) {
				//~ char* end;
				//~ readAddr = strtol(optarg, &end, 16);
			//~ }
			//~ break;
		case 'r':
			run = 1;
			break;
		case 'g':
			grab = 1;
			break;
		case 'i':
			i2c_log = 1;
			i2c_dev = optarg;
			break;
		case 'w':
			verbose = 2;
			break;
		case 'v':
			verbose = 1;
			break;
		case 'a':
			automatic = 1;
			break;
		case 't':
			time = atof(optarg);
			break;
		case 'S':
			shutter = atoi(optarg);
			break;
		case 'm':
			manual = 1;
			break;
		case 'k':
			socket = optarg;
			break;
		case 'D':
			device = optarg;
			break;
		case 'f':
			file = optarg;
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
		default:
			print_usage(argv[0]);
			break;
		}
	}
}

int main(int argc, char *argv[])
{
	int ret;
	int fd;
	FILE* lfd = NULL;
	double t, t0;
	
	parse_opts(argc, argv);
	
	printf("\nADNS testing tool\n");

	ret = init_SPI(&fd);
		
	if (manual) {
		printf("set frame and shutter period to maximum bounds\n");
		set_ADNS_ext_conf(fd,3);	
	}

	if (shutter) {
		printf("set shutter period maximum bounds: %d\n", shutter);
		set_ADNS_FPS_bounds(fd, shutter);
	}

	if (automatic) {
		printf("set auto frame and shutter period\n");
		set_ADNS_ext_conf(fd,0);	
	}

	
	return ret;

	if (file != NULL) {
		printf("save values to file: %s\n",file);
		// setup log file
		lfd = fopen(file, "w");
		fprintf(lfd, "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s", "t", "MOT", "dX", "dY", "SQUAL", "shut", "pxSum", "OVF", "RES", "valid");
		if (i2c_log) {
			fprintf(lfd, "\t%s\t%s\t%s\t%s\t%s", "servo", "bright 0", "bright 1", "bright 2", "bright 3");
			// init i2c 
			i2cInit(i2c_dev, I2C_SLAVE_ADDRESS);
		}
		fprintf(lfd, "\n");
	}

	if (grab) {
		uint8_t frame[900];
		read_ADNS_raw(fd, frame);
		close(fd);
		return ret;
	}			
	
	get_ADNS_FPS_bounds(fd);
	t0 = getTime();

	if (run) {
		//~ set_ADNS_conf(fd,0x10);	
		set_ADNS_conf(fd,0x00);	
		while(1) {
			t = getTime();
			read_ADNS(fd);
			printf("%f\t%u\t%u\t%d\t%d\t\t%d\n", t - t0, adns.motion.MOT, adns.squal, adns.delta_X, adns.delta_Y, adns.shutter);		
			usleep(100000);			
		}
	}
	
	do {
		t = getTime();
		read_ADNS(fd);
		if (lfd != NULL) {
			uint16_t servo_value = i2cReadW(0x32);
			uint16_t brightness_value[4];
			brightness_value[0] = i2cReadW(0x76);
			brightness_value[1] = i2cReadW(0x78);
			brightness_value[2] = i2cReadW(0x72);
			brightness_value[3] = i2cReadW(0x74);

			fprintf(lfd, "%f\t%u\t%d\t%d", t - t0, adns.motion.MOT, adns.delta_X, adns.delta_Y);		
			fprintf(lfd, "\t%d\t%d\t%d\t%d", adns.squal, adns.shutter, adns.pixel_sum, adns.motion.OVF);		
			fprintf(lfd, "\t%d\t0x%x", adns.motion.RES, adns.product_ID + adns.inv_product_ID);
			if (i2c_log) {
				fprintf(lfd, "\t%u", servo_value);
				fprintf(lfd, "\t%u\t%u\t%u\t%u", brightness_value[0], brightness_value[1], brightness_value[2], brightness_value[3]);
			}
			fprintf(lfd, "\n");
		}
		usleep(adns.frame_period / 24);
	} while ((t - t0) < time);
	
	if (lfd != NULL) fclose(lfd);	
	close(fd);

	return ret;
}
