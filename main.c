#include <stdint.h>
#include <unistd.h>		//close, sleep
#include <stdio.h>		//fprintf, printf
#include <stdlib.h>		//atoi, atof
#include <string.h>		//strcmp
#include <getopt.h>		//getoptlong
#include <sys/time.h>	//gettimeofday

#include "adns.h"
#include "i2c.h"
#include "socket-server.h"

#define I2C_SLAVE_ADDRESS	0x18

static uint8_t automatic = 0;
static uint8_t socket = 0;
static const char *file = NULL;
static uint8_t i2c_log = 0;
static const char *i2c_dev = "/dev/i2c-0";
static uint8_t manual = 0;
//~ static uint16_t readAddr;
static uint8_t run;
static uint16_t shutter = 0;
static double time = 0;
static uint8_t verbose = 0;
static uint8_t res = 0;
static uint8_t grab = 0;
static uint8_t quit = 0;

double getTime() {
	struct timeval tp;
	gettimeofday( &tp, NULL );
	return tp.tv_sec + tp.tv_usec/1E6;
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-afimStvbVdDhlLsO3]\n", prog);
	puts(" general\n"
	     "  -f --file     log file to write to\n"
	     "  -g --grab     grab frame\n"
	     "  -i --i2c      additional i2c sensor\n"
	     "  -k --socket   write using socket\n"
	     "  -r --run      run\n"
	     "  -t --time     run time\n"
	     "  -v --verbose  be verbose\n"
	     "  -w --werbose  be wery verbose\n"
	     " ADNS specific\n"
	     "  -a --auto     set auto frame and shutter period\n"
	     "  -m --manual   set fixed frame and shutter period\n"
	     "  -S --shutter  set shutter period\n"
	     "  -X --highres  set resolution to high\n"
	     " SPI specific\n"
	     "  -b --bpw      bits per word \n"
	     "  -C --cs-high  chip select active high\n"
	     "  -d --delay    delay (usec)\n"
	     "  -D --device   device to use (default /dev/spidev0.0)\n"
	     "  -H --cpha     clock phase\n"
	     "  -l --loop     loopback\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -R --ready    \n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -O --cpol     clock polarity\n"
	     "  -3 --3wire    SI/SO signals shared\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	//forbid error message
	opterr=0;

	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "shutter", 1, 0, 'S' },
			{ "file",    1, 0, 'f' },
			{ "grab",    0, 0, 'g' },
			{ "help",    0, 0, 'h' },
			{ "i2c",     1, 0, 'i' },
			{ "socket",  0, 0, 'k' },
			{ "run",     0, 0, 'r' },
			{ "time",    1, 0, 't' },
			{ "verbose", 0, 0, 'v' },
			{ "werbose", 0, 0, 'w' },
			{ "manual",  0, 0, 'm' },
			{ "highres", 0, 0, 'X' },
			{ "auto",    0, 0, 'a' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:f:i:S:t:aghkmrvwX", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
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
			case 'X':
				res = 1;
				break;
			case 'm':
				manual = 1;
				break;
			case 'k':
				socket = 1;
				break;
			case 'f':
				file = optarg;
				break;
			case 'h':
				print_usage(argv[0]);
				break;
			default:;
		}
	}
}

int main(int argc, char *argv[])
{
	int ret;
	int fd;
	FILE* lfd = NULL;
	double t, t0;

	printf("\nADNS connect tool\n");
	
	if (argc < 2) {
		print_usage(argv[0]);
		return EXIT_SUCCESS;
	}
	
	parse_opts(argc, argv);

	ret = init_SPI(&fd, argc, argv);
	if (ret < 0) {
		printf("SPI initialization failed\n");
		return EXIT_SUCCESS;
	};
	
	if (manual) {
		printf("\tset frame and shutter period to maximum bounds\n");
		ADNS_set_ext_conf(fd,0x03);	
	}

	if (shutter) {
		printf("\tset shutter period maximum bounds: %d\n", shutter);
		ADNS_set_FPS_bounds(fd, shutter);
	}

	if (automatic) {
		printf("\tset auto frame and shutter period\n");
		ADNS_set_ext_conf(fd,0);	
	}
	
	if (res) {
		printf("\tset resolution to high\n");
		ADNS_set_conf(fd,0x10);	
	} else ADNS_set_conf(fd,0x00);	
		
	if (grab) {
		printf("waring: sensor needs to be manually reset after frame grabing is finished\n");
	}
	
	if (file != NULL) {
		printf("\tsave values to file: %s\n",file);
		// setup log file
		lfd = fopen(file, "w");
		fprintf(lfd, "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s", "t", "MOT", "dX", "dY", "SQUAL", "shut", "pxSum", "OVF", "RES", "valid");
		if (i2c_log) {
			fprintf(lfd, "\t%s\t%s\t%s\t%s\t%s", "servo", "bright 0", "bright 1", "bright 2", "bright 3");
			// init i2c 
			i2cInit(i2c_dev, I2C_SLAVE_ADDRESS);
		}
		fprintf(lfd, "\n");
	} else lfd = stdout;

	// socket server functionality 
	if (socket) {
		printf("\tsetup server socket\n");
		socket_server_init();

		while(1) {
			printf( "\tlisten\n");
			if (socket_server_wait_for_client() != SUCCESS) {
				socket_server_close();
				return EXIT_SUCCESS;
			}
			
			char *buffer = NULL;
			int size;
			while(1) {
				socket_server_receive(&buffer, &size);
				// parse received data
				if (size > 0) {
					if ((strcmp("quit", buffer) == 0)
						|| (strcmp("exit", buffer) == 0) 
						|| (strcmp("q", buffer) == 0)) { 
						quit = 1;
					} else if ((strcmp("grab", buffer) == 0)
						|| (strcmp("g", buffer) == 0)) { 
						grab = 1;
					};
					
					free(buffer);
				}

				if (quit) {
					if (verbose) printf("\t\tsocket: termination signal received\n");
					printf("\tconnection closed by client\n");
					quit = 0;
					client_socket_close();
					break;
				}
				if (grab) {
					if (verbose) printf("\t\tsocket: raw frame request received\n");

					uint8_t frame[900];
					if (ADNS_read_frame_burst(fd, frame) >= 900) {
						socket_server_send((char*)frame,900);
					} else {
						// capture failed
						if (verbose) printf("\t\traw frame capture failed\n");
						// send only ping answer
						socket_server_send((char*)frame,1);
					}
					grab=0;
				}
			}
		}
		socket_server_close();
		return EXIT_SUCCESS;
	}

	if (grab) {
		uint8_t frame[900];
		ADNS_read_frame_burst(fd, frame);
		close(fd);
		return EXIT_SUCCESS;
	}			
	
	ADNS_get_FPS_bounds(fd);
	t0 = getTime();

//	if (run) {
//		while(1) {
//			t = getTime();
//			ADNS_read_motion_burst(fd);
//			printf("%f\t%u\t%u\t%d\t%d\t\t%d\n", t - t0, adns.motion.MOT, adns.squal, adns.delta_X, adns.delta_Y, adns.shutter);		
//		}
//	}
	
	do {
		t = getTime();
		ADNS_read_motion_burst(fd);
		
		fprintf(lfd, "%f\t%u\t%d\t%d", t - t0, adns.motion.MOT, adns.delta_X, adns.delta_Y);	
		fprintf(lfd, "\t%d\t%d\t%d\t%d", adns.squal, adns.shutter, adns.pixel_sum, adns.motion.OVF);
		fprintf(lfd, "\t%d\t0x%x", adns.motion.RES, adns.product_ID + adns.inv_product_ID);
		
		if (i2c_log) {
			uint16_t servo_value = i2cReadW(0x32);
			uint16_t brightness_value[4];
			brightness_value[0] = i2cReadW(0x76);
			brightness_value[1] = i2cReadW(0x78);
			brightness_value[2] = i2cReadW(0x72);
			brightness_value[3] = i2cReadW(0x74);
			fprintf(lfd, "\t%u", servo_value);
			fprintf(lfd, "\t%u\t%u\t%u\t%u", brightness_value[0], brightness_value[1], brightness_value[2], brightness_value[3]);
		}
		fprintf(lfd, "\n");

		usleep(100000);			
//		usleep(adns.frame_period / 24);
	} while (((t - t0) < time) || run);
	
	if (lfd != NULL) fclose(lfd);
	close(fd);

	return EXIT_SUCCESS;
}
