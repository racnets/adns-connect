/*
 * adns.h
 */

#ifndef ADNS_H_
#define ADNS_H_

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
extern adns3080_t adns;

int ADNS_read_motion_burst(int fd);
int ADNS_read_frame_burst(int fd, uint8_t * frame);
int ADNS_read_all(int fd);
int ADNS_get_FPS_bounds(int fd);
int ADNS_set_FPS_bounds(int fd, int shutter);
int ADNS_get_ext_conf(int fd);
int ADNS_set_ext_conf(int fd, uint8_t config);
int ADNS_set_conf(int fd, uint8_t config);

int init_SPI(int* file, int argc, char *argv[]);

#endif /* ADNS_H_ */
