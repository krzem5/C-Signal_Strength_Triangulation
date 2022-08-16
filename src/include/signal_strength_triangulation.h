#ifndef __SIGNAL_STRENGTH_TRIANGULATION_H__
#define __SIGNAL_STRENGTH_TRIANGULATION_H__ 1
#include <stdint.h>



typedef uint8_t antena_count_t;



typedef int32_t distance_t;



typedef struct _ANTENA_LOCATION{
	distance_t x;
	distance_t y;
	distance_t z;
} antena_location_t;



typedef struct _TRIANGULATION_STATE{
	antena_count_t count;
	antena_location_t* data;
	distance_t _base_x;
	distance_t _base_y;
	distance_t _base_z;
	distance_t _base_error;
} triangulation_state_t;



typedef struct _RECEIVER_LOCATION{
	distance_t x;
	distance_t y;
	distance_t z;
	distance_t error;
} receiver_location_t;



void free_triangulation_state(triangulation_state_t* state);



void triangulate_antenas(antena_count_t count,const distance_t* antena_signals,triangulation_state_t* state);



void triangulate_point(const triangulation_state_t* state,const distance_t* antena_signals,receiver_location_t* out);



#endif
