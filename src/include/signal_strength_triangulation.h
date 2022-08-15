#ifndef __SIGNAL_STRENGTH_TRIANGULATION_H__
#define __SIGNAL_STRENGTH_TRIANGULATION_H__ 1
#include <stdint.h>



typedef uint8_t antena_count_t;



typedef struct _ANTENA_LOCATION{
	float x;
	float y;
	float z;
} antena_location_t;



typedef struct _TRIANGULATION_STATE{
	antena_count_t count;
	antena_location_t* data;
	float _base_x;
	float _base_y;
	float _base_z;
	float _base_error;
} triangulation_state_t;



typedef struct _RECEIVER_LOCATION{
	float x;
	float y;
	float z;
	float error;
} receiver_location_t;



void free_triangulation_state(triangulation_state_t* state);



void triangulate_antenas(antena_count_t count,const float* antena_signals,triangulation_state_t* state);



void triangulate_point(const triangulation_state_t* state,const float* antena_signals,receiver_location_t* out);



#endif
