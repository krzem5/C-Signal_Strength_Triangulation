#include <math.h>
#include <signal_strength_triangulation.h>
#include <stdlib.h>
#include <stdio.h>



#define ANTENA_DISTANCE_SQ(idx,x_,y_,z_) (((state->data+(idx))->x-(x_))*((state->data+(idx))->x-(x_))+((state->data+(idx))->y-(y_))*((state->data+(idx))->y-(y_))+((state->data+(idx))->z-(z_))*((state->data+(idx))->z-(z_)))
#define CONVERGENCE_ITER_COUNT 100


static inline distance_t _int_abs(distance_t x){
	distance_t mask=x>>31;
	return (x+mask)^mask;
}



static inline uint32_t _int_sqrt(uint32_t x){
	return (uint32_t)sqrtf((float)x);
}



void free_triangulation_state(triangulation_state_t* state){
	state->count=0;
	free(state->data);
	state->data=NULL;
	state->_base_x=0;
	state->_base_y=0;
	state->_base_z=0;
	state->_base_error=0;
}



void triangulate_antenas(antena_count_t count,const distance_t* antena_signals,triangulation_state_t* state){
	state->count=count;
	state->data=malloc(count*sizeof(antena_location_t));
	state->data->x=0;
	state->data->y=0;
	state->data->z=0;
	distance_t base_x=0;
	distance_t base_y=0;
	distance_t base_z=0;
	for (antena_count_t i=1;i<count;i++){
		distance_t x=base_x/i;
		distance_t y=base_y/i;
		distance_t z=base_z/i;
		distance_t dist_sum=0;
		distance_t error=0;
		distance_t max_radius=0;
		for (antena_count_t j=0;j<i;j++){
			distance_t dist=_int_sqrt(ANTENA_DISTANCE_SQ(j,base_x,base_y,base_z));
			if (dist>error){
				error=dist;
			}
			if (antena_signals[j]>max_radius){
				max_radius=antena_signals[j];
			}
			dist_sum+=_int_abs(dist-antena_signals[j]);
		}
		error+=max_radius*2;
		for (uint8_t j=0;j<CONVERGENCE_ITER_COUNT;j++){
			distance_t quad_error=error&0xfffffffc;
			error>>=2;
			distance_t half_error=error>>1;
			x-=quad_error-error-half_error;
			y+=error+half_error;
			z+=(error<<1)+half_error;
			uint8_t next_pos;
			for (uint8_t k=0;k<64;k++){
				if (!(k&3)){
					y+=error;
					z-=quad_error;
				}
				if (!(k&15)){
					x+=error;
					y-=quad_error;
				}
				distance_t new_dist_sum=0;
				for (antena_count_t l=0;l<i;l++){
					new_dist_sum+=_int_abs(_int_sqrt(ANTENA_DISTANCE_SQ(l,x,y,z))-antena_signals[l]);
				}
				if (!k||new_dist_sum<dist_sum){
					dist_sum=new_dist_sum;
					next_pos=k;
				}
				z+=error;
			}
			z+=(next_pos&3)*error-quad_error;
			quad_error-=error;
			next_pos>>=2;
			x+=(next_pos>>2)*error-quad_error;
			y+=(next_pos&3)*error-quad_error;
		}
		base_x+=x;
		base_y+=y;
		base_z+=z;
		(state->data+i)->x=x;
		(state->data+i)->y=y;
		(state->data+i)->z=z;
		antena_signals+=i;
	}
	base_x/=count;
	base_y/=count;
	base_z/=count;
	distance_t base_error=0;
	while (count){
		count--;
		distance_t dist=ANTENA_DISTANCE_SQ(count,base_x,base_y,base_z);
		if (dist>base_error){
			base_error=dist;
		}
	}
	state->_base_x=base_x;
	state->_base_y=base_y;
	state->_base_z=base_z;
	state->_base_error=_int_sqrt(base_error);
}



void triangulate_point(const triangulation_state_t* state,const distance_t* antena_signals,receiver_location_t* out){
	distance_t x=state->_base_x;
	distance_t y=state->_base_y;
	distance_t z=state->_base_z;
	distance_t dist_sum=0;
	distance_t max_radius=0;
	for (antena_count_t i=0;i<state->count;i++){
		if (antena_signals[i]>max_radius){
			max_radius=antena_signals[i];
		}
		dist_sum+=_int_abs(_int_sqrt(ANTENA_DISTANCE_SQ(i,x,y,z))-antena_signals[i]);
	}
	distance_t error=state->_base_error+max_radius*2;
	for (uint8_t i=0;i<CONVERGENCE_ITER_COUNT;i++){
		distance_t prev_error=error;
		error>>=2;
		distance_t next_x=x;
		distance_t next_y=y;
		distance_t next_z=z;
		x-=prev_error-error;
		y+=error;
		z+=error<<1;
		for (uint8_t j=0;j<64;j++){
			if (!(j&3)){
				y+=error;
				z-=prev_error;
			}
			if (!(j&15)){
				x+=error;
				y-=prev_error;
			}
			distance_t new_dist_sum=0;
			for (antena_count_t k=0;k<state->count;k++){
				new_dist_sum+=_int_abs(_int_sqrt(ANTENA_DISTANCE_SQ(k,x,y,z))-antena_signals[k]);
			}
			if (new_dist_sum<dist_sum){
				dist_sum=new_dist_sum;
				next_x=x;
				next_y=y;
				next_z=z;
			}
			z+=error;
		}
		x=next_x;
		y=next_y;
		z=next_z;
	}
	out->x=x;
	out->y=y;
	out->z=z;
	out->error=dist_sum;
}
