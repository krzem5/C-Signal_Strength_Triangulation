#include <math.h>
#include <signal_strength_triangulation.h>
#include <stdlib.h>



#define ANTENA_DISTANCE_SQ(idx,x_,y_,z_) (((state->data+(idx))->x-(x_))*((state->data+(idx))->x-(x_))+((state->data+(idx))->y-(y_))*((state->data+(idx))->y-(y_))+((state->data+(idx))->z-(z_))*((state->data+(idx))->z-(z_)))
#define ANTENA_DISTANCE(idx,x_,y_,z_) sqrtf(ANTENA_DISTANCE_SQ((idx),(x_),(y_),(z_)))
#define CONVERGENCE_ITER_COUNT 100



void free_triangulation_state(triangulation_state_t* state){
	state->count=0;
	free(state->data);
	state->data=NULL;
	state->_base_x=0;
	state->_base_y=0;
	state->_base_z=0;
	state->_base_error=0;
}



void triangulate_antenas(antena_count_t count,const float* antena_signals,triangulation_state_t* state){
	state->count=count;
	state->data=malloc(count*sizeof(antena_location_t));
	state->data->x=0;
	state->data->y=0;
	state->data->z=0;
	float base_x=0;
	float base_y=0;
	float base_z=0;
	for (antena_count_t i=1;i<count;i++){
		float x=base_x/i;
		float y=base_y/i;
		float z=base_z/i;
		float dist_sum=0;
		float error=0;
		float max_radius=0;
		for (antena_count_t j=0;j<i;j++){
			float dist=ANTENA_DISTANCE(j,base_x,base_y,base_z);
			if (dist>error){
				error=dist;
			}
			if (antena_signals[j]>max_radius){
				max_radius=antena_signals[j];
			}
			dist_sum+=fabs(dist-antena_signals[j]);
		}
		error+=max_radius*2;
		for (uint8_t j=0;j<CONVERGENCE_ITER_COUNT;j++){
			float prev_error=error;
			error/=4;
			float next_x=x;
			float next_y=y;
			float next_z=z;
			x-=error*3;
			y+=error;
			z+=error*2;
			for (uint8_t k=0;k<64;k++){
				if (!(k&3)){
					y+=error;
					z-=prev_error;
				}
				if (!(k&15)){
					x+=error;
					y-=prev_error;
				}
				float new_dist_sum=0;
				for (antena_count_t l=0;l<i;l++){
					new_dist_sum+=fabs(ANTENA_DISTANCE(l,x,y,z)-antena_signals[l]);
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
	float base_error=0;
	while (count){
		count--;
		float dist=ANTENA_DISTANCE_SQ(count,base_x,base_y,base_z);
		if (dist>base_error){
			base_error=dist;
		}
	}
	state->_base_x=base_x;
	state->_base_y=base_y;
	state->_base_z=base_z;
	state->_base_error=sqrtf(base_error);
}



void triangulate_point(const triangulation_state_t* state,const float* antena_signals,receiver_location_t* out){
	float x=state->_base_x;
	float y=state->_base_y;
	float z=state->_base_z;
	float dist_sum=0;
	float max_radius=0;
	for (antena_count_t i=0;i<state->count;i++){
		if (antena_signals[i]>max_radius){
			max_radius=antena_signals[i];
		}
		dist_sum+=fabs(ANTENA_DISTANCE(i,x,y,z)-antena_signals[i]);
	}
	float error=state->_base_error+max_radius*2;
	for (uint8_t i=0;i<CONVERGENCE_ITER_COUNT;i++){
		float prev_error=error;
		error/=4;
		float next_x=x;
		float next_y=y;
		float next_z=z;
		x-=error*3;
		y+=error;
		z+=error*2;
		for (uint8_t j=0;j<64;j++){
			if (!(j&3)){
				y+=error;
				z-=prev_error;
			}
			if (!(j&15)){
				x+=error;
				y-=prev_error;
			}
			float new_dist_sum=0;
			for (antena_count_t k=0;k<state->count;k++){
				new_dist_sum+=fabs(ANTENA_DISTANCE(k,x,y,z)-antena_signals[k]);
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
