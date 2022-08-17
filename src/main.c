#include <math.h>
#include <signal_strength_triangulation.h>
#include <stdint.h>
#include <stdio.h>



#define METER_TO_DISTANCE(x) ((distance_t)((x)*768))
#define DISTANCE_TO_METER(x) ((x)/768.0f)

#define SET_SIGNAL(from,to,strength) (antena_signals[(from)*((from)-1)/2+(to)]=(strength))



int main(int argc,const char** argv){
	distance_t antena_signals[3];
	SET_SIGNAL(1,0,METER_TO_DISTANCE(4.0));
	SET_SIGNAL(2,0,METER_TO_DISTANCE(4.742362));
	SET_SIGNAL(2,1,METER_TO_DISTANCE(3.238827));
	triangulation_state_t state;
	triangulate_antenas(3,antena_signals,&state);
	for (antena_count_t i=0;i<state.count;i++){
		antena_location_t point_a=*(state.data+i);
		printf("[%u]: (%.6f, %.6f, %.6f)\n",i,DISTANCE_TO_METER(point_a.x),DISTANCE_TO_METER(point_a.y),DISTANCE_TO_METER(point_a.z));
		for (antena_count_t j=0;j<i;j++){
			antena_location_t point_b=*(state.data+j);
			float dist=sqrtf(DISTANCE_TO_METER(point_a.x-point_b.x)*DISTANCE_TO_METER(point_a.x-point_b.x)+DISTANCE_TO_METER(point_a.y-point_b.y)*DISTANCE_TO_METER(point_a.y-point_b.y)+DISTANCE_TO_METER(point_a.z-point_b.z)*DISTANCE_TO_METER(point_a.z-point_b.z));
			printf("  %u <-> %u: %.6f | %.6f -> ~%.6f\n",i,j,dist,DISTANCE_TO_METER(antena_signals[i*(i-1)/2+j]),fabs(dist-DISTANCE_TO_METER(antena_signals[i*(i-1)/2+j])));
		}
	}
	printf("{%.6f, %.6f, %.6f, ~%.6f}\n",DISTANCE_TO_METER(state._base_x),DISTANCE_TO_METER(state._base_y),DISTANCE_TO_METER(state._base_z),DISTANCE_TO_METER(state._base_error));
	receiver_location_t receiver;
	distance_t receiver_antena_signals[3]={
		METER_TO_DISTANCE(2.2f),
		METER_TO_DISTANCE(1.9f),
		METER_TO_DISTANCE(3.0f)
	};
	triangulate_point(&state,receiver_antena_signals,&receiver);
	printf("(%.6f, %.6f, %.6f) -> ~%.6f\n",DISTANCE_TO_METER(receiver.x),DISTANCE_TO_METER(receiver.y),DISTANCE_TO_METER(receiver.z),DISTANCE_TO_METER(receiver.error));
	free_triangulation_state(&state);
	return 0;
}
