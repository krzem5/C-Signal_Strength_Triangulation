#include <math.h>
#include <signal_strength_triangulation.h>
#include <stdio.h>



int main(int argc,const char** argv){
	float antena_signals[3]={
		4.0f, // 1 <-> 0
		4.742362280551751f,3.23882694814033f // 2 <-> 0, 2 <-> 1
	};
	triangulation_state_t state;
	triangulate_antenas(3,antena_signals,&state);
	for (antena_count_t i=0;i<state.count;i++){
		antena_location_t point_a=*(state.data+i);
		printf("[%u]: (%.6f, %.6f, %.6f)\n",i,point_a.x,point_a.y,point_a.z);
		for (antena_count_t j=0;j<i;j++){
			antena_location_t point_b=*(state.data+j);
			float dist=sqrtf((point_a.x-point_b.x)*(point_a.x-point_b.x)+(point_a.y-point_b.y)*(point_a.y-point_b.y)+(point_a.z-point_b.z)*(point_a.z-point_b.z));
			printf("  %u <-> %u: %.6f | %.6f -> ~%.6f\n",i,j,dist,antena_signals[i*(i-1)/2+j],fabs(dist-antena_signals[i*(i-1)/2+j]));
		}
	}
	receiver_location_t receiver;
	float receiver_antena_signals[3]={
		2.2f,
		1.9f,
		3.0f
	};
	triangulate_point(&state,receiver_antena_signals,&receiver);
	printf("(%.6f, %.6f, %.6f) -> ~%.6f\n",receiver.x,receiver.y,receiver.z,receiver.error);
	free_triangulation_state(&state);
	return 0;
}
