#include <math.h>
#include <signal_strength_triangulation.h>
#include <stdint.h>
#include <stdlib.h>



#define ANTENA_DISTANCE_SQ(idx,x_,y_,z_) (((state->data+(idx))->x-(x_))*((state->data+(idx))->x-(x_))+((state->data+(idx))->y-(y_))*((state->data+(idx))->y-(y_))+((state->data+(idx))->z-(z_))*((state->data+(idx))->z-(z_)))



static const uint32_t _sqrt_values[192]={
	0x02000080,0x01fc0781,0x01f81f82,0x01f44683,0x01f07c84,0x01ecc085,0x01e91386,0x01e57387,
	0x01e1e188,0x01de5d89,0x01dae68a,0x01d77b8b,0x01d41d8c,0x01d0cb8d,0x01cd858e,0x01ca4b8f,
	0x01c71c90,0x01c71c90,0x01c3f891,0x01c0e092,0x01bdd293,0x01bacf94,0x01b7d695,0x01b4e896,
	0x01b20397,0x01b20397,0x01af2898,0x01ac5799,0x01a98e9a,0x01a6d09b,0x01a41a9c,0x01a41a9c,
	0x01a16d9d,0x019ec89e,0x019c2d9f,0x019999a0,0x019999a0,0x01970ea1,0x01948ba2,0x01920fa3,
	0x018f9ca4,0x018f9ca4,0x018d30a5,0x018acba6,0x01886ea7,0x01886ea7,0x018618a8,0x0183c9a9,
	0x018181aa,0x018181aa,0x017f40ab,0x017d05ac,0x017ad2ad,0x017ad2ad,0x0178a4ae,0x01767daf,
	0x01745db0,0x01745db0,0x017242b1,0x01702eb2,0x016e1fb3,0x016e1fb3,0x016c16b4,0x016a13b5,
	0x016a13b5,0x016816b6,0x01661eb7,0x01661eb7,0x01642cb8,0x01623fb9,0x016058ba,0x016058ba,
	0x015e75bb,0x015c98bc,0x015c98bc,0x015ac0bd,0x0158edbe,0x0158edbe,0x01571ebf,0x015555c0,
	0x015555c0,0x015390c1,0x0151d0c2,0x0151d0c2,0x015015c3,0x014e5ec4,0x014e5ec4,0x014cabc5,
	0x014afdc6,0x014afdc6,0x014953c7,0x0147aec8,0x0147aec8,0x01460cc9,0x01460cc9,0x01446fca,
	0x0142d6cb,0x0142d6cb,0x014141cc,0x013fb0cd,0x013fb0cd,0x013e22ce,0x013e22ce,0x013c99cf,
	0x013b13d0,0x013b13d0,0x013991d1,0x013813d2,0x013813d2,0x013698d3,0x013698d3,0x013521d4,
	0x0133aed5,0x0133aed5,0x01323ed6,0x01323ed6,0x0130d1d7,0x012f68d8,0x012f68d8,0x012e02d9,
	0x012e02d9,0x012c9fda,0x012b40db,0x012b40db,0x0129e4dc,0x0129e4dc,0x01288bdd,0x01288bdd,
	0x012735de,0x0125e2df,0x0125e2df,0x012492e0,0x012492e0,0x012345e1,0x012345e1,0x0121fbe2,
	0x0120b4e3,0x0120b4e3,0x011f70e4,0x011f70e4,0x011e2ee5,0x011e2ee5,0x011cf0e6,0x011cf0e6,
	0x011bb4e7,0x011a7be8,0x011a7be8,0x011945e9,0x011945e9,0x011811ea,0x011811ea,0x0116e0eb,
	0x0116e0eb,0x0115b1ec,0x011485ed,0x011485ed,0x01135cee,0x01135cee,0x011235ef,0x011235ef,
	0x011111f0,0x011111f0,0x010feff1,0x010feff1,0x010ecff2,0x010ecff2,0x010db2f3,0x010db2f3,
	0x010c97f4,0x010c97f4,0x010b7ef5,0x010a68f6,0x010a68f6,0x010953f7,0x010953f7,0x010842f8,
	0x010842f8,0x010732f9,0x010732f9,0x010624fa,0x010624fa,0x010519fb,0x010519fb,0x010410fc,
	0x010410fc,0x010309fd,0x010309fd,0x010204fe,0x010204fe,0x010101ff,0x010101ff,0x010101ff
};



#ifdef __arm__
static const uint8_t _shift_values[32]={
	30,22,30,20,18,10,28,2,20,16,14,12,8,6,28,0,
	22,18,10,2,16,14,6,24,12,4,8,24,4,26,26,0
};
#endif



static inline distance_t _int_abs(distance_t x){
	return (x+(x>>31))^(x>>31);
}



static inline uint32_t _int_sqrt(uint32_t x){
	x|=1;
#ifdef __arm__
	uint32_t t=x|(x>>1);
	t|=t>>2;
	t|=t>>4;
	t|=t>>8;
	uint8_t shift=_shift_values[((t|(t>>16))*0x07c4acdd)>>27];
	x=(x<<shift)>>16;
	t=_sqrt_values[(x>>8)-64];
	return (((t&0xff)<<7)+((x*(t>>16)+((x*(t&0xffff))>>16))>>9))>>(shift>>1);
#else
	uint8_t shift=__builtin_clz(x)&0xfe;
	x<<=shift;
	uint32_t t=_sqrt_values[(x>>24)-64];
	return (((t&0xff)<<7)+((uint32_t)((((uint64_t)x)*t)>>41)))>>(shift>>1);
#endif
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
		do{
			distance_t quad_error=error&0xfffffffc;
			error>>=2;
			distance_t tmp=error+(error>>1);
			y+=tmp;
			tmp+=error;
			x-=tmp;
			z+=tmp;
			uint8_t next_pos;
			for (uint8_t j=0;j<64;j++){
				if (!(j&3)){
					y+=error;
					z-=quad_error;
				}
				if (!(j&15)){
					x+=error;
					y-=quad_error;
				}
				distance_t new_dist_sum=0;
				for (antena_count_t k=0;k<i;k++){
					new_dist_sum+=_int_abs(_int_sqrt(ANTENA_DISTANCE_SQ(k,x,y,z))-antena_signals[k]);
				}
				if (!j||new_dist_sum<dist_sum){
					dist_sum=new_dist_sum;
					next_pos=j;
				}
				z+=error;
			}
			x+=((next_pos>>4)-3)*error;
			y+=(((next_pos>>2)&3)-3)*error;
			z+=((next_pos&3)-4)*error;
		} while (error);
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
	do{
		distance_t quad_error=error&0xfffffffc;
		error>>=2;
		distance_t tmp=error+(error>>1);
		y+=tmp;
		tmp+=error;
		x-=tmp;
		z+=tmp;
		uint8_t next_pos;
		for (uint8_t i=0;i<64;i++){
			if (!(i&3)){
				y+=error;
				z-=quad_error;
			}
			if (!(i&15)){
				x+=error;
				y-=quad_error;
			}
			distance_t new_dist_sum=0;
			for (antena_count_t j=0;j<state->count;j++){
				new_dist_sum+=_int_abs(_int_sqrt(ANTENA_DISTANCE_SQ(j,x,y,z))-antena_signals[j]);
			}
			if (!i||new_dist_sum<dist_sum){
				dist_sum=new_dist_sum;
				next_pos=i;
			}
			z+=error;
		}
		x+=((next_pos>>4)-3)*error;
		y+=(((next_pos>>2)&3)-3)*error;
		z+=((next_pos&3)-4)*error;
	} while (error);
	out->x=x;
	out->y=y;
	out->z=z;
	out->error=dist_sum;
}
