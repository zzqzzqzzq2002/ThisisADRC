#ifndef __ADRC__
#define __ADRC__

//#include "main.h"

#include <math.h>
#include <string.h>
#include <stdint.h>

#define timeradio 1000.f

typedef struct _ADRC_InitNLC_TypeDef
{
	float B;
	float Alpha_p;
	float Alpha_n;
	float Alpha_f;
	float Beta_p;
	float Beta_n;
	float Beta_f;
	float Delta;
	float MaxOutputClamp;
}ADRC_InitNLC;

typedef struct _ADRC_InitESO_TypeDef
{
	float Alpha1;
	float Alpha2;
	float Alpha3;
	float Omega;
	float Delta;
}ADRC_InitESO;

typedef struct _ADRC_InitOther_TypeDef
{
	float DeadBand;
}ADRC_InitOther;

typedef struct _ADRC_Init_TypeDef
{
	struct
	{
		float R;
	}TD;

	ADRC_InitNLC NLC;
	ADRC_InitESO ESO;
	ADRC_InitOther Other;
}ADRC_Init;

typedef struct _ADRC_TypeDef
{
	float Target;
	float Lastnot0_target;

	float Measure;

	struct _TD
	{
		float R1;
		float R2;
		float V1;
		float V2;

		float R;
	}TD;

	struct _NLC
	{
		float E0;
		float E1;
		float E2;
		float U0;
		float U;

		float B;
		float Alpha_p;
		float Alpha_n;
		float Alpha_f;
		float Beta_p;
		float Beta_n;
		float Beta_f;
		float Delta;
		float MaxOutputClamp;
	}NLC;

	struct _ESO
	{
		float Z1;
		float Z2;
		float Z3;

		float Alpha1;
		float Alpha2;
		float Alpha3;
		// float Beta1;
		// float Beta2;
		// float Beta3;
		float Omega;
		float Delta;
	}ESO;

	struct _Time
	{
		uint32_t Time_p;
		uint32_t Time_n;
		float Dtime;
	}Time;

	float Output;
	float Last_output;
	float MaxErr_recoder;

	struct _Other
	{
		float DeadBand;
	}Other;

	void (*TD_cal)(struct _ADRC_TypeDef * adrc);
	void (*NLC_cal)(struct _ADRC_TypeDef * adrc);
	void (*ESO_cal)(struct _ADRC_TypeDef * adrc);
	void (*fullCal)(struct _ADRC_TypeDef * adrc);
	void (*getTimeStamp)(struct _ADRC_TypeDef * adrc);
	void (*param_init)(struct _ADRC_TypeDef * adrc, ADRC_Init adrc_init);
	void (*inputStatus)(struct _ADRC_TypeDef * adrc, float target, float feedback);
	float (*getlastOutput)(struct _ADRC_TypeDef * adrc);
	float (*getOutput)(struct _ADRC_TypeDef * adrc, float target, float feedback);
	void (*reset_TD)(struct _ADRC_TypeDef * adrc, float new_R);
	void (*reset_NLC)(struct _ADRC_TypeDef * adrc, ADRC_InitNLC new_NLC);
	void (*reset_ESO)(struct _ADRC_TypeDef * adrc, ADRC_InitESO new_ESO);
	void (*reset_Other)(struct _ADRC_TypeDef * adrc, float new_DeadBand);
	void (*reset_allParam)(struct _ADRC_TypeDef * adrc, ADRC_Init adrc_init);
	void (*restart)(struct _ADRC_TypeDef * adrc);
	void (*clean_recoder)(struct _ADRC_TypeDef * adrc);
}ADRC;

extern const ADRC_Init Default_ADRC;
extern const ADRC_Init Default_3508_speed;

ADRC zepi_create_ADRC(void);
#define create_ADRC zepi_create_ADRC

///////////////////////////////////////////////////////////////////
#define zepi_sign(x) ((x)>0?1:((x)<0?-1:1))
#define zepi_max(a /*Generic*/,b /*Generic*/) ((a)>(b)?(a):(b))
#define zepi_min(a /*Generic*/,b /*Generic*/) ((a)<(b)?(a):(b))
#define zepi_clamp(n /*Generic*/,a /*Generic*/,b /*Generic*/) (zepi_min(zepi_max(n,a),b))
///////////////////////////////////////////////////////////////////

#endif
