#include "lib_adrc.h"

const ADRC_Init Default_ADRC =
{
	.TD.R = 10000000.f,

	.NLC =
	{
		.B = 1.f,
		.Alpha_p = 1.f,
		.Alpha_n = 1.f,
		.Alpha_f = 1.f,
		.Beta_p = 0,
		.Beta_n = 0,
		.Beta_f = 0,
		.MaxOutputClamp = 0,
		.Delta = 0
	},

	.ESO =
	{
		.Alpha1 = 1.f,
		.Alpha2 = 1.f,
		.Alpha3 = 1.f,
		.Omega = 0,
		.Delta = 0
	},

	.Other =
	{
		.DeadBand = 0.f
	}
};

const ADRC_Init Default_3508_speed =
{
	.TD.R = 2000.f,

	.NLC =
	{
		.B = 1.92f,
		.Alpha_p = 0.5f,
		.Alpha_n = 0.98f,
		.Alpha_f = 0.9f,
		.Beta_p = 0.5f,
		.Beta_n = 50.f,
		.Beta_f = 0.f,
		.MaxOutputClamp = 16384.f
	},

	.ESO =
	{
		.Alpha1 = 1.1f,
		.Alpha2 = 0.87f,
		.Alpha3 = 0.25f,
		.Omega = 20.f,
		.Delta = 0
	},

	.Other =
	{
		.DeadBand = 0
	}
};

float zepi_fst(float x1, float x2, float r, float h)
{
	float y, d, a0, a1, a, sy, sa, result;

	y = x1 + h * x2;
	d = r * h*h;
	d = (d==0) ? 0.000001f : d;
	a0 = sqrtf(d*(d + 8.f * fabsf(y)));
	a1 = 0.5f *(a0 - d)* zepi_sign(y) + h * x2;
	sy = 0.5f *(zepi_sign(y+d)- zepi_sign(y-d));
	a = (h * x2 + y - a1)* sy + a1;
	sa = 0.5f *(zepi_sign(a+d)- zepi_sign(a-d));
	result = -r *((a / d - zepi_sign(a))* sa + zepi_sign(a));

	return result;
}

static inline float zepi_fal(float x, float alpha, float delta)
{
	float y;

	delta = fabsf(delta);
	if(fabsf(x)>delta) y = powf(fabsf(x), alpha)* zepi_sign(x);
	else y = x / powf(delta, alpha-1);

	return y;
}

static void _adrc_TD_cal(ADRC * adrc)
{
	adrc->TD.V1 = adrc->TD.R2;
	adrc->TD.V2 = zepi_fst(adrc->TD.R1-adrc->Target, adrc->TD.R2, adrc->TD.R, adrc->Time.Dtime);
	adrc->TD.R1 += adrc->Time.Dtime * adrc->TD.V1;
	adrc->TD.R2 += adrc->Time.Dtime * adrc->TD.V2;
}

static void _adrc_NLC_cal(ADRC * adrc)
{
	adrc->NLC.E1 = adrc->TD.R1 - adrc->ESO.Z1;
	adrc->NLC.E2 = adrc->TD.R2 - adrc->ESO.Z2;
	adrc->NLC.E0 += adrc->NLC.E1 * adrc->Time.Dtime;
	adrc->NLC.U0 = adrc->NLC.Beta_p * zepi_fal(adrc->NLC.E0, adrc->NLC.Alpha_p, adrc->NLC.Delta) +
				   adrc->NLC.Beta_n * zepi_fal(adrc->NLC.E1, adrc->NLC.Alpha_n, adrc->NLC.Delta) +
				   adrc->NLC.Beta_f * zepi_fal(adrc->NLC.E2, adrc->NLC.Alpha_f, adrc->NLC.Delta);
	//adrc->NLC.U0 = -zepi_fst(adrc->NLC.E1, adrc->NLC.E2, adrc->NLC.Delta, adrc->Time.Dtime);
	//adrc->NLC.U0 = adrc->NLC.Beta_n * adrc->NLC.E1 + adrc->NLC.Beta_f * adrc->NLC.E2;
	adrc->NLC.U = (adrc->NLC.U0 - adrc->ESO.Z3)/ adrc->NLC.B;
	adrc->NLC.U = zepi_clamp(adrc->NLC.U, -adrc->NLC.MaxOutputClamp, adrc->NLC.MaxOutputClamp);
	adrc->Output = adrc->NLC.U;
}

static void _adrc_ESO_cal(ADRC * adrc)
{
	float e, b1, b2, b3, obLimit;

	b1 = 3.f * adrc->ESO.Omega;
	b2 = 3.f * adrc->ESO.Omega*adrc->ESO.Omega;
	b3 = adrc->ESO.Omega*adrc->ESO.Omega*adrc->ESO.Omega;
	obLimit = adrc->NLC.MaxOutputClamp * adrc->NLC.B;
	e = adrc->ESO.Z1 - adrc->Measure;
	adrc->ESO.Z1 += adrc->Time.Dtime *(adrc->ESO.Z2 - b1 * zepi_fal(e, adrc->ESO.Alpha1, adrc->ESO.Delta));
	adrc->ESO.Z1 = zepi_clamp(adrc->ESO.Z1, -obLimit * 2.f, obLimit * 2.f);
	adrc->ESO.Z2 += adrc->Time.Dtime *(adrc->ESO.Z3 - b2 * zepi_fal(e, adrc->ESO.Alpha2, adrc->ESO.Delta)+ adrc->NLC.B * adrc->NLC.U);
	adrc->ESO.Z2 = zepi_clamp(adrc->ESO.Z2, -obLimit / adrc->Time.Dtime, obLimit / adrc->Time.Dtime);
	adrc->ESO.Z3 -= adrc->Time.Dtime *(b3 * zepi_fal(e, adrc->ESO.Alpha3, adrc->ESO.Delta));
	adrc->ESO.Z3 = zepi_clamp(adrc->ESO.Z3, -adrc->NLC.MaxOutputClamp * adrc->ESO.Omega, adrc->NLC.MaxOutputClamp * adrc->ESO.Omega);
}

static inline void _adrc_fullCal(ADRC * adrc)
{
	adrc->TD_cal(adrc);
	adrc->ESO_cal(adrc);
	adrc->NLC_cal(adrc);
}

static void _adrc_getTimeStamp(ADRC * adrc)
{
	//zepi_UNUSED(adrc);
	adrc->Time.Time_p = adrc->Time.Time_n;
	adrc->Time.Time_n = HAL_GetTick();
	adrc->Time.Dtime = (adrc->Time.Time_n - adrc->Time.Time_p)/ timeradio;
}

static void _adrc_param_init(ADRC * adrc, ADRC_Init adrc_init)
{
	memset(adrc, 0, sizeof(&adrc));
	adrc->TD.R = adrc_init.TD.R;
	adrc->NLC.B = adrc_init.NLC.B;
	adrc->NLC.Alpha_p = adrc_init.NLC.Alpha_p;
	adrc->NLC.Alpha_n = adrc_init.NLC.Alpha_n;
	adrc->NLC.Alpha_f = adrc_init.NLC.Alpha_f;
	adrc->NLC.Beta_p = adrc_init.NLC.Beta_p;
	adrc->NLC.Beta_n = adrc_init.NLC.Beta_n;
	adrc->NLC.Beta_f = adrc_init.NLC.Beta_f;
	adrc->NLC.Delta = adrc_init.NLC.Delta;
	adrc->NLC.MaxOutputClamp = adrc_init.NLC.MaxOutputClamp;
	adrc->ESO.Alpha1 = adrc_init.ESO.Alpha1;
	adrc->ESO.Alpha2 = adrc_init.ESO.Alpha2;
	adrc->ESO.Alpha3 = adrc_init.ESO.Alpha3;
	adrc->ESO.Omega = adrc_init.ESO.Omega;
	adrc->ESO.Delta = adrc_init.ESO.Delta;
	adrc->Other.DeadBand = adrc_init.Other.DeadBand>=0 ? adrc_init.Other.DeadBand : -adrc_init.Other.DeadBand;
}

static void _adrc_inputStatus(ADRC * adrc, float target, float feedback)
{
	float err = 0.f;

	adrc->Lastnot0_target = adrc->Target==0 ? adrc->Lastnot0_target : adrc->Target;
	adrc->Target = target;
	err = adrc->Target - feedback;
	err = err>=0 ? err : -err;
	adrc->Measure = err>adrc->Other.DeadBand ? feedback : adrc->Target;
	adrc->getTimeStamp(adrc);
}

static float _adrc_getLastOutput(ADRC * adrc)
{
	adrc->Last_output = adrc->Output;
	float feedback_error = adrc->Target - adrc->Measure;
	adrc->MaxErr_recoder = feedback_error>adrc->MaxErr_recoder ? feedback_error : adrc->MaxErr_recoder;
	return adrc->Output;
}

static float _adrc_getOutput(ADRC * adrc, float target, float feedback)
{
	adrc->inputStatus(adrc, target, feedback);
	adrc->fullCal(adrc);
	return adrc->getlastOutput(adrc);
}

static void _adrc_reset_TD(ADRC * adrc, float R)
{
	adrc->TD.R = R;
}

static void _adrc_reset_NLC(ADRC * adrc, ADRC_InitNLC new_NLC)
{
	adrc->NLC.B = new_NLC.B;
	adrc->NLC.Alpha_p = new_NLC.Alpha_p;
	adrc->NLC.Alpha_n = new_NLC.Alpha_n;
	adrc->NLC.Alpha_f = new_NLC.Alpha_f;
	adrc->NLC.Beta_p = new_NLC.Beta_p;
	adrc->NLC.Beta_n = new_NLC.Beta_n;
	adrc->NLC.Beta_f = new_NLC.Beta_f;
	adrc->NLC.Delta = new_NLC.Delta;
	adrc->NLC.MaxOutputClamp = new_NLC.MaxOutputClamp;
}

static void _adrc_reset_ESO(ADRC * adrc, ADRC_InitESO new_ESO)
{
	adrc->ESO.Alpha1 = new_ESO.Alpha1;
	adrc->ESO.Alpha2 = new_ESO.Alpha2;
	adrc->ESO.Alpha3 = new_ESO.Alpha3;
	adrc->ESO.Omega = new_ESO.Omega;
	adrc->ESO.Delta = new_ESO.Delta;
}

static void _adrc_reset_Other(ADRC * adrc, float new_DeadBand)
{
	adrc->Other.DeadBand = new_DeadBand>=0 ? new_DeadBand : -new_DeadBand;
}

static void _adrc_reset_allParam(ADRC * adrc, ADRC_Init adrc_init)
{
	memset(adrc, 0, sizeof(&adrc));
	adrc->TD.R = adrc_init.TD.R;
	adrc->NLC.B = adrc_init.NLC.B;
	adrc->NLC.Alpha_p = adrc_init.NLC.Alpha_p;
	adrc->NLC.Alpha_n = adrc_init.NLC.Alpha_n;
	adrc->NLC.Alpha_f = adrc_init.NLC.Alpha_f;
	adrc->NLC.Beta_p = adrc_init.NLC.Beta_p;
	adrc->NLC.Beta_n = adrc_init.NLC.Beta_n;
	adrc->NLC.Beta_f = adrc_init.NLC.Beta_f;
	adrc->NLC.Delta = adrc_init.NLC.Delta;
	adrc->ESO.Alpha1 = adrc_init.ESO.Alpha1;
	adrc->ESO.Alpha2 = adrc_init.ESO.Alpha2;
	adrc->ESO.Alpha3 = adrc_init.ESO.Alpha3;
	adrc->ESO.Omega = adrc_init.ESO.Omega;
	adrc->ESO.Delta = adrc_init.ESO.Delta;
	adrc->NLC.MaxOutputClamp = adrc_init.NLC.MaxOutputClamp;
	adrc->Other.DeadBand = adrc_init.Other.DeadBand;
}

static void _adrc_restart(ADRC * adrc)
{
	adrc->Target = 0;
	adrc->Lastnot0_target = 0;
	adrc->Measure = 0;
	adrc->TD.R1 = 0;
	adrc->TD.R2 = 0;
	adrc->TD.V1 = 0;
	adrc->TD.V2 = 0;
	adrc->NLC.E0 = 0;
	adrc->NLC.E1 = 0;
	adrc->NLC.E2 = 0;
	adrc->NLC.U0 = 0;
	adrc->NLC.U = 0;
	adrc->ESO.Z1 = 0;
	adrc->ESO.Z2 = 0;
	adrc->ESO.Z3 = 0;
	adrc->Output = 0;
	adrc->Last_output = 0;
	adrc->MaxErr_recoder = 0;
}

static void _adrc_clean_recoder(ADRC * adrc)
{
	adrc->MaxErr_recoder = 0;
}

ADRC zepi_create_ADRC(void)
{
	ADRC adrc = {0};

	adrc.TD_cal = _adrc_TD_cal;
	adrc.NLC_cal = _adrc_NLC_cal;
	adrc.ESO_cal = _adrc_ESO_cal;
	adrc.fullCal = _adrc_fullCal;
	adrc.getTimeStamp = _adrc_getTimeStamp;
	adrc.param_init = _adrc_param_init;
	adrc.inputStatus = _adrc_inputStatus;
	adrc.getlastOutput = _adrc_getLastOutput;
	adrc.getOutput = _adrc_getOutput;
	adrc.reset_TD = _adrc_reset_TD;
	adrc.reset_NLC = _adrc_reset_NLC;
	adrc.reset_ESO = _adrc_reset_ESO;
	adrc.reset_Other = _adrc_reset_Other;
	adrc.reset_allParam = _adrc_reset_allParam;
	adrc.restart = _adrc_restart;
	adrc.clean_recoder = _adrc_clean_recoder;

	return adrc;
}
