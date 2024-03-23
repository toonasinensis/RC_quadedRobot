#include "filter_algorithm.h"

void LpFilter(ST_LPF* lpf) 
{                                                                   
	fp32 fir_a = 1/(1 + lpf->off_freq * lpf->samp_tim); 
    lpf->out = fir_a * lpf->preout + (1 - fir_a) * lpf->in;
	lpf->preout = lpf->out;	
}
