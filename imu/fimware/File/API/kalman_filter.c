#include "kalman_filter.h"

float debug_Q[4] = {10,0,0,1000}, debug_R[4] = {1000000,0,0,100};


void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
    mat_init(&F->xhat,2,1,(float *)I->xhat_data);
    mat_init(&F->xhatminus,2,1,(float *)I->xhatminus_data);
    mat_init(&F->z,2,1,(float *)I->z_data);
    mat_init(&F->AT,2,2,(float *)I->AT_data);
    mat_init(&F->HT,2,2,(float *)I->HT_data);
    mat_init(&F->Q,2,2,(float *)I->Q_data);
    mat_init(&F->R,2,2,(float *)I->R_data);
    mat_init(&F->P,2,2,(float *)I->P_data);
    mat_init(&F->Pminus,2,2,(float *)I->Pminus_data);
    mat_init(&F->K,2,2,(float *)I->K_data);
    mat_init(&F->A,2,2,(float *)I->A_data);
    mat_init(&F->H,2,2,(float *)I->H_data);
}

float *kalman_filter_calc(kalman_filter_t *F, float signal1, float signal2)
{
    float TEMP_data1[4],TEMP_data2[4],TEMP_data3[4] = {0, 0, 0, 0};
    float TEMP_data21[2] = {0, 0};
    float TEMP_Edata[4] = {1, 0, 0, 1};
    mat TEMP1,TEMP2,TEMP3,TEMPE,TEMP21;

    /*******debug Q,R********/
    mat_init(&F->Q,2,2,(float *)debug_Q);
    mat_init(&F->R,2,2,(float *)debug_R);
    /************************/

    mat_init(&TEMP1,2,2,(float *)TEMP_data1);
    mat_init(&TEMP2,2,2,(float *)TEMP_data2);
    mat_init(&TEMP3,2,2,(float *)TEMP_data3);
    mat_init(&TEMPE,2,2,(float *)TEMP_Edata);
    mat_init(&TEMP21,2,1,(float *)TEMP_data21);

    F->z.pData[0] = signal1;
    F->z.pData[1] = signal2;

    //1. xhat'(k)= A xhat(k-1)
    mat_mult(&F->A, &F->xhat, &F->xhatminus);

    //2. P'(k) = A P(k-1) AT + Q
    mat_mult(&F->A, &F->P, &TEMP1);
    mat_mult(&TEMP1, &F->AT, &TEMP2);
    mat_add(&TEMP2, &F->Q, &F->Pminus);

    //3. K(k) = P'(k) HT / (H P'(k) HT + R)
    mat_mult(&F->H, &F->Pminus, &TEMP1);
    mat_mult(&TEMP1, &F->HT, &TEMP2);
    mat_add(&TEMP2, &F->R, &TEMP3);

    mat_inv(&TEMP3, &TEMP1);
    mat_mult(&F->Pminus, &F->HT, &TEMP2);
    mat_mult(&TEMP2, &TEMP1, &F->K);

    //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
    mat_mult(&F->H, &F->xhatminus, &TEMP21);
    mat_sub(&F->z, &TEMP21, &F->xhat);
    mat_mult(&F->K, &F->xhat, &TEMP21);
    mat_add(&F->xhatminus, &TEMP21, &F->xhat);

    //5. P(k) = (1-K(k)H)P'(k)
    mat_mult(&F->K, &F->H, &TEMP1);
    mat_sub(&TEMPE, &TEMP1, &TEMP2);
    mat_mult(&TEMP2, &F->Pminus, &F->P);

    F->filtered_value[0] = F->xhat.pData[0];
    F->filtered_value[1] = F->xhat.pData[1];

    return F->filtered_value;
}

