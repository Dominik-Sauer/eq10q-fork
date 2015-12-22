/***************************************************************************
 *   Copyright (C) 2011 by Pere Ràfols Soler                               *
 *   sapista2@gmail.com                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/***************************************************************************
This file contains the filter definitions
****************************************************************************/

#ifndef  FILTER_H
  #define FILTER_H

#include <math.h>

//#include <stdio.h>

//Constants definitions
#ifndef PI
  #define PI 3.1416
#endif

#include "../filter_type.h"

//Interpolation params
#define FREQ_INTER_DEC_SECOND 30.0f
#define GAIN_INTER_DB_SECOND 15.0f
#define Q_INTER_DEC_SECOND 10.0f

typedef struct Filter {
    double b0, b1, b2, a1, a2; //Second Order coefficients
    double b1_0, b1_1, b1_2, a1_1, a1_2; //Second Order extra coefficients
    int filter_order;  //filter order
    double fs; //sample rate
    float gain, freq, q;
    int is_enabled;
    FilterType iType; //Filter type

    //Interpolation Params
    float freqInter;
    float gainInter;
    float QInter;
} Filter;

typedef struct
{ 
  double buf_0;
  double buf_1;
  double buf_2;
  double buf_e0;
  double buf_e1;
  double buf_e2; 
}Buffers;

//Initialize filter instance
static inline Filter *FilterInit(double rate) {
    Filter *filter = (Filter *)malloc(sizeof(Filter));

    filter->fs= rate;
    filter->gain = 1.0f;
    filter->freq = 100.0f;
    filter->q = 1.0f;
    filter->is_enabled = 0;
    filter->iType = FILTER_TYPE_NOT_SET;

    //Interpolations
    filter->freqInter = pow(10.0f, FREQ_INTER_DEC_SECOND/(float)rate);
    filter->gainInter = pow(10.0f,0.05f * GAIN_INTER_DB_SECOND)/(float)rate;
    filter->QInter = pow(10.0f, Q_INTER_DEC_SECOND/(float)rate);

    return filter;
}


//Destroy a filter instance
static inline void FilterClean(Filter *filter)
{
    free(filter);
}

//Clean buffers
void flushBuffers(Buffers *buf);

typedef struct {
    double a0;
    double a1;
    double a2;

    double b0;
    double b1;
    double b2;

    double a1_0;
    double a1_1;
    double a1_2;

    double b1_0;
    double b1_1;
    double b1_2;
} Coefficients;

static inline void initialize_coefficients( Coefficients* p_coefficients ) {
    p_coefficients->a0 = 1.0;
    p_coefficients->a1 = 1.0;
    p_coefficients->a2 = 1.0;

    p_coefficients->b0 = 1.0;
    p_coefficients->b1 = 1.0;
    p_coefficients->b2 = 1.0;

    p_coefficients->a1_0 = 1.0;
    p_coefficients->a1_1 = 1.0;
    p_coefficients->a1_2 = 1.0;

    p_coefficients->b1_0 = 1.0;
    p_coefficients->b1_1 = 1.0;
    p_coefficients->b1_2 = 1.0;

    return;
}

static inline void interpolate_freq( Filter* filter, float fFreq ) {
    float err = fFreq / filter->freq;

    filter->freq
        = ( err > filter->freqInter )   ? filter->freq * filter->freqInter
        : ( err < 1/filter->freqInter ) ? filter->freq / filter->freqInter
                                        : fFreq;

    return;
}

static inline void interpolate_gain( Filter* filter, float fGain ) {
    filter->gain = fGain;
    return;

    float err = fGain - filter->gain;

    filter->gain
        = ( err > filter->gainInter )   ? filter->gain + filter->gainInter
        : ( err < - filter->gainInter ) ? filter->gain - filter->gainInter
                                        : fGain;

    return;
}

static inline void interpolate_q( Filter* filter, float fQ ) {
    float err = fQ / filter->q;

    filter->q
        = ( err > filter->QInter )      ? filter->q * filter->QInter
        : ( err < 1/filter->QInter )    ? filter->q / filter->QInter
                                        : fQ;

    return;
}

static inline void compute_HPF1_coefficients( Filter* filter, Coefficients* p_coefficients ) {
    filter->filter_order = 0;

    double w0 = tanf( PI * ( filter->freq / filter->fs ) );

    p_coefficients->b0 = 1.0;
    p_coefficients->b1 = -1.0;
    p_coefficients->b2 = 0.0;
    p_coefficients->a0 = w0+1.0;
    p_coefficients->a1 = w0-1.0;
    p_coefficients->a2 = 0.0;

    return;
}

static inline void compute_LPF1_coefficients( Filter* filter, Coefficients* p_coefficients ) {
    filter->filter_order = 0;

    double w0 = tanf( PI * ( filter->freq / filter->fs ) );

    p_coefficients->b0 = w0;
    p_coefficients->b1 = w0;
    p_coefficients->b2 = 0.0;
    p_coefficients->a0 = w0+1.0;
    p_coefficients->a1 = w0-1.0;
    p_coefficients->a2 = 0.0;

    return;
}

static inline void compute_HPF2_coefficients( Filter* filter, Coefficients* p_coefficients ) {
    filter->filter_order = 0;

    double w0 = 2 * PI * ( filter->freq / filter->fs );
    double alpha = sinf( w0 ) / ( 2 * filter->q );
    double cosf_w0 = cosf( w0 );

    p_coefficients->b1_0 = p_coefficients->b0 = (1 + cosf_w0)/2; //b0
    p_coefficients->b1_1 = p_coefficients->b1 = -(1 + cosf_w0); //b1
    p_coefficients->b1_2 = p_coefficients->b2 = (1 + cosf_w0)/2; //b2
    p_coefficients->a1_0 = p_coefficients->a0 = 1 + alpha; //a0
    p_coefficients->a1_1 = p_coefficients->a1 = -2 * cosf_w0; //a1
    p_coefficients->a1_2 = p_coefficients->a2 = 1 - alpha; //a2

    return;
}

static inline void compute_LPF2_coefficients( Filter* filter, Coefficients* p_coefficients ) {
    filter->filter_order = 0;

    double w0 = 2 * PI * ( filter->freq / filter->fs );
    double alpha = sinf( w0 ) / ( 2 * filter->q );
    double cosf_w0 = cosf( w0 );

    p_coefficients->b1_0 = p_coefficients->b0 = (1 - cosf_w0)/2; //b0
    p_coefficients->b1_1 = p_coefficients->b1 = 1 - cosf_w0; //b1
    p_coefficients->b1_2 = p_coefficients->b2 = (1 - cosf_w0)/2; //b2
    p_coefficients->a1_0 = p_coefficients->a0 = 1 + alpha; //a0
    p_coefficients->a1_1 = p_coefficients->a1 = -2 * cosf_w0; //a1
    p_coefficients->a1_2 = p_coefficients->a2 = 1 - alpha; //a2

    return;
}

static inline void compute_HPF3_coefficients( Filter* filter, Coefficients* p_coefficients ) {
    filter->filter_order = 1;

    double w0 = 2 * PI * ( filter->freq / filter->fs );
    double alpha = sinf( w0 ) / ( 2 * filter->q );
    double cosf_w0 = cosf( w0 );

    p_coefficients->b0 = (1 + cosf_w0)/2; //b0
    p_coefficients->b1 = - (1 + cosf_w0); //b1
    p_coefficients->b2 = (1 + cosf_w0)/2; //b2
    p_coefficients->a0 = 1 + alpha; //a0
    p_coefficients->a1 = -2 * cosf_w0; //a1
    p_coefficients->a2 = 1 - alpha; //a2

    w0 = tanf( w0 / 2.0 );

    p_coefficients->b1_0 = 1.0;
    p_coefficients->b1_1 = -1.0;
    p_coefficients->b1_2 = 0.0;
    p_coefficients->a1_0 = w0 + 1.0;
    p_coefficients->a1_1 = w0 - 1.0;
    p_coefficients->a1_2 = 0.0;

    return;
}

static inline void compute_LPF3_coefficients( Filter* filter, Coefficients* p_coefficients ) {
    filter->filter_order = 1;

    double w0 = 2 * PI * ( filter->freq / filter->fs );
    double alpha = sinf( w0 ) / ( 2 * filter->q );
    double cosf_w0 = cosf( w0 );

    p_coefficients->b0 = (1 - cosf_w0)/2; //b0
    p_coefficients->b1 = 1 - cosf_w0; //b1
    p_coefficients->b2 = (1 - cosf_w0)/2; //b2
    p_coefficients->a0 = 1 + alpha; //a0
    p_coefficients->a1 = -2 * cosf_w0; //a1
    p_coefficients->a2 = 1 - alpha; //a2

    w0 = tanf( w0 / 2.0 );

    p_coefficients->b1_0 = w0;
    p_coefficients->b1_1 = w0;
    p_coefficients->b1_2 = 0.0;
    p_coefficients->a1_0 = w0 + 1.0;
    p_coefficients->a1_1 = w0 - 1.0;
    p_coefficients->a1_2 = 0.0;

    return;
}

static inline void compute_HPF4_coefficients( Filter* filter, Coefficients* p_coefficients ) {
    filter->filter_order = 1;

    return compute_HPF2_coefficients( filter, p_coefficients );
}

static inline void compute_LPF4_coefficients( Filter* filter, Coefficients* p_coefficients ) {
    filter->filter_order = 1;

    return compute_LPF2_coefficients( filter, p_coefficients );
}

static inline void compute_LSH_coefficients( Filter* filter, Coefficients* p_coefficients ) {
    filter->filter_order = 0;

    double w0 = 2 * PI * ( filter->freq / filter->fs );
    double A = sqrtf( filter->gain );
    double sqrtf_A = sqrtf( A );
    double alpha = sinf( w0 ) / ( 2 * filter->q );
    double sqrtf_A_alpha_2 = 2*sqrtf_A*alpha;
    double cosf_w0 = cosf( w0 );

    p_coefficients->b0 = A * ( (A+1) - (A-1)*cosf_w0 + sqrtf_A_alpha_2 ); //b0
    p_coefficients->b1 = 2*A * ( (A-1) - (A+1)*cosf_w0 ); //b1
    p_coefficients->b2 = A * ( (A+1) - (A-1)*cosf_w0 - sqrtf_A_alpha_2 ); //b2
    p_coefficients->a0 = (A+1) + (A-1)*cosf_w0 + sqrtf_A_alpha_2; //a0
    p_coefficients->a1 = -2 * ( (A-1) + (A+1)*cosf_w0 ); //a1
    p_coefficients->a2 = (A+1) + (A-1)*cosf_w0 - sqrtf_A_alpha_2; //a2

    return;
}

static inline void compute_HSH_coefficients( Filter* filter, Coefficients* p_coefficients ) {
    filter->filter_order = 0;

    double w0 = 2 * PI * ( filter->freq / filter->fs );
    double A = sqrtf( filter->gain );
    double sqrtf_A = sqrtf( A );
    double alpha = sinf( w0 ) / ( 2 * filter->q );
    double sqrtf_A_alpha_2 = 2*sqrtf_A*alpha;
    double cosf_w0 = cosf( w0 );

    p_coefficients->b0 = A * ( (A+1) + (A-1)*cosf_w0 + sqrtf_A_alpha_2 ); //b0
    p_coefficients->b1 = -2*A * ( (A-1) + (A+1)*cosf_w0 ); //b1
    p_coefficients->b2 = A * ( (A+1) + (A-1)*cosf_w0 - sqrtf_A_alpha_2 ); //b2
    p_coefficients->a0 = (A+1) - (A-1)*cosf_w0 + sqrtf_A_alpha_2; //a0
    p_coefficients->a1 = 2 * ( (A-1) - (A+1)*cosf_w0 ); //a1
    p_coefficients->a2 = (A+1) - (A-1)*cosf_w0 - sqrtf_A_alpha_2; //a2

    return;
}

static inline void compute_NOTCH_coefficients( Filter* filter, Coefficients* p_coefficients ) {
    filter->filter_order = 0;

    double w0 = 2 * PI * ( filter->freq / filter->fs );
    double alpha = sinf( w0 ) / ( 2*filter->q );
    double cosf_w0 = cosf( w0 );

    p_coefficients->b0 =  1.0; //b0
    p_coefficients->b1 = -2 * cosf_w0; //b1
    p_coefficients->b2 =  1.0; //b2
    p_coefficients->a0 =  1.0 + alpha; //a0
    p_coefficients->a1 = -2 * cosf_w0; //a1
    p_coefficients->a2 =  1.0 - alpha; //a2

    return;
}

static inline void compute_PEAK_NOGAIN_coefficients( Filter* filter, Coefficients* p_coefficients ) {
    filter->filter_order = 0;

    p_coefficients->b0 = 1.0;
    p_coefficients->b1 = 0.0;
    p_coefficients->b2 = 0.0;
    p_coefficients->a0 = 1.0;
    p_coefficients->a1 = 0.0;
    p_coefficients->a2 = 0.0;

    return;
}

static inline void compute_PEAK_GAIN_coefficients( Filter* filter, Coefficients* p_coefficients ) {
    filter->filter_order = 0;

    double w0 = 2 * PI * ( filter->freq / filter->fs );

    double AA = sqrtf( filter->gain );
    double A2 = filter->gain;
    double PI2 = PI * PI;
    double Q2 = filter->q * filter->q;
    double w02 = w0 * w0;
    double w02_PI22 = (w02 - PI2)*(w02 - PI2);

    //Equivalent analog filter and analog gains
    double G1 = sqrtf(
        ( w02_PI22 + ( A2 * w02 * PI2 ) / Q2 )
        /
        ( w02_PI22 + ( w02 * PI2 ) / ( Q2*A2 ) )
    );
    double GB = sqrt(G1*filter->gain);
    double GB2 = GB * GB;
    double G2 = filter->gain * filter->gain;
    double G12 = G1 * G1;

    //Digital filter
    double F   = fabsf(G2  - GB2);// + 0.00000001f; ///TODO akest petit num sumat en teoria no hi va pero he detectat div by 0
    double G00 = fabsf(G2  - 1.0);// + 0.00000001f;  ///TODO akest petit num sumat en teoria no hi va pero he detectat div by 0
    double F00 = fabsf(GB2 - 1.0);// + 0.00000001f;  ///TODO akest petit num sumat en teoria no hi va pero he detectat div by 0
    double G01 = fabsf(G2  - G1);// + 0.00000001f;  ///TODO akest petit num sumat en teoria no hi va pero he detectat div by 0
    double G11 = fabsf(G2  - G12);// + 0.00000001f;  ///TODO akest petit num sumat en teoria no hi va pero he detectat div by 0
    double F01 = fabsf(GB2 - G1);// + 0.00000001f;  ///TODO akest petit num sumat en teoria no hi va pero he detectat div by 0
    double F11 = fabsf(GB2 - G12);// + 0.00000001f;  ///TODO akest petit num sumat en teoria no hi va pero he detectat div by 0
    double W2 = sqrtf(G11 / G00) * tanf(w0/2.0) * tanf(w0/2.0);

    //Bandwidth condition
    double Aw = (w0/(AA*filter->q))*sqrtf((GB2-A2 * A2)/(1.0 - GB2)); //Analog filter bandwidth at GB
    double DW = (1.0 + sqrtf(F00 / F11) * W2) * tanf(Aw/2.0); //Prewarped digital bandwidth

//     printf("G1=%f Aw=%f DW=%f F11=%f GB2=%f G12=%f\r\n",G1,Aw,DW,F11,GB2,G12);

    //Digital coefs
    double C = F11 * DW * DW - 2.0 * W2 * (F01 - sqrtf(F00 * F11));
    double D = 2.0 * W2 * (G01 - sqrtf(G00 * G11));
    double A = sqrtf((C + D) / F);
    double B = sqrtf((G2 * C + GB2 * D) / F);

//     printf("A=%f B=%f C=%f D=%f\r\n", A, B, C, D);

    p_coefficients->b0 = G1 + W2 + B;
    p_coefficients->b1 =  -2.0*(G1 - W2);
    p_coefficients->b2 = G1 - B + W2;
    p_coefficients->a0 = 1.0 + W2 + A;
    p_coefficients->a1 = -2.0*(1.0 - W2);
    p_coefficients->a2 = 1.0 + W2 - A;

    return;
}

static inline int is_filter_gain_significant( Filter* filter ) {
    return ( filter->gain > 1.01 || filter->gain < 0.98 );
}

static inline void compute_PEAK_coefficients( Filter* filter, Coefficients* p_coefficients ) {
    return is_filter_gain_significant( filter ) ? compute_PEAK_GAIN_coefficients( filter, p_coefficients )
                                                : compute_PEAK_NOGAIN_coefficients( filter, p_coefficients );
}

static inline void compute_BPF_coefficients( Filter* filter, Coefficients* p_coefficients ) {
    double w0 = 2 * PI * ( filter->freq / filter->fs );
    double alpha = sinf( w0 ) / ( 2*filter->q );
    double cosf_w0 = cosf( w0 );

    p_coefficients->b0 = alpha;
    p_coefficients->b1 = 0.0;
    p_coefficients->b2 = -alpha;
    p_coefficients->a0 = 1.0 + alpha;
    p_coefficients->a1 = -2.0 * cosf_w0;
    p_coefficients->a2 = 1.0 -alpha;

//     printf("****BPF****\n");

    return;
}

static inline void normalize_coefs( Filter* filter, Coefficients* p_coefficients ){
    filter->b0 = (p_coefficients->b0/p_coefficients->a0); //b0
    filter->b1 = (p_coefficients->b1/p_coefficients->a0); //b1
    filter->b2 = (p_coefficients->b2/p_coefficients->a0); //b2
    filter->a1 = (p_coefficients->a1/p_coefficients->a0); //a1
    filter->a2 = (p_coefficients->a2/p_coefficients->a0); //a2
    filter->b1_0 = (p_coefficients->b1_0/p_coefficients->a1_0);
    filter->b1_1 = (p_coefficients->b1_1/p_coefficients->a1_0);
    filter->b1_2 = (p_coefficients->b1_2/p_coefficients->a1_0);
    filter->a1_1 = (p_coefficients->a1_1/p_coefficients->a1_0);
    filter->a1_2 = (p_coefficients->a1_2/p_coefficients->a1_0);
}

static inline void disable_filter( Filter* filter, FilterType iType ) {
    filter->is_enabled = 0;
    filter->iType = iType;

    filter->b0 = filter->b1_0 = 1.0;
    filter->b1 = filter->b2 = filter->a1 = filter->a2
    = filter->b1_1 = filter->b1_2 = filter->a1_1 = filter->a1_2 = 0.0;

    return;
}

static inline void compute_coefficients( Filter* filter, Coefficients* p_coefficients, FilterType iType ){
    switch( iType ) {
        case HPF_ORDER_1:
            compute_HPF1_coefficients( filter, p_coefficients );
            break;
        case HPF_ORDER_2:
            compute_HPF2_coefficients( filter, p_coefficients );
            break;
        case HPF_ORDER_3:
            compute_HPF3_coefficients( filter, p_coefficients );
            break;
        case HPF_ORDER_4:
            compute_HPF4_coefficients( filter, p_coefficients );
            break;

        case LPF_ORDER_1:
            compute_LPF1_coefficients( filter, p_coefficients );
            break;
        case LPF_ORDER_2:
            compute_LPF2_coefficients( filter, p_coefficients );
            break;
        case LPF_ORDER_3:
            compute_LPF3_coefficients( filter, p_coefficients );
            break;
        case LPF_ORDER_4:
            compute_LPF4_coefficients( filter, p_coefficients );
            break;

        case LOW_SHELF:
            compute_LSH_coefficients( filter, p_coefficients );
            break;
        case HIGH_SHELF:
            compute_HSH_coefficients( filter, p_coefficients );
            break;

        case PEAK:
            compute_PEAK_coefficients( filter, p_coefficients );
            break;
        case NOTCH:
            compute_NOTCH_coefficients( filter, p_coefficients );
            break;

        case BAND_PASS:
            compute_BPF_coefficients( filter, p_coefficients );
            break;

        default:
            break;
    }

    return;
}

//Compute filter coefficients
static inline void calcCoefs(Filter *filter, float fGain, float fFreq, float fQ, FilterType iType, int is_enabled) //p2 = GAIN p3 = Q
{   
    if( ! is_enabled ) {
        disable_filter( filter, iType );
        return;
    }

    Coefficients coefficients;
    Coefficients* p_coefficients = &coefficients;

    initialize_coefficients( p_coefficients );

    interpolate_freq( filter, fFreq );
    interpolate_gain( filter, fGain );
    interpolate_q( filter, fQ );

    filter->is_enabled = is_enabled;
    filter->iType = iType;

    compute_coefficients( filter, p_coefficients, iType );

    normalize_coefs( filter, p_coefficients );

    //Print coefs
//     printf("Coefs b0=%f b1=%f b2=%f a1=%f a2=%f\r\n",filter->b0,filter->b1,filter->b2,filter->a1,filter->a2);
//     printf("Gain = %f Freq = %f Q = %f\r\n", filter->gain, filter->freq, filter->q);
}

#define DENORMAL_TO_ZERO(x) if (fabs(x) < (1e-300)) x = 0.0; //Min float is 1.1754943e-38 (Min double is 2.23×10−308)

//Compute filter
static inline  void computeFilter(Filter *filter, Buffers *buf, double *inputSample)
{
  //Process 1, 2 orders
  //w(n)=x(n)-a1*w(n-1)-a2*w(n-2)
  buf->buf_0 = (*inputSample)-filter->a1*buf->buf_1-filter->a2*buf->buf_2;

  
  /*
  //Denomar hard TEST
  static unsigned int den_counter = 0;
  if (fabs(buf->buf_0) < (1e-30))
  {
    den_counter++;
    printf("#DENORMAL# %d\r\n",den_counter);
  }
  */
    
    
  DENORMAL_TO_ZERO(buf->buf_0);
  //y(n)=bo*w(n)+b1*w(n-1)+b2*w(n-2)
  *inputSample = filter->b0*buf->buf_0 + filter->b1*buf->buf_1+ filter->b2*buf->buf_2;

  buf->buf_2 = buf->buf_1;
  buf->buf_1 = buf->buf_0;
  
  //Process 3,4 orders if apply
  if(filter->filter_order)
  {
      //w(n)=x(n)-a1*w(n-1)-a2*w(n-2)
      buf->buf_e0 = (*inputSample)-filter->a1_1*buf->buf_e1-filter->a1_2*buf->buf_e2;
      //y(n)=bo*w(n)+b1*w(n-1)+b2*w(n-2)
      DENORMAL_TO_ZERO(buf->buf_e0);
      *inputSample =  filter->b1_0*buf->buf_e0 + filter->b1_1*buf->buf_e1+ filter->b1_2*buf->buf_e2;

      buf->buf_e2 = buf->buf_e1;
      buf->buf_e1 = buf->buf_e0;
  }
}
#endif
