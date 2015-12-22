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
This file is the implementation of the EQ plugin
This plugin is inside the Sapista Plugins Bundle
This file implements functionalities for a large numbers of equalizers
****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <fftw3.h>

#include <lv2/lv2plug.in/ns/lv2core/lv2.h>
#include <lv2/lv2plug.in/ns/ext/atom/forge.h>
#include <lv2/lv2plug.in/ns/ext/atom/util.h>
#include <lv2/lv2plug.in/ns/ext/urid/urid.h>


#include "uris.h"

#include "gui/eq_defines.h"
#include "dsp/vu.h"
#include "dsp/db.h"
#include "dsp/filter.h"

//Data from CMake
#define NUM_BANDS @Eq_Bands_Count@
#define EQ_URI @Eq_Uri@


typedef struct {
  //Plugin ports
  float *fBypass;
  float *fInGain;
  float *fOutGain;
  float *fBandGain[NUM_BANDS];
  float *fBandFreq[NUM_BANDS];
  float *fBandParam[NUM_BANDS];
  float *fBandType[NUM_BANDS];
  float *fBandEnabled[NUM_BANDS];
  float *fInput;
  float *fOutput;
  float *fVuIn;
  float *fVuOut;
  LV2_Atom_Sequence *notify_port;
  const LV2_Atom_Sequence* control_port;

  //Features
  LV2_URID_Map *map;
  
  //Forge for creating atoms
  LV2_Atom_Forge forge;
  LV2_Atom_Forge_Frame notify_frame;
  
  //Atom URID
  Eq10qURIs uris;
  double sampleRate;
        
  //Plugin DSP
  Filter *ProcFilter[NUM_BANDS]; //Dummy pointers to Filter structures, used in processing loop.
  Filter *PortFilter[NUM_BANDS]; //Filter used for reading LV2 ports and containing the actual coeficients
  Filter *FlatFilter; //Allways contains coeficients for a flat filter in order to be used as a bypass in MidSide processing option
  Buffers buf[NUM_BANDS];
  Vu *InputVu;
  Vu *OutputVu;

  //FFT Analysis
  int fft_ix, fft_ix2; //Index to follow buffers
  double *fft_in, *fft_out;
  double *fft_in2, *fft_out2; //Time shifted-seconf-fft-vectors
  fftw_plan fft_p, fft_p2;
  int fft_on;
  double fft_normalization;
} EQ;

static void cleanupEQ(LV2_Handle instance)
{
    EQ *plugin = (EQ *)instance;
    int i;

    FilterClean(plugin->FlatFilter);
    for(i=0; i<NUM_BANDS; i++)
    {
        FilterClean(plugin->PortFilter[i]);
    }

    VuClean(plugin->InputVu);
    VuClean(plugin->OutputVu);

    fftw_destroy_plan(plugin->fft_p);
    fftw_free(plugin->fft_in); fftw_free(plugin->fft_out);
    fftw_destroy_plan(plugin->fft_p2);
    fftw_free(plugin->fft_in2); fftw_free(plugin->fft_out2);
    free(instance);
}

static void connectPortEQ(LV2_Handle instance, uint32_t port, void *data)
{
  EQ *plugin = (EQ *)instance;

  //Connect standar ports
  switch (port)
  {
    case EQ_BYPASS:
      plugin->fBypass = data;
    break;

    case EQ_INGAIN:
      plugin->fInGain = data;
    break;

    case EQ_OUTGAIN:
      plugin->fOutGain = data;
    break;

    case INPUT_PORT:
      plugin->fInput = data;
    break;
    
    case OUTPUT_PORT:
      plugin->fOutput = data;
    break;

    default:
        //Connect BandGain ports
        if( port >= (BAND_PORT_OFFSET) && port < (BAND_PORT_OFFSET + NUM_BANDS))
        {
            plugin->fBandGain[port - BAND_PORT_OFFSET] = data;
        }

        //Connect BandFreq ports
        else if(port >= (BAND_PORT_OFFSET + NUM_BANDS) && port < (BAND_PORT_OFFSET + 2*NUM_BANDS))
        {
            plugin->fBandFreq[port - BAND_PORT_OFFSET - NUM_BANDS] = data;
        }

        //Connect BandParam ports
        else if(port >= (BAND_PORT_OFFSET + 2*NUM_BANDS) && port < (BAND_PORT_OFFSET + 3*NUM_BANDS))
        {
            plugin->fBandParam[port - BAND_PORT_OFFSET - 2*NUM_BANDS] = data;
        }

        //Connect BandType ports
        else if(port >= (BAND_PORT_OFFSET + 3*NUM_BANDS) && port < (BAND_PORT_OFFSET + 4*NUM_BANDS))
        {
            plugin->fBandType[port - BAND_PORT_OFFSET - 3*NUM_BANDS] = data;
        }

        //Connect BandEnabled ports
        else if(port >= (BAND_PORT_OFFSET + 4*NUM_BANDS) && port < (BAND_PORT_OFFSET + 5*NUM_BANDS))
        {
            plugin->fBandEnabled[port - BAND_PORT_OFFSET - 4*NUM_BANDS] = data;
        }

        //Connect VuInput ports
        else if( port == (BAND_PORT_OFFSET + 5*NUM_BANDS) )
        {
            plugin->fVuIn = data;
        }

        //Connect VuOutput ports
        else if( port == BAND_PORT_OFFSET + 5*NUM_BANDS + 1 )
        {
            plugin->fVuOut = data;
        }

        //Connect Atom notify_port output port to GUI
        else if( port == BAND_PORT_OFFSET + 5*NUM_BANDS + 2 )
        {
            plugin->notify_port = (LV2_Atom_Sequence*)data;
        }

        //Connect Atom control_port input port from GUI
        else if ( port == BAND_PORT_OFFSET + 5*NUM_BANDS + 3 )
        {
            plugin->control_port = (const LV2_Atom_Sequence*)data;
        }
    break;
  }
}

static LV2_Handle instantiateEQ(const LV2_Descriptor *descriptor, double s_rate, const char *path, const LV2_Feature *const * features)
{
  int i;
  EQ *plugin_data = (EQ *)malloc(sizeof(EQ));  
  plugin_data->sampleRate = s_rate;
  
  plugin_data->FlatFilter = FilterInit(s_rate);
  calcCoefs(plugin_data->FlatFilter, 0.0, 20.0, 1.0, F_PEAK, 0.0); //Create a always-flat filter in FlatFilter
  
  for(i=0; i<NUM_BANDS; i++)
  {
    plugin_data->PortFilter[i] = FilterInit(s_rate);
    flushBuffers(&plugin_data->buf[i]);
    plugin_data->ProcFilter[i] = plugin_data->PortFilter[i]; //Initially all filters points to LV2 Port controlled filters
  }
  plugin_data->InputVu = VuInit(s_rate);
  plugin_data->OutputVu = VuInit(s_rate);
  
  // Get host features
  for (i = 0; features[i]; ++i) 
  {
    if (!strcmp(features[i]->URI, LV2_URID__map))
    {
        plugin_data->map = (LV2_URID_Map*)features[i]->data;
    } 
  }
  if (!plugin_data->map)
  {
    printf("EQ10Q Error: Host does not support urid:map\n");
    goto fail;
  }

  // Map URIs and initialise forge
  map_eq10q_uris(plugin_data->map, &plugin_data->uris);
  lv2_atom_forge_init(&plugin_data->forge, plugin_data->map);
  
  //Initialize FFT objects
  plugin_data->fft_ix = 0;
  plugin_data->fft_ix2 = FFT_N/2;
  plugin_data->fft_in = (double*) fftw_malloc(sizeof(double) * FFT_N);
  plugin_data->fft_in2 = (double*) fftw_malloc(sizeof(double) * FFT_N);
  plugin_data->fft_out = (double*) fftw_malloc(sizeof(double) * FFT_N);
  plugin_data->fft_out2 = (double*) fftw_malloc(sizeof(double) * FFT_N);
  plugin_data->fft_p = fftw_plan_r2r_1d(FFT_N, plugin_data->fft_in, plugin_data->fft_out, FFTW_R2HC, FFTW_ESTIMATE);
  plugin_data->fft_p2 = fftw_plan_r2r_1d(FFT_N, plugin_data->fft_in2, plugin_data->fft_out2, FFTW_R2HC, FFTW_ESTIMATE);
  plugin_data->fft_on = 0; //Initialy no GUI then no need to compute FFT
  plugin_data->fft_normalization = pow(2.0/ ((double) FFT_N), 2.0);
  for(i = 0; i<= FFT_N; i++)
  {
    plugin_data->fft_in[i] = 0;
    plugin_data->fft_in2[i] = 0;
    plugin_data->fft_out[i] = 0;
    plugin_data->fft_out2[i] = 0; //First fft_out2 samples will not be calculated by FFT (first-time shift)
  }
  return (LV2_Handle)plugin_data;
  
  fail:
    free(plugin_data);
    return 0;
}

static void runEQ_v2(LV2_Handle instance, uint32_t sample_count)
{

  EQ *plugin_data = (EQ *)instance;
 
  //Get values of control ports
  const int iBypass = *(plugin_data->fBypass) > 0.0f ? 1 : 0;  
  const float fInGain = dB2Lin(*(plugin_data->fInGain));
  const float fOutGain = dB2Lin(*(plugin_data->fOutGain));
  int bd, pos; //loop index
   

  //Set up forge to write directly to notify output port.
  const uint32_t notify_capacity = plugin_data->notify_port->atom.size;
  lv2_atom_forge_set_buffer(&plugin_data->forge, (uint8_t*)plugin_data->notify_port, notify_capacity);
  lv2_atom_forge_sequence_head(&plugin_data->forge, &plugin_data->notify_frame, 0);
  //printf("Notify port size %d\n", notify_capacity);
   
  //Interpolation coefs force to recompute
  int recalcCoefs[NUM_BANDS];
  int forceRecalcCoefs = 0;

  double fftInSample; //Sample to push throught the FFT buffer
  double  current_sample;
   
  //Read EQ Ports and mark to recompute if changed
  for(bd = 0; bd<NUM_BANDS; bd++)
  {
    if(dB2Lin(*(plugin_data->fBandGain[bd])) != plugin_data->PortFilter[bd]->gain ||
	*plugin_data->fBandFreq[bd] != plugin_data->PortFilter[bd]->freq ||
	*plugin_data->fBandParam[bd] != plugin_data->PortFilter[bd]->q ||
	((int)(*plugin_data->fBandType[bd])) != plugin_data->PortFilter[bd]->iType ||
	((int)(*plugin_data->fBandEnabled[bd])) != plugin_data->PortFilter[bd]->is_enabled)
    {
      recalcCoefs[bd] = 1;
      forceRecalcCoefs = 1;
    }
    else
    {
      recalcCoefs[bd] = 0;
    }
  }
  
  //Read input Atom control port (Data from GUI)
  if(plugin_data->control_port)
  {
    const LV2_Atom_Event* ev = lv2_atom_sequence_begin(&(plugin_data->control_port)->body);
    // For each incoming message...
    while (!lv2_atom_sequence_is_end(&plugin_data->control_port->body, plugin_data->control_port->atom.size, ev)) 
    {
      // If the event is an atom:Object
      if (ev->body.type == plugin_data->uris.atom_Object)
      {
        const LV2_Atom_Object* obj = (const LV2_Atom_Object*)&ev->body;
        if (obj->body.otype == plugin_data->uris.atom_fft_on)
        {
          plugin_data->fft_on = 1;
        }
        else if(obj->body.otype == plugin_data->uris.atom_fft_off)
        {
          plugin_data->fft_on = 0;
          plugin_data->fft_ix = 0;
          plugin_data->fft_ix = FFT_N/2;
        }
        else if(obj->body.otype == plugin_data->uris.atom_sample_rate_request)
        {
          //Send sample rate
          LV2_Atom_Forge_Frame frameSR;       
          lv2_atom_forge_frame_time(&plugin_data->forge, 0);
          lv2_atom_forge_object( &plugin_data->forge, &frameSR, 0, plugin_data->uris.atom_sample_rate_response); 
          lv2_atom_forge_key(&plugin_data->forge, plugin_data->uris.atom_sample_rate_key); 
          lv2_atom_forge_double(&plugin_data->forge, plugin_data->sampleRate); 
          lv2_atom_forge_pop(&plugin_data->forge, &frameSR);
                
          // Close off sequence
          lv2_atom_forge_pop(&plugin_data->forge, &plugin_data->notify_frame);
        }
      }
      ev = lv2_atom_sequence_next(ev);
    }
  }
  
  int should_send_fft = 0;

  //Compute the filter
  for (pos = 0; pos < sample_count; pos++) 
  {       
    //Get input
    current_sample = (double)plugin_data->fInput[pos];
    DENORMAL_TO_ZERO(current_sample);

    //The input amplifier
    current_sample *= fInGain;
    fftInSample = current_sample;
    //Update VU input sample
    SetSample(plugin_data->InputVu, current_sample);

    //Process every band
    if(!iBypass)
    {       
      
      //FFT of input data after input gain
      if(plugin_data->fft_on)
      {
        //Hanning Windowing
        plugin_data->fft_in[plugin_data->fft_ix] = fftInSample* 0.5 * (1.0-cos((2.0*PI*((double)plugin_data->fft_ix))/((double)(FFT_N-1))));
        plugin_data->fft_in2[plugin_data->fft_ix2] = fftInSample* 0.5 * (1.0-cos((2.0*PI*((double)plugin_data->fft_ix2))/((double)(FFT_N-1))));
        
        plugin_data->fft_ix++;     
        plugin_data->fft_ix2++;
        
        if(plugin_data->fft_ix == FFT_N)
        {
          //FFT inout buffer full compute
          fftw_execute(plugin_data->fft_p);
          
          //Compute FFT Normalized Magnitude^2 
          double real, img;
          int ffti;
          for(ffti = 0; ffti<= FFT_N/2; ffti++)
          {
            real = plugin_data->fft_out[ffti];
            if(ffti > 0 && ffti < (FFT_N/2))
            {
              img = plugin_data->fft_out[FFT_N -ffti];
            }
            else
            {
              img = 0.0;
            }
            plugin_data->fft_out[ffti] = 0.5*(plugin_data->fft_normalization*(real*real + img*img) + plugin_data->fft_out2[ffti]);
// 			if( 20.0f * log10( plugin_data->fft_out2[ffti] ) > -10.0f ){
// 				printf(
// 					"|f:%05.0f:%4.10f",
// 					ffti*(plugin_data->sampleRate/FFT_N),
// 					20.0f * log10( plugin_data->fft_out2[ffti] )
// 				);
// 			}
		  }
//           printf("|\n");
          plugin_data->fft_ix = 0;      
          

          //Send FFT data vector
		  should_send_fft = 1;
        }
        
        if(plugin_data->fft_ix2 == FFT_N)
        {
          //FFT inout buffer full compute
          fftw_execute(plugin_data->fft_p2);
          
          //Compute FFT Normalized Magnitude^2 
          double real, img;
          int ffti;
          for(ffti = 0; ffti<= FFT_N/2; ffti++)
          {
            real = plugin_data->fft_out2[ffti];
            if(ffti > 0 && ffti < (FFT_N/2))
            {
              img = plugin_data->fft_out2[FFT_N -ffti];
            }
            else
            {
              img = 0.0;
            }
            plugin_data->fft_out2[ffti] = plugin_data->fft_normalization*(real*real + img*img);
          }

          plugin_data->fft_ix2 = 0;                
        }
      }
      
      //Coefs Interpolation
      if(forceRecalcCoefs)
      {
	for(bd = 0; bd<NUM_BANDS; bd++)
	{
	  if(recalcCoefs[bd])
	  {
	    calcCoefs(plugin_data->PortFilter[bd],
		    dB2Lin(*(plugin_data->fBandGain[bd])),
		    *plugin_data->fBandFreq[bd],
		    *plugin_data->fBandParam[bd],
		    (int)(*plugin_data->fBandType[bd]), 
		    ((float)(0x01 & ((int)(*plugin_data->fBandEnabled[bd])))));
	    }
	  }
	}
      
      
      //EQ PROCESSOR
        computeFilter(plugin_data->ProcFilter[0], &plugin_data->buf[0],&current_sample);
	#if NUM_BANDS >= 4
        computeFilter(plugin_data->ProcFilter[1], &plugin_data->buf[1],&current_sample);
        computeFilter(plugin_data->ProcFilter[2], &plugin_data->buf[2],&current_sample);
        computeFilter(plugin_data->ProcFilter[3], &plugin_data->buf[3],&current_sample);
    #endif

    #if NUM_BANDS >= 6
        computeFilter(plugin_data->ProcFilter[4], &plugin_data->buf[4],&current_sample);
        computeFilter(plugin_data->ProcFilter[5], &plugin_data->buf[5],&current_sample);
    #endif

    #if NUM_BANDS ==10
        computeFilter(plugin_data->ProcFilter[6], &plugin_data->buf[6],&current_sample);
        computeFilter(plugin_data->ProcFilter[7], &plugin_data->buf[7],&current_sample);
        computeFilter(plugin_data->ProcFilter[8], &plugin_data->buf[8],&current_sample);
        computeFilter(plugin_data->ProcFilter[9], &plugin_data->buf[9],&current_sample);
    #endif

           
      //The output amplifier
      current_sample *= fOutGain;
      //Update VU output sample
      SetSample(plugin_data->OutputVu, current_sample);
    }

    //Write on output
    plugin_data->fOutput[pos] = (float)current_sample;
  }

	if (should_send_fft) {
		LV2_Atom_Forge_Frame frameFft;
		lv2_atom_forge_frame_time(&plugin_data->forge, 0);
		lv2_atom_forge_object( &plugin_data->forge, &frameFft, 0, plugin_data->uris.atom_fft_data_event);
		lv2_atom_forge_key(&plugin_data->forge, plugin_data->uris.atom_fft_data_key);
		lv2_atom_forge_vector(&plugin_data->forge, sizeof(double), plugin_data->uris.atom_Double, (FFT_N/2), plugin_data->fft_out);
		lv2_atom_forge_pop(&plugin_data->forge, &frameFft);

		// Close off sequence
		lv2_atom_forge_pop(&plugin_data->forge, &plugin_data->notify_frame);
	}

  
  //Update VU ports
  *( plugin_data->fVuIn ) = ComputeVu(plugin_data->InputVu, sample_count);
  *( plugin_data->fVuOut ) = ComputeVu(plugin_data->OutputVu, sample_count);
}

static const LV2_Descriptor eqDescriptor = {
  EQ_URI,
  instantiateEQ,
  connectPortEQ,
  NULL,
  runEQ_v2,
  NULL,
  cleanupEQ,
  NULL
};

LV2_SYMBOL_EXPORT
const LV2_Descriptor *lv2_descriptor(uint32_t index)
{
  switch (index) {
  case 0:
    return &eqDescriptor;
  default:
    return NULL;
  }
}
