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

#include <lv2/lv2plug.in/ns/lv2core/lv2.h>
#include <lv2/lv2plug.in/ns/ext/atom/forge.h>
#include <lv2/lv2plug.in/ns/ext/atom/util.h>
#include <lv2/lv2plug.in/ns/ext/urid/urid.h>

#include "uris.h"

#include "gui/eq_defines.h"
#include "dsp/vu.h"
#include "dsp/db.h"
#include "dsp/filter.h"
#include "dsp/fft.h"

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
    Filter *filter[NUM_BANDS];
    Buffers buf[NUM_BANDS];

    Vu *InputVu;
    Vu *OutputVu;

    //FFT Analysis
    FFT fft1;
    int is_fft_on;
} EQ;

static void cleanupEQ(LV2_Handle instance)
{
    EQ *plugin = (EQ *)instance;

    for(int i=0; i<NUM_BANDS; i++)
        FilterClean(plugin->filter[i]);

    VuClean(plugin->InputVu);
    VuClean(plugin->OutputVu);

    cleanup_FFT(&plugin->fft1);

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
    EQ *plugin = (EQ *)malloc( sizeof(EQ) );
    plugin->sampleRate = s_rate;

    for( int i = 0; i < NUM_BANDS; i++ ) {
        plugin->filter[i] = FilterInit(s_rate);
        flushBuffers(&plugin->buf[i]);
    }

    plugin->InputVu = VuInit(s_rate);
    plugin->OutputVu = VuInit(s_rate);

    for (int i = 0; features[i]; i++ ) {
        if( !strcmp( features[i]->URI, LV2_URID__map ) )
            plugin->map = (LV2_URID_Map*)features[i]->data;
    }

    if ( !plugin->map ) {
        printf("EQ10Q Error: Host does not support urid:map\n");
        goto fail;
    }

    map_eq10q_uris( plugin->map, &plugin->uris );
    lv2_atom_forge_init( &plugin->forge, plugin->map );

    initialize_FFT( &plugin->fft1, FFT_N, 0 );
    plugin->is_fft_on = 0;

    return (LV2_Handle)plugin;

    fail:
        free( plugin );
        return 0;
}

#define _is_fft_on_requested( plugin, obj )        ( obj->body.otype == plugin->uris.atom_fft_on )
#define _is_fft_off_requested( plugin, obj )       ( obj->body.otype == plugin->uris.atom_fft_off )
#define _is_sample_rate_requested( plugin, obj )   ( obj->body.otype == plugin->uris.atom_sample_rate_request )

static inline void _handle_control_event ( EQ *plugin, const LV2_Atom_Event *atom_event ) {
    if ( atom_event->body.type != plugin->uris.atom_Object )
        return;

    const LV2_Atom_Object* obj = (const LV2_Atom_Object*)&atom_event->body;

    if ( _is_fft_on_requested( plugin, obj ) ) {
        plugin->is_fft_on = 1;
        return;
    }

    if( _is_fft_off_requested( plugin, obj ) ) {
        plugin->is_fft_on = 0;
        reset_FFT( &plugin->fft1, 0 );
        return;
    }

    if( _is_sample_rate_requested( plugin, obj ) ){
        //Send sample rate
        LV2_Atom_Forge_Frame frameSR;
        lv2_atom_forge_frame_time(&plugin->forge, 0);
        lv2_atom_forge_object( &plugin->forge, &frameSR, 0, plugin->uris.atom_sample_rate_response);
        lv2_atom_forge_key(&plugin->forge, plugin->uris.atom_sample_rate_key);
        lv2_atom_forge_double(&plugin->forge, plugin->sampleRate);
        lv2_atom_forge_pop(&plugin->forge, &frameSR);

        // Close off sequence
        lv2_atom_forge_pop(&plugin->forge, &plugin->notify_frame);
    }
}

static inline void _handle_control_events ( EQ *plugin ) {
    if( ! plugin->control_port )
        return;

    const LV2_Atom_Event* atom_event
        = lv2_atom_sequence_begin( &(plugin->control_port)->body );

    while(
        ! lv2_atom_sequence_is_end(
            &plugin->control_port->body,
            plugin->control_port->atom.size,
            atom_event
        )
    ) {
        _handle_control_event( plugin, atom_event );

        atom_event = lv2_atom_sequence_next( atom_event );
    }
}

static inline void _send_fft ( EQ *plugin ) {
    LV2_Atom_Forge_Frame frameFft;
    lv2_atom_forge_frame_time(&plugin->forge, 0);
    lv2_atom_forge_object( &plugin->forge, &frameFft, 0, plugin->uris.atom_fft_data_event);
    lv2_atom_forge_key(&plugin->forge, plugin->uris.atom_fft_data_key);
    lv2_atom_forge_vector(
        &plugin->forge,
        sizeof(double),
        plugin->uris.atom_Double,
        plugin->fft1.half_size,
        plugin->fft1.frequency_samples
    );
    lv2_atom_forge_pop(&plugin->forge, &frameFft);

    // Close off sequence
    lv2_atom_forge_pop(&plugin->forge, &plugin->notify_frame);
}

static void runEQ_v2( LV2_Handle instance, uint32_t sample_count ) {
    EQ *plugin = (EQ *)instance;

    //Get values of control ports
    const int iBypass = *(plugin->fBypass) > 0.0f ? 1 : 0;
    const float fInGain = dB2Lin(*(plugin->fInGain));
    const float fOutGain = dB2Lin(*(plugin->fOutGain));

    //Set up forge to write directly to notify output port.
    const uint32_t notify_capacity = plugin->notify_port->atom.size;
    lv2_atom_forge_set_buffer(&plugin->forge, (uint8_t*)plugin->notify_port, notify_capacity);
    lv2_atom_forge_sequence_head(&plugin->forge, &plugin->notify_frame, 0);
    //printf("Notify port size %d\n", notify_capacity);

    //Interpolation coefs force to recompute
    int recalcCoefs[NUM_BANDS];
    int forceRecalcCoefs = 0;

    double current_sample;

    //Read EQ Ports and mark to recompute if changed
    for(int band = 0; band<NUM_BANDS; band++)
    {
        if(
            dB2Lin(*(plugin->fBandGain[band])) != plugin->filter[band]->gain
            ||
            *plugin->fBandFreq[band] != plugin->filter[band]->freq
            ||
            *plugin->fBandParam[band] != plugin->filter[band]->q
            ||
            ((int)(*plugin->fBandType[band])) != plugin->filter[band]->filter_type
            ||
            ((int)(*plugin->fBandEnabled[band])) != plugin->filter[band]->is_enabled
        ) {
            recalcCoefs[band] = 1;
            forceRecalcCoefs = 1;
        } else {
            recalcCoefs[band] = 0;
        }
    }

    //Read input Atom control port (Data from GUI)
    _handle_control_events( plugin );

    int fft_is_ready = 0;

    //Compute the filter
    for ( int current_sample_index = 0; current_sample_index < sample_count; current_sample_index++) {
        current_sample = (double)plugin->fInput[current_sample_index];
        DENORMAL_TO_ZERO(current_sample);
        current_sample *= fInGain;

        SetSample(plugin->InputVu, current_sample);

        //Process every band
        if(!iBypass) {
            if(plugin->is_fft_on)
                fft_is_ready = add_sample_and_maybe_compute_FFT( &plugin->fft1, current_sample );

            //Coefs Interpolation
            if( forceRecalcCoefs ) {
                for( int band = 0; band < NUM_BANDS; band++ ) {
                    if(recalcCoefs[band])
                        calcCoefs(plugin->filter[band],
                            dB2Lin(*(plugin->fBandGain[band])),
                            *plugin->fBandFreq[band],
                            *plugin->fBandParam[band],
                            (int)(*plugin->fBandType[band]),
                            (int)(*plugin->fBandEnabled[band])
                        );
                }
            }

            for( int band = 0; band < NUM_BANDS; band++ )
                current_sample = computeFilter(
                    plugin->filter[band],
                    &plugin->buf[band],
                    current_sample
                );

        }

        current_sample *= fOutGain;
        SetSample(plugin->OutputVu, current_sample);

        plugin->fOutput[current_sample_index] = (float)current_sample;
    }

    if (fft_is_ready)
        _send_fft( plugin );

    //Update VU ports
    *( plugin->fVuIn ) = ComputeVu(plugin->InputVu, sample_count);
    *( plugin->fVuOut ) = ComputeVu(plugin->OutputVu, sample_count);
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
