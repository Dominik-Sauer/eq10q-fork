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
    float *bypass_port;
    float *in_gain_port;
    float *out_gain_port;
    float *gain_ports[NUM_BANDS];
    float *freq_ports[NUM_BANDS];
    float *q_ports[NUM_BANDS];
    float *filter_type_ports[NUM_BANDS];
    float *enabled_ports[NUM_BANDS];
    float *input_port;
    float *output_port;
    float *in_vu_meter_port;
    float *out_vu_meter_port;
    LV2_Atom_Sequence *notify_port;
    const LV2_Atom_Sequence* control_port;

    //Features
    LV2_URID_Map *map;

    //Forge for creating atoms
    LV2_Atom_Forge forge;
    LV2_Atom_Forge_Frame notify_frame;

    //Atom URID
    Eq10qURIs uris;
    double sample_rate;

    //Plugin DSP
    Filter filter[NUM_BANDS];
    FilterParams current_filter_params[NUM_BANDS];
    Buffers buf[NUM_BANDS];

    Vu *InputVu;
    Vu *OutputVu;

    //FFT Analysis
    FFT fft1;
    int is_fft_on;

    // HACK:
    int flux_counter;
} EQ;

static void cleanupEQ(LV2_Handle instance)
{
    EQ *plugin = (EQ *)instance;

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
      plugin->bypass_port = data;
    break;

    case EQ_INGAIN:
      plugin->in_gain_port = data;
    break;

    case EQ_OUTGAIN:
      plugin->out_gain_port = data;
    break;

    case INPUT_PORT:
      plugin->input_port = data;
    break;

    case OUTPUT_PORT:
      plugin->output_port = data;
    break;

    default:
        //Connect BandGain ports
        if( port >= (BAND_PORT_OFFSET) && port < (BAND_PORT_OFFSET + NUM_BANDS))
        {
            plugin->gain_ports[port - BAND_PORT_OFFSET] = data;
        }

        //Connect BandFreq ports
        else if(port >= (BAND_PORT_OFFSET + NUM_BANDS) && port < (BAND_PORT_OFFSET + 2*NUM_BANDS))
        {
            plugin->freq_ports[port - BAND_PORT_OFFSET - NUM_BANDS] = data;
        }

        //Connect BandParam ports
        else if(port >= (BAND_PORT_OFFSET + 2*NUM_BANDS) && port < (BAND_PORT_OFFSET + 3*NUM_BANDS))
        {
            plugin->q_ports[port - BAND_PORT_OFFSET - 2*NUM_BANDS] = data;
        }

        //Connect BandType ports
        else if(port >= (BAND_PORT_OFFSET + 3*NUM_BANDS) && port < (BAND_PORT_OFFSET + 4*NUM_BANDS))
        {
            plugin->filter_type_ports[port - BAND_PORT_OFFSET - 3*NUM_BANDS] = data;
        }

        //Connect BandEnabled ports
        else if(port >= (BAND_PORT_OFFSET + 4*NUM_BANDS) && port < (BAND_PORT_OFFSET + 5*NUM_BANDS))
        {
            plugin->enabled_ports[port - BAND_PORT_OFFSET - 4*NUM_BANDS] = data;
        }

        //Connect VuInput ports
        else if( port == (BAND_PORT_OFFSET + 5*NUM_BANDS) )
        {
            plugin->in_vu_meter_port = data;
        }

        //Connect VuOutput ports
        else if( port == BAND_PORT_OFFSET + 5*NUM_BANDS + 1 )
        {
            plugin->out_vu_meter_port = data;
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

static LV2_Handle instantiateEQ(const LV2_Descriptor *descriptor, double sample_rate, const char *path, const LV2_Feature *const * features)
{
    EQ *plugin = (EQ *)malloc( sizeof(EQ) );
    plugin->sample_rate = sample_rate;

    for( int band = 0; band < NUM_BANDS; band++ ) {
        FilterInit(&plugin->filter[band], sample_rate);
        FilterParamsInit(&plugin->current_filter_params[band], sample_rate);
        flushBuffers(&plugin->buf[band]);
    }

    plugin->InputVu = VuInit(sample_rate);
    plugin->OutputVu = VuInit(sample_rate);

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

    plugin->flux_counter = 0;

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
        lv2_atom_forge_double(&plugin->forge, plugin->sample_rate);
        lv2_atom_forge_pop(&plugin->forge, &frameSR);

//         // Close off sequence
//         lv2_atom_forge_pop(&plugin->forge, &plugin->notify_frame);
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

static inline void _init_lv2_atom_forge ( EQ *plugin ) {
    lv2_atom_forge_set_buffer(
        &plugin->forge,
        (uint8_t*)plugin->notify_port,
        plugin->notify_port->atom.size
    );
    lv2_atom_forge_sequence_head(
        &plugin->forge,
        &plugin->notify_frame,
        0
    );
}

static inline void _finish_lv2_atom_forge( EQ *plugin ) {
    lv2_atom_forge_pop(&plugin->forge, &plugin->notify_frame);
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
}

static inline void _send_coefs ( EQ *plugin, int band ) {
    LV2_Atom_Forge_Frame coefs_changed_frame;

    lv2_atom_forge_frame_time(&plugin->forge, 0);
    lv2_atom_forge_object(
        &plugin->forge,
        &coefs_changed_frame,
        0,
        plugin->uris.atom_band_coefs_changed
    );

    lv2_atom_forge_key(&plugin->forge, plugin->uris.atom_band_coefs_band);
    lv2_atom_forge_int(&plugin->forge, band);

    lv2_atom_forge_key(&plugin->forge, plugin->uris.atom_band_coefs_gain);
    lv2_atom_forge_double(&plugin->forge, plugin->current_filter_params[band].gain);

    lv2_atom_forge_key(&plugin->forge, plugin->uris.atom_band_coefs_freq);
    lv2_atom_forge_double(&plugin->forge, plugin->current_filter_params[band].freq);

    lv2_atom_forge_key(&plugin->forge, plugin->uris.atom_band_coefs_q);
    lv2_atom_forge_double(&plugin->forge, plugin->current_filter_params[band].q);

    lv2_atom_forge_key(&plugin->forge, plugin->uris.atom_band_coefs_filter_type);
    lv2_atom_forge_int(&plugin->forge, (int)plugin->current_filter_params[band].filter_type);

    lv2_atom_forge_key(&plugin->forge, plugin->uris.atom_band_coefs_is_enabled);
    lv2_atom_forge_int(&plugin->forge, (int)plugin->current_filter_params[band].is_enabled);

    lv2_atom_forge_pop(
        &plugin->forge,
        &coefs_changed_frame
    );
}

static inline int _is_filter_params_equal( const FilterParams const *a, const FilterParams const *b ){
    return !! memcmp(a, b, sizeof(FilterParams));
}

static inline void _copy_ports_to_filter_params( EQ *plugin, int band, FilterParams *filter_params ) {
    filter_params->gain        = dB2Lin(*(plugin->gain_ports[band]));
    filter_params->freq        = *plugin->freq_ports[band];
    filter_params->q           = *plugin->q_ports[band];
    filter_params->filter_type = (FilterType)*plugin->filter_type_ports[band];
    filter_params->is_enabled  = (int)*plugin->enabled_ports[band];
}

static void runEQ_v2( LV2_Handle instance, uint32_t sample_count ) {
    EQ *plugin = (EQ *)instance;

    //Get values of control ports
    const int iBypass = *(plugin->bypass_port) > 0.0f ? 1 : 0;
    const float in_gain_linear = dB2Lin(*(plugin->in_gain_port));
    const float out_gain_linear = dB2Lin(*(plugin->out_gain_port));

    _init_lv2_atom_forge( plugin );

    for(int band = 0; band<NUM_BANDS; band++)
    {
        _copy_ports_to_filter_params( plugin, band, plugin->current_filter_params + band );

        if( _is_filter_params_equal(
            &plugin->filter[band].params,
            plugin->current_filter_params + band
        ) )
        {
            calcCoefs(
                plugin->filter + band,
                plugin->current_filter_params + band
            );

            _send_coefs( plugin, band );
        }
    }

    _handle_control_events( plugin );

    int fft_is_ready = 0;

    //Compute the filter
    for ( int current_sample_index = 0; current_sample_index < sample_count; current_sample_index++) {
        double current_sample = (double)plugin->input_port[current_sample_index];

        DENORMAL_TO_ZERO(current_sample);
        current_sample *= in_gain_linear;

        SetSample( plugin->InputVu, current_sample );

        //Process every band
        if(!iBypass) {
            if(plugin->is_fft_on)
                fft_is_ready = add_sample_and_maybe_compute_FFT( &plugin->fft1, current_sample );

            for( int band = 0; band < NUM_BANDS; band++ ) {
                if( ! plugin->filter[band].params.is_enabled )
                    continue;

                current_sample = computeFilter(
                    plugin->filter + band,
                    plugin->buf + band,
                    current_sample
                );
            }

        }

        current_sample *= out_gain_linear;
        SetSample(plugin->OutputVu, current_sample);

        plugin->output_port[current_sample_index] = (float)current_sample;
    }

    if (fft_is_ready)
        _send_fft( plugin );

    _finish_lv2_atom_forge( plugin );

    //Update VU ports
    *( plugin->in_vu_meter_port ) = ComputeVu(plugin->InputVu, sample_count);
    *( plugin->out_vu_meter_port ) = ComputeVu(plugin->OutputVu, sample_count);
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
