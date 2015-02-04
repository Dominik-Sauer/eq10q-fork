/***************************************************************************
 *   Copyright (C) 2009 by Pere Ràfols Soler                               *
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

#ifndef EQ_MAIN_WIN_H
  #define EQ_MAIN_WIN_H

#include <iostream>
#include <string>

#include <gtkmm/eventbox.h>
#include <gtkmm/alignment.h>
#include <gtkmm/box.h>
#include <gtkmm/alignment.h>

#include "button.h"
#include "toggle_button.h"

#include <gtkmm/messagedialog.h>
#include <gtkmm/image.h>

#include <cmath>

//LV2 UI header
#include "../lv2_ui.h"
#include "../../forge.h"
#include "../../util.h"
#include "../../urid.h"
#include "../../uris.h"

#include "bandctl.h"
#include "gainctl.h"
#include "eqparams.h"
#include "bodeplot.h"
#include "fftctlwidget.h"

//Include eq definition
#include "../eq_defines.h"

#define IMAGE_LOGO_PATH "icons/logoeq10q.png"
#define TIMER_VALUE_MS 100

//Test print information, comment out for the final release
//#define PRINT_DEBUG_INFO

using namespace sigc;

class EqMainWindow : public Gtk::EventBox
{
  public:
    EqMainWindow(int iAudioChannels, int iNumBands, const char *uri, const char *bundlePath, const LV2_Feature *const *features);
    virtual ~EqMainWindow();   
    
    // Informing GUI about changes in the control ports
    void gui_port_event(LV2UI_Handle ui, uint32_t port, uint32_t buffer_size, uint32_t format, const void * buffer)
    {
      //Atom event from DSP
      if((int)port == (PORT_OFFSET + 2*m_iNumOfChannels + 5*m_iNumOfBands + 2*m_iNumOfChannels))
      {
         
          const LV2_Atom* atom = (const LV2_Atom*)buffer;               
          
          if (format == uris.atom_eventTransfer && atom->type == uris.atom_Object)
          {           
            const LV2_Atom_Object* obj = (const LV2_Atom_Object*)atom;
            if (obj->body.otype == uris.Dsp_2_Ui_COM)
            {
              const LV2_Atom* samplerate_val = NULL;        
              const LV2_Atom* fftdata_val = NULL;      
              const int n_props  = lv2_atom_object_get(obj, 
                                                       uris.atom_sample_rate, &samplerate_val, 
                                                       uris.atom_fft_data, &fftdata_val,
                                                       NULL);

              if (n_props != 2 || 
                  samplerate_val->type != uris.atom_Double ||
                  fftdata_val->type != uris.atom_Vector)
              {
                std::cout<<"Atom Object does not have the required properties with correct types"<<std::endl;
              }
              else
              {
                SampleRate = ((const LV2_Atom_Double*)samplerate_val)->body;
                m_Bode->setSampleRate(SampleRate);
                
                const LV2_Atom_Vector* vec = (const LV2_Atom_Vector*)fftdata_val; 
                if (vec->body.child_type != uris.atom_Double) 
                {
                   std::cout<<"Atom fft Vector has incorrect element type"<<std::endl;
                }
                else
                {

                  // Number of elements = (total size - header size) / element size
                  const size_t n_elem = ((fftdata_val->size - sizeof(LV2_Atom_Vector_Body))/ sizeof(double));
      
                  //std::cout<<"N Elem "<<n_elem << std::endl;
                  
                  //Copy data to bodeplot
                  if(n_elem == (FFT_N/2))
                  {
                    // Double elements immediately follow the vector body header
                    double* fftdata = (double*)(&vec->body + 1);
                    memcpy(m_Bode->fft_raw_data, fftdata, n_elem*sizeof(double));
                    m_Bode->setFftData();
                  }                 
                }
              }
            }
          }
      }
      
      
      //Standard LV2 ports handling
      float data = * static_cast<const float*>(buffer);
      
      #ifdef PRINT_DEBUG_INFO
	std::cout<<"gui_port_event Entring....... "<<std::endl;
      #endif
      
        // Checking if params are the same as specified in the LV2 documentation for no-atom ports
        if (format != 0) {
	    #ifdef PRINT_DEBUG_INFO
	      std::cout<<"\t-- Return Format != 0"<<std::endl;
	    #endif
            return;
        }
        if (buffer_size != 4) {  
	    #ifdef PRINT_DEBUG_INFO
	      std::cout<<"\t-- Return buffer_size != 4"<<std::endl;
	    #endif  
            return;
        }

        // Updating values in GUI ========================================================
	switch (port)
	{
	  case EQ_BYPASS:
	    m_bypassValue = data > 0.5 ? 1 : 0;
	    m_port_event_Bypass = true; //Marck port even boolean
	    ///m_BypassButton.set_active(data > 0.5);
	    #ifdef PRINT_DEBUG_INFO
	      std::cout<<"\t-- BYPASS"<<std::endl;
	    #endif  
	  break;

	  case EQ_INGAIN:
	    //m_InGainValue = data;
	    m_CurParams->setInputGain(data);
	    m_port_event_InGain = true; //Marck port even boolean
	    ///m_InGain->setGain(data);
	    
	    #ifdef PRINT_DEBUG_INFO
	      std::cout<<"\t-- Input Gain"<<std::endl;
	    #endif  
	  break;

	  case EQ_OUTGAIN:
	    //m_OutGainValue = data;
	    m_CurParams->setOutputGain(data);
	    m_port_event_OutGain = true; //Marck port even boolean
	    ///m_OutGain->setGain(data);
	    #ifdef PRINT_DEBUG_INFO
	      std::cout<<"\t-- Out Gain"<<std::endl;
	    #endif  
	  break;

	  default:
	    //Connect BandGain ports
	    if((int)port >= (PORT_OFFSET + 2*m_iNumOfChannels) && (int)port < (PORT_OFFSET + 2*m_iNumOfChannels + m_iNumOfBands))
	    {
	      ///m_BandCtlArray[(int)port - PORT_OFFSET - 2*m_iNumOfChannels]->setGain(data);
	      ///m_Bode->setBandGain((int)port - PORT_OFFSET - 2*m_iNumOfChannels, data);
	      m_CurParams->setBandGain((int)port - PORT_OFFSET - 2*m_iNumOfChannels, data);
	      m_port_event_Curve = true; //Marck port even boolean
	      m_port_event_Curve_Gain[(int)port - PORT_OFFSET - 2*m_iNumOfChannels] = true;
	      #ifdef PRINT_DEBUG_INFO
		std::cout<<"\t-- Band Gain"<<std::endl;
	      #endif  
	    }

	    //Connect BandFreq ports
	    else if((int)port >= (PORT_OFFSET + 2*m_iNumOfChannels + m_iNumOfBands) && (int)port < (PORT_OFFSET + 2*m_iNumOfChannels + 2*m_iNumOfBands))
	    {
	      ///m_BandCtlArray[(int)port - PORT_OFFSET - 2*m_iNumOfChannels - m_iNumOfBands]->setFreq(data);
	      ///m_Bode->setBandFreq((int)port - PORT_OFFSET - 2*m_iNumOfChannels - m_iNumOfBands, data);
	      m_CurParams->setBandFreq((int)port - PORT_OFFSET - 2*m_iNumOfChannels - m_iNumOfBands, data);
	      m_port_event_Curve = true; //Marck port even boolean
	      m_port_event_Curve_Freq[(int)port - PORT_OFFSET - 2*m_iNumOfChannels - m_iNumOfBands] = true;
	      #ifdef PRINT_DEBUG_INFO
		std::cout<<"\t-- Band Freq"<<std::endl;
	      #endif  
	    }

	    //Connect BandParam ports
	    else if((int)port >= (PORT_OFFSET + 2*m_iNumOfChannels + 2*m_iNumOfBands) && (int)port < (PORT_OFFSET + 2*m_iNumOfChannels + 3*m_iNumOfBands))
	    {
	      ///m_BandCtlArray[(int)port - PORT_OFFSET - 2*m_iNumOfChannels - 2*m_iNumOfBands]->setQ(data);
	      ///m_Bode->setBandQ((int)port - PORT_OFFSET - 2*m_iNumOfChannels - 2*m_iNumOfBands, data);
	      m_CurParams->setBandQ((int)port - PORT_OFFSET - 2*m_iNumOfChannels - 2*m_iNumOfBands, data);
	      m_port_event_Curve = true; //Marck port even boolean
	      m_port_event_Curve_Q[(int)port - PORT_OFFSET - 2*m_iNumOfChannels - 2*m_iNumOfBands] = true;
	      #ifdef PRINT_DEBUG_INFO
		std::cout<<"\t-- Band Q"<<std::endl;
	      #endif  
	    }

	    //Connect BandType ports
	    else if((int)port >= (PORT_OFFSET + 2*m_iNumOfChannels + 3*m_iNumOfBands) && (int)port < (PORT_OFFSET + 2*m_iNumOfChannels + 4*m_iNumOfBands))
	    {
	      ///m_BandCtlArray[(int)port - PORT_OFFSET - 2*m_iNumOfChannels - 3*m_iNumOfBands]->setFilterType(data);
	      ///m_Bode->setBandType((int)port - PORT_OFFSET - 2*m_iNumOfChannels - 3*m_iNumOfBands, data);
	      m_CurParams->setBandType((int)port - PORT_OFFSET - 2*m_iNumOfChannels - 3*m_iNumOfBands, (int)data);
	      m_port_event_Curve = true; //Marck port even boolean
	      m_port_event_Curve_Type[(int)port - PORT_OFFSET - 2*m_iNumOfChannels - 3*m_iNumOfBands] = true;
	      #ifdef PRINT_DEBUG_INFO
		std::cout<<"\t-- Band Type"<<std::endl;
	      #endif  
	    }

	    //Connect BandEnabled ports
	    else if((int)port >= (PORT_OFFSET + 2*m_iNumOfChannels + 4*m_iNumOfBands) && (int)port < (PORT_OFFSET + 2*m_iNumOfChannels + 5*m_iNumOfBands))
	    {
	      ///m_BandCtlArray[(int)port - PORT_OFFSET - 2*m_iNumOfChannels - 4*m_iNumOfBands]->setEnabled(data > 0.5);
	      ///m_Bode->setBandEnable((int)port - PORT_OFFSET - 2*m_iNumOfChannels - 4*m_iNumOfBands, data > 0.5);
	      m_CurParams->setBandEnabled((int)port - PORT_OFFSET - 2*m_iNumOfChannels - 4*m_iNumOfBands, data > 0.5);
	      m_port_event_Curve = true; //Marck port even boolean
	      m_port_event_Curve_Enable[(int)port - PORT_OFFSET - 2*m_iNumOfChannels - 4*m_iNumOfBands] = true;
	      #ifdef PRINT_DEBUG_INFO
		std::cout<<"\t-- Band Enabled"<<std::endl;
	      #endif  
	    }

	    //Connect VuInput ports
	    else if((int)port >= (PORT_OFFSET + 2*m_iNumOfChannels + 5*m_iNumOfBands) && (int)port < (PORT_OFFSET + 2*m_iNumOfChannels + 5*m_iNumOfBands + m_iNumOfChannels))
	    {
	      m_InGain->setVu((int)port - PORT_OFFSET - 2*m_iNumOfChannels - 5*m_iNumOfBands,data);
	      #ifdef PRINT_DEBUG_INFO
		std::cout<<"\t-- Vu input"<<std::endl;
	      #endif  
	    }

	    //Connect VuOutput ports
	    else if((int)port >= (PORT_OFFSET + 2*m_iNumOfChannels + 5*m_iNumOfBands + m_iNumOfChannels) && (int)port < (PORT_OFFSET + 2*m_iNumOfChannels + 5*m_iNumOfBands + 2*m_iNumOfChannels))
	    {
	      m_OutGain->setVu((int)port - PORT_OFFSET - 2*m_iNumOfChannels - 5*m_iNumOfBands - m_iNumOfChannels, data);
	      #ifdef PRINT_DEBUG_INFO
		std::cout<<"\t-- Vu output"<<std::endl;
	      #endif  
	    }
	       
	    //No more ports here
	    else
	    {
	      #ifdef PRINT_DEBUG_INFO	    
		std::cout<<"\t--  Return port index is out of range"<<std::endl;
	      #endif
	      return;
	    }
	  break;
	}       
        
	#ifdef PRINT_DEBUG_INFO	    
	  std::cout<<"\t--  Return OK"<<std::endl;
	#endif
	
    }

    LV2UI_Controller controller;
    LV2UI_Write_Function write_function;
    Eq10qURIs uris;
    LV2_URID_Map*  map;
    LV2_Atom_Forge forge;

  protected:
    EqParams *m_AParams, *m_BParams, *m_CurParams;
    BandCtl **m_BandCtlArray; 
    GainCtl *m_InGain, *m_OutGain;
    Gtk::HBox m_BandBox, m_ABFlatBox, m_GainEqBox, m_PlotBox;
    Gtk::VBox m_CurveBypassBandsBox, m_MainBox;
    ToggleButton m_BypassButton, m_AButton, m_BButton;
    Gtk::Alignment m_FlatAlign, m_ABAlign, m_ButtonAAlign, m_ButtonBAlign, m_BypassAlign, m_LoadAlign, m_SaveAlign;
    Button m_FlatButton, m_SaveButton, m_LoadButton;
    Gtk::Alignment m_MainWidgetAlign;
    PlotEQCurve *m_Bode;
    Gtk::Image *image_logo_center;
    FFTWidget *m_FftGainScale;

    void loadEqParams();
    void changeAB(EqParams *toBeCurrent);
    void saveToFile();
    void loadFromFile();
    void sendAtomFftOn(bool fft_activated);
    
    //Signal Handlers
    void onButtonA();
    void onButtonB();
    void onButtonFlat();
    void onButtonBypass();
    void onBandChange(int iBand, int iField, float fValue);
    void onInputGainChange();
    void onOutputGainChange();
    void onRealize();
    void onCurveChange(int band_ix, float Gain, float Freq, float Q);
    void onCurveBandEnable(int band_ix, bool IsEnabled);
    bool on_timeout();
    void onButtonFft();
    void onFftGainScale();
    void onBodeSelectBand(int band);
    void onBodeUnselectBand();
    void onBandCtlSelectBand(int band);
    void onBandCtlUnselectBand();
    
    //Override default signal handler:
    virtual bool on_expose_event(GdkEventExpose* event);
    
  private:
    double SampleRate;
    float m_bypassValue;   
    const int m_iNumOfChannels;
    const int m_iNumOfBands;
    bool m_bMutex, m_port_event_InGain, m_port_event_OutGain, m_port_event_Bypass, m_port_event_Curve;
    bool *m_port_event_Curve_Gain, *m_port_event_Curve_Freq, *m_port_event_Curve_Q, *m_port_event_Curve_Type, *m_port_event_Curve_Enable;
    std::string m_pluginUri;
    std::string m_bundlePath;
};

#endif
