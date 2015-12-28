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


#include <gtkmm/alignment.h>
#include <gtkmm/box.h>
#include <gtkmm/messagedialog.h>
#include <gtkmm/image.h>

#include <cmath>

//LV2 UI header
#include <lv2/lv2plug.in/ns/extensions/ui/ui.h>
#include <lv2/lv2plug.in/ns/ext/atom/forge.h>
#include <lv2/lv2plug.in/ns/ext/atom/util.h>
#include <lv2/lv2plug.in/ns/ext/urid/urid.h>
#include "../../uris.h"

#include "mainwidget.h"
#include "bandctl.h"
#include "vuwidget.h"
#include "knob2.h"
#include "eqparams.h"
#include "bodeplot.h"
#include "button.h"
#include "toggle_button.h"
#include "abbutton.h"
#include "sidechainbox.h"

//Include eq definition
#include "../eq_defines.h"

#define IMAGE_LOGO_PATH "icons/logoeq10q.png"
#define TIMER_VALUE_MS 100

//Test print information, comment out for the final release
//#define PRINT_DEBUG_INFO

using namespace sigc;

class EqMainWindow : public MainWidget
{
  public:
    EqMainWindow(int iNumBands, const char *uri, const char *bundlePath, const LV2_Feature *const *features);
    virtual ~EqMainWindow();   
    void request_sample_rate();
        
    // Informing GUI about changes in the control ports
    void gui_port_event(LV2UI_Handle ui, uint32_t port, uint32_t buffer_size, uint32_t format, const void * buffer);

    LV2UI_Controller controller;
    LV2UI_Write_Function write_function;
    Eq10qURIs uris;
    LV2_URID_Map*  map;
    LV2_Atom_Forge forge;

  protected:
    EqParams *m_AParams, *m_BParams, *m_CurParams;
    BandCtl **m_BandCtlArray; 
    Gtk::HBox m_BandBox, m_ABFlatBox, m_GainEqBox, m_PlotBox;
    Gtk::VBox m_CurveBypassBandsBox, m_MainBox, m_InGainBox, m_OutGainBox, m_FftCtlVBox, m_dBScaleBox, m_FftdBBox, m_StereoBox;
    ToggleButton m_BypassButton, m_FftRtaActive, m_FftSpecActive, m_dB10Scale, m_dB25Scale, m_dB50Scale;
    AbButton m_AButton;
    Gtk::Alignment m_FlatAlign, m_ABAlign, m_ButtonAAlign, m_BypassAlign, m_LoadAlign, m_SaveAlign, m_FftAlign, m_FftAlignInner, m_FftAlngGain, m_FftAlngRange, m_dBScaleAlign, m_dBScaleAlignInner, m_StereoInnerAlng, m_StereAlng;
    Button m_FlatButton, m_SaveButton, m_LoadButton, m_FftHold;
    Gtk::Alignment m_MainWidgetAlign;
    PlotEQCurve *m_Bode;
    Gtk::Image *image_logo_center;
    KnobWidget2 *m_GainFaderIn, *m_GainFaderOut, *m_FftGain, *m_FftRange;
    VUWidget *m_VuMeterIn, *m_VuMeterOut;
    SideChainBox *m_FftBox, *m_dBScaleFrame;
    
    void loadEqParams();
    void changeAB(EqParams *toBeCurrent);
    void saveToFile();
    void loadFromFile();
    void sendAtomFftOn(bool fft_activated);
    
    //Signal Handlers
    void onButtonA();
    void onButtonFlat();
    void onButtonBypass();
    void onBandChange(int iBand, int iField, float fValue);
    void onInputGainChange();
    void onOutputGainChange();
    void onCurveChange(int band_ix, float Gain, float Freq, float Q);
    void onCurveBandEnable(int band_ix, bool IsEnabled);
    bool on_timeout();
    void onButtonFftRta();
    void onButtonFftSpc();
    void onHoldFft_press();
    void onHoldFft_release();
    void onFftGainScale();
    void onFftRangeScale();
    void onBodeSelectBand(int band);
    void onBodeUnselectBand();
    void onBandCtlSelectBand(int band);
    void onBandCtlUnselectBand();
    
    void onDbScale10Changed();
    void onDbScale25Changed();
    void onDbScale50Changed();
    
    void onLeftRightModeSelected();
        
  private:
    double m_sample_rate;
    float m_bypassValue;
    const int m_iNumOfBands;
    bool m_bMutex, m_port_event_InGain, m_port_event_OutGain, m_port_event_Bypass, m_port_event_Curve;
    bool *m_port_event_Curve_Gain, *m_port_event_Curve_Freq, *m_port_event_Curve_Q, *m_port_event_Curve_Type, *m_port_event_Curve_Enable;
    std::string m_pluginUri;
    std::string m_bundlePath;
};

#endif
