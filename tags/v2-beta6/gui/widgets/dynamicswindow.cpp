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

#include <stdlib.h>
#include <iostream>

#include <cstring>
#include <gtkmm/window.h>
#include "dynamicswindow.h"
#include "guiconstants.h"
#include "colors.h"
#include "setwidgetcolors.h"

#define KNOB_ICON_FILE "/knobs/knob2_35px.png"
#define KNOB_SIZE_X 75
#define KNOB_SIZE_Y 72
#define WIDGET_BORDER 3
#define LOGO_PATH "icons/logodynamics.png"

DynMainWindow::DynMainWindow(const char *uri, std::string bundlePath, std::string title, bool isCompressor)
  :m_pluginUri(uri),
  m_bundlePath(bundlePath),
  m_bIsCompressor(isCompressor)
{ 
  m_InGainFader = Gtk::manage(new KnobWidget2(-20.0, 20.0, "In Gain", "dB", (m_bundlePath + KNOB_ICON_FILE).c_str() ));
  m_InputVu = Gtk::manage(new VUWidget(1, -48.0, 6.0,false, true));
  m_GainReductionVu = Gtk::manage(new VUWidget(1, 0.0, 60.0, true));
  m_Attack = Gtk::manage(new KnobWidget2(0.1, 500.0, "Attack", "ms", (m_bundlePath + KNOB_ICON_FILE).c_str() , KNOB_TYPE_TIME ));
  m_Release = Gtk::manage(new KnobWidget2(5.0, 4000.0, "Release", "ms", (m_bundlePath + KNOB_ICON_FILE).c_str() , KNOB_TYPE_TIME ));
  m_HPF = Gtk::manage(new KnobWidget2(20.0, 20000.0, "Key HPF", "Hz",  (m_bundlePath + KNOB_ICON_FILE).c_str() , KNOB_TYPE_FREQ));
  m_LPF = Gtk::manage(new KnobWidget2(20.0, 20000.0, "Key LPF", "Hz",  (m_bundlePath + KNOB_ICON_FILE).c_str() , KNOB_TYPE_FREQ));

  if(m_bIsCompressor)
  {
    //Is Compressor or Expander
    m_Hold_Makeup = Gtk::manage(new KnobWidget2(0.0, 20.0, "Makeup", "dB", (m_bundlePath + KNOB_ICON_FILE).c_str() ));
    m_Range_Ratio = Gtk::manage(new KnobWidget2(1.0, 40.0, "Ratio", "dB", (m_bundlePath + KNOB_ICON_FILE).c_str() ));
    m_Knee = Gtk::manage(new KnobWidget2(0.0, 20.0, "Knee", "dB", (m_bundlePath + KNOB_ICON_FILE).c_str() ));
  }
  else
  {
    //Is Gate
    m_Hold_Makeup = Gtk::manage(new KnobWidget2(5.0, 3000.0, "Hold", "ms", (m_bundlePath + KNOB_ICON_FILE).c_str() , KNOB_TYPE_TIME ));
    m_Range_Ratio = Gtk::manage(new KnobWidget2(-90.0, -20.0, "Range", "dB", (m_bundlePath + KNOB_ICON_FILE).c_str() ));
  }
  
  m_KeyButton.set_label("Key");
  m_KeyButton.set_size_request(-1,25);
  m_ButtonAlign.add(m_KeyButton);
 
  //load image logo
  image_logo = new Gtk::Image(m_bundlePath + "/" + LOGO_PATH);
  m_LTitle.set_use_markup(true);
  m_LTitle.set_markup( "<span font_weight=\"bold\" size=\"x-large\" font_family=\"Monospace\">" + title + "</span>");
  m_LTitle.set_angle(90);
  m_TitleAlign.add(m_LTitle);
  m_TitleAlign.set(0.5,0.8,0,0);
  m_TitleBox.pack_start(m_TitleAlign, Gtk::PACK_EXPAND_WIDGET);
  m_TitleBox.pack_start(*image_logo, Gtk::PACK_SHRINK);
  m_TitleFrame.add(m_TitleBox);
  m_TitleFrame.set_label(""); //Must be empty tabler to apply the style
   
  m_VuInAlign.add(*m_InputVu);
  m_VuInAlign.set_border_width(1);
  m_VuInFrame.add(m_VuInAlign);
  m_VuInFrame.set_label("Vu&Th");
  m_VuGrAlign.add(*m_GainReductionVu);
  m_VuGrAlign.set_border_width(1);
  m_VuGrFrame.add(m_VuGrAlign);
  m_VuGrFrame.set_label("Gr");
  
  m_GattingBox.set_border_width(WIDGET_BORDER+2);
  m_GattingBox.set_spacing(WIDGET_BORDER);
  m_GattingBox.pack_start(*m_InGainFader, Gtk::PACK_SHRINK);
  m_GattingBox.pack_start(*m_Range_Ratio, Gtk::PACK_SHRINK );
  if(m_bIsCompressor)
  {
    m_GattingBox.pack_start(*m_Knee, Gtk::PACK_SHRINK );
  }
  m_GattingBox.pack_start(*m_Attack, Gtk::PACK_SHRINK );
  m_GattingBox.pack_start(*m_Release, Gtk::PACK_SHRINK);
  m_GattingBox.pack_start(*m_Hold_Makeup, Gtk::PACK_SHRINK);
  m_GattingBox.show_all_children();
  m_GattingFrame.add(m_GattingBox);
  m_GattingFrame.set_label("DYN");
  
  m_SideChainBox.set_border_width(WIDGET_BORDER);
  m_SideChainBox.set_spacing(WIDGET_BORDER * 5);
  m_SideChainBox.pack_start(*m_LPF, Gtk::PACK_SHRINK );
  m_SideChainBox.pack_start(*m_HPF, Gtk::PACK_SHRINK );
  m_SideChainBox.pack_start(m_ButtonAlign,Gtk::PACK_SHRINK);
  m_SideChainBox.show_all_children();
  m_SideChainFrame.add(m_SideChainBox);
  m_SideChainFrame.set_label("SC");
  
  m_VuBox.pack_start(m_TitleFrame, Gtk::PACK_SHRINK );
  m_VuBox.pack_start(*m_InGainFader, Gtk::PACK_SHRINK );
  m_VuBox.pack_start(m_GattingFrame, Gtk::PACK_SHRINK );
  m_VuBox.pack_start(m_VuInFrame, Gtk::PACK_SHRINK );
  m_VuBox.pack_start(m_VuGrFrame, Gtk::PACK_SHRINK );
  m_VuBox.pack_start(m_SideChainFrame, Gtk::PACK_SHRINK );
  m_VuBox.show_all_children();
  m_VuBox.show();
   
  m_MainWidgetAlign.set_padding(3,3,3,3);
  m_MainWidgetAlign.add(m_VuBox);
  add(m_MainWidgetAlign);
  m_MainWidgetAlign.show();
  
  //Set cutom theme color:
  Gdk::Color m_WinBgColor;
  SetWidgetColors m_WidgetColors;

  //Set Main widget Background
  m_WinBgColor.set_rgb(GDK_COLOR_MACRO( BACKGROUND_R ), GDK_COLOR_MACRO( BACKGROUND_G ), GDK_COLOR_MACRO( BACKGROUND_B ));
  modify_bg(Gtk::STATE_NORMAL, m_WinBgColor);
  m_WidgetColors.setGenericWidgetColors(m_SideChainFrame.get_label_widget());
  m_WidgetColors.setGenericWidgetColors(m_GattingFrame.get_label_widget());
  m_WidgetColors.setGenericWidgetColors(m_VuGrFrame.get_label_widget());
  m_WidgetColors.setGenericWidgetColors(m_VuInFrame.get_label_widget());
  m_WidgetColors.setGenericWidgetColors(m_TitleFrame.get_label_widget());
  m_WidgetColors.setButtonColors(&m_KeyButton);
  m_WidgetColors.setGenericWidgetColors(&m_LTitle);
  
  //Connect signals
  m_InGainFader->signal_changed().connect(sigc::mem_fun(*this, &DynMainWindow::onGainChange));
  m_InputVu->signal_changed().connect(sigc::mem_fun(*this, &DynMainWindow::onThresholdChange));
  m_Range_Ratio->signal_changed().connect(sigc::mem_fun(*this, &DynMainWindow::onRangeChange));
  m_Attack->signal_changed().connect(sigc::mem_fun(*this, &DynMainWindow::onAttackChange));
  m_Hold_Makeup->signal_changed().connect(sigc::mem_fun(*this, &DynMainWindow::onHoldChange));
  m_Release->signal_changed().connect(sigc::mem_fun(*this, &DynMainWindow::onReleaseChange));
  m_LPF->signal_changed().connect(sigc::mem_fun(*this, &DynMainWindow::onLPFChange));
  m_HPF->signal_changed().connect(sigc::mem_fun(*this, &DynMainWindow::onHPFChange));
  m_KeyButton.signal_clicked().connect(sigc::mem_fun(*this, &DynMainWindow::onKeyListenChange));
  if(m_bIsCompressor)
  {
    m_Knee->signal_changed().connect(sigc::mem_fun(*this, &DynMainWindow::onKneeChange));
  }
  signal_realize().connect( sigc::mem_fun(*this, &DynMainWindow::onRealize));
}

DynMainWindow::~DynMainWindow()
{
  delete m_InputVu;
  delete m_GainReductionVu;
  delete m_InGainFader;
  delete m_Attack;
  delete m_Hold_Makeup;
  delete m_Release;
  delete m_Range_Ratio;
  if(m_bIsCompressor)
  {
    delete m_Knee;
  }
  delete m_HPF;
  delete m_LPF;
  delete image_logo;
}

void DynMainWindow::onRealize()
{
  Gtk::Window* toplevel = dynamic_cast<Gtk::Window *>(this->get_toplevel()); 
  toplevel->set_resizable(false);
}

void DynMainWindow::onGainChange()
{ 
  //Write to LV2 port
  float aux;
  aux = m_InGainFader->get_value();
  write_function(controller, PORT_GAIN, sizeof(float), 0, &aux);
}

void DynMainWindow::onThresholdChange()
{
  //Write to LV2 port
  float aux;
  aux = m_InputVu->get_value_th();
  write_function(controller, PORT_THRESHOLD, sizeof(float), 0, &aux);
}

void DynMainWindow::onRangeChange()
{
  //Write to LV2 port
  float aux;
  aux = m_Range_Ratio->get_value();
  write_function(controller, PORT_RANGE, sizeof(float), 0, &aux);
}

void DynMainWindow::onAttackChange()
{
  //Write to LV2 port
  float aux;
  aux = m_Attack->get_value();
  write_function(controller, PORT_ATACK, sizeof(float), 0, &aux); 
}

void DynMainWindow::onHoldChange()
{
  //Write to LV2 port
  float aux;
  aux = m_Hold_Makeup->get_value();
  write_function(controller, PORT_HOLD, sizeof(float), 0, &aux);
}

void DynMainWindow::onReleaseChange()
{
  //Write to LV2 port
  float aux;
  aux = m_Release->get_value();
  write_function(controller, PORT_DECAY, sizeof(float), 0, &aux);
}

void DynMainWindow::onKneeChange()
{
  //Write to LV2 port
  float aux;
  aux = m_Knee->get_value();
  write_function(controller, PORT_KNEE, sizeof(float), 0, &aux);
}

void DynMainWindow::onHPFChange()
{
  //Write to LV2 port
  float aux;
  aux = m_HPF->get_value();
  write_function(controller, PORT_HPFFREQ, sizeof(float), 0, &aux);
}

void DynMainWindow::onLPFChange()
{
  //Write to LV2 port
  float aux;
  aux = m_LPF->get_value();
  write_function(controller, PORT_LPFFREQ, sizeof(float), 0, &aux);
}

void DynMainWindow::onKeyListenChange()
{
  //Write to LV2 port
  float aux;
  aux = m_KeyButton.get_active() ? 1.0 : 0.0;
  write_function(controller, PORT_KEY_LISTEN, sizeof(float), 0, &aux);
}