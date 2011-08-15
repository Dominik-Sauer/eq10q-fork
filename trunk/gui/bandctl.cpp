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

#include "bandctl.h"

BandCtl::BandCtl( const int iBandNum, bool *bSemafor):
m_ButtonAlign(Gtk::ALIGN_CENTER, Gtk::ALIGN_CENTER, 0.0, 0.0),
m_ComboAlign(Gtk::ALIGN_CENTER, Gtk::ALIGN_CENTER, 0.0, 0.0),
m_iBandNum(iBandNum)
{
  m_Gain = Gtk::manage(new EQButton(GAIN_TYPE, bSemafor));
  m_Freq = Gtk::manage(new EQButton(FREQ_TYPE, bSemafor));
  m_Q = Gtk::manage(new EQButton(Q_TYPE, bSemafor));

  Glib::ustring sText = Glib::ustring::compose("Band %1", m_iBandNum);
  m_BandLabel.set_label(sText);

  pack_start(m_BandLabel, Gtk::PACK_SHRINK );
  pack_start(m_ComboAlign, Gtk::PACK_EXPAND_PADDING );
  pack_start(*m_Gain, Gtk::PACK_EXPAND_PADDING );
  pack_start(*m_Freq, Gtk::PACK_EXPAND_PADDING );
  pack_start(*m_Q, Gtk::PACK_EXPAND_PADDING );
  pack_start(m_ButtonAlign, Gtk::PACK_EXPAND_PADDING );

  m_OnButton.set_size_request(35,-1);
  m_ButtonAlign.add(m_OnButton);
  
  set_spacing(2);
  set_homogeneous(false);
  set_size_request(80,-1);

  m_FilterSel.set_size_request(60,-1);
  m_ComboAlign.add(m_FilterSel);
  
  m_ComboAlign.show();
  m_BandLabel.show();
  m_OnButton.show();
  m_FilterSel.show();
  m_Gain->show();
  m_Freq->show();
  m_Q->show();
  m_ButtonAlign.show();
  show();

  m_OnButton.set_label("ON");
  m_OnButton.signal_clicked().connect(sigc::mem_fun(*this, &BandCtl::onButtonClicked));
  m_FilterSel.signal_changed().connect(sigc::mem_fun(*this, &BandCtl::onComboChanged));
}

BandCtl::~BandCtl(){
  delete m_Gain;
  delete m_Freq;
  delete m_Q;
}

//Data accesors
float BandCtl::getGain(){
  return m_Gain->getValue();
}

float BandCtl::getFreq(){
  return m_Freq->getValue();
}

float BandCtl::getQ(){
  return m_Q->getValue();
}

float BandCtl::getFilterType(){
  return (float)m_iFilterType;
}

bool BandCtl::getEnabled()
{
  return m_bBandIsEnabled;
}

void BandCtl::setGain(float fGain){
  m_Gain->setValue(fGain);
}

void BandCtl::setFreq(float fFreq){
  m_Freq->setValue(fFreq);
}

void BandCtl::setQ(float fQ){
  m_Q->setValue(fQ);
}

void BandCtl::setFilterType(float fType){
  m_FilterSel.set_active((int)fType);
}

void BandCtl::setEnabled(bool bIsEnabled)
{
  m_OnButton.set_active(bIsEnabled);
}

//GTK signal Handlers
void BandCtl::onButtonClicked()
{
  m_bBandIsEnabled = m_OnButton.get_active();
  configSensitive();
}

void BandCtl::onComboChanged()
{
  m_iFilterType = m_FilterSel.get_active_row_number();
  configSensitive();
}

void BandCtl::configSensitive()
{
  if(!m_bBandIsEnabled)
  {
    m_Q->set_sensitive(false);
    m_Gain->set_sensitive(false);
    m_Freq->set_sensitive(false);
  }
  else
  {
    switch(m_iFilterType)
    {
      case LPF_ORDER_1:
      case HPF_ORDER_1:
	m_Q->set_sensitive(false);
	m_Gain->set_sensitive(false);
	m_Freq->set_sensitive(true);
      break;
      
      case LPF_ORDER_2:
      case LPF_ORDER_3:
      case LPF_ORDER_4:
      case HPF_ORDER_2:
      case HPF_ORDER_3:
      case HPF_ORDER_4:
	setQ(HPF_LPF_Q_DEFAULT);
	goto hpf_lpf;
      case NOTCH:
	setQ(NOTCH_Q_DEFAULT);

  hpf_lpf:
	m_Gain->set_sensitive(false);
	m_Q->set_sensitive(true);
	m_Freq->set_sensitive(true);
      break;

      case LOW_SHELF:
      case HIGH_SHELF:
	setQ(HIGH_LOW_SHELF_Q_DEFAULT);
	goto shelvings;
      case PEAK:
	setQ(PEAK_Q_DEFAULT);
  shelvings:
	m_Gain->set_sensitive(true);
	m_Q->set_sensitive(true);
	m_Freq->set_sensitive(true);
      break;
    }
  }
}
