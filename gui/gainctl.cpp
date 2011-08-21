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

#include "gainctl.h"
#include "guiconstants.h"

GainCtl::GainCtl(const Glib::ustring sTitle, int iNumOfChannel, bool bTrueIfIn):
m_bTrueIfIn(bTrueIfIn),
m_iNumOfChannels(iNumOfChannel)
{
  m_GainScale.set_digits(1);
  m_GainScale.set_draw_value(true);
  m_GainScale.set_value_pos(Gtk::POS_TOP);
  m_GainScale.set_inverted(true);
  m_GainScale.set_range(GAIN_MIN, GAIN_MAX);
  m_GainScale.set_value(GAIN_DEFAULT);
  m_GainLabel.set_label(sTitle);
  pack_start(m_GainLabel);
  pack_start(m_GainScale);
  set_spacing(2);
  set_homogeneous(false);
  m_GainScale.set_size_request(40,100); 
///TODO: Invoke construtor for VUwidget
  m_GainLabel.show();
  m_GainScale.show();
  show();
  
  m_GainScale.signal_value_changed().connect(sigc::mem_fun(*this, &GainCtl::onGainChanged));
}

void GainCtl::setGain(float fValue){
  m_GainScale.set_value((double) fValue);
}

float GainCtl::getGain(){
  return (float)m_GainScale.get_value();
}

GainCtl::signal_GainChanged GainCtl::signal_changed()
{
  return m_GainChangedSignal;
}

void GainCtl::onGainChanged()
{
  m_GainChangedSignal.emit(m_bTrueIfIn, getGain());
}


GainCtl::~GainCtl(){
///TODO: Delet the VU pointer
}
