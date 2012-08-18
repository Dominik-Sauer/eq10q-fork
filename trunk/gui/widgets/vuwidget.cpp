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

#include <iostream>
#include <cmath>
#include <cstdio>

#include "colors.h"
#include "vuwidget.h"

#define BAR_SEPARATION 0.004
#define TEXT_OFFSET 17
#define MARGIN 0.02
#define SPACE_BETWEEN_CHANNELS 0.03
#define CHANNEL_WIDTH 8
#define GREEN_BARS 40
#define YELLOW_BARS 10
#define RED_BARS 8

VUWidget::VUWidget(int iChannels, float fMin) 
  :m_iChannels(iChannels),
  m_fMin(fMin),
  m_fValues(new float[m_iChannels]),
  m_fPeaks(new float[m_iChannels])
{
  
  for (int i = 0; i < m_iChannels; i++) {
    m_fValues[i] = 0.0;
    m_fPeaks[i] = 0.0;
  }
  
  set_size_request(TEXT_OFFSET +  CHANNEL_WIDTH* m_iChannels, 150);
 
  m_fBarWidth = (1.0 - 2*MARGIN)/(float)(GREEN_BARS+YELLOW_BARS+RED_BARS) - BAR_SEPARATION;
  m_fBarStep = BAR_SEPARATION + m_fBarWidth;

}

VUWidget::~VUWidget()
{
  delete [] m_fValues;
  delete [] m_fPeaks;
}
  
void VUWidget::setValue(int iChannel, float fValue)
{
  m_fValues[iChannel] = fValue;
  if (m_fValues[iChannel] > m_fPeaks[iChannel])
  {
    m_fPeaks[iChannel] = m_fValues[iChannel];
    Glib::signal_timeout().connect_once(sigc::mem_fun(*this, &VUWidget::onTimeout), 3000);
  }
  redraw();
}

void VUWidget::onTimeout()
{
  for(int i = 0; i < m_iChannels; i++)
  {
    clearPeak(i);
  }
}

void VUWidget::clearPeak(int iChannel)
{
  m_fPeaks[iChannel] = 0.0;
  redraw();
} 

void VUWidget::redraw()
{
  Glib::RefPtr<Gdk::Window> win = get_window();
  if(win)
  {
    Gdk::Rectangle r(0, 0, get_allocation().get_width(), get_allocation().get_height());
    win->invalidate_rect(r, false);
  }
}

bool VUWidget::on_expose_event(GdkEventExpose* event)
{
  Glib::RefPtr<Gdk::Window> window = get_window();
  if(window)
  {
  
    Gtk::Allocation allocation = get_allocation();
    const int width = allocation.get_width();
    const int height = allocation.get_height();

    //Compute number of bars for each zone
    float fdBValue[m_iChannels];
    float fdBPeak[m_iChannels];
    int iActiveVu;
    float fTextOffset = TEXT_OFFSET/(float)width;
    float fTextHeight = TEXT_OFFSET/(float)height;
    float fChannelWidth = (1 - fTextOffset - 2*MARGIN)/(float)m_iChannels;
  
    //Translate input to dBu
    for(int i = 0; i<m_iChannels; i++)
    {
      if (m_fValues[i] > 0)
      {
	fdBValue[i] = 20*log10(m_fValues[i]);
      }
      else
      {
	fdBValue[i] = -60;
      }
      if (m_fPeaks[i] > 0)
      {
	fdBPeak[i] = 20*log10(m_fPeaks[i]);
      }
      else
      {
	fdBPeak[i] = -60;
      }
      fdBPeak[i] = fdBPeak[i] > 4.0 ? 4.0 : fdBPeak[i];
    }
    
    Cairo::RefPtr<Cairo::Context> cr = window->create_cairo_context();

    //Clip inside acording the expose event
    cr->rectangle(event->area.x, event->area.y, event->area.width, event->area.height);
    cr->clip();
    cr->set_source_rgb(BACKGROUND_R, BACKGROUND_G, BACKGROUND_B);
    cr->paint(); //Fill all with background color
    
    //Draw text with pango
    cr->save();
    Glib::RefPtr<Pango::Layout> pangoLayout = Pango::Layout::create(cr);
    Pango::FontDescription font_desc("sans 6.5");
    pangoLayout->set_font_description(font_desc);
    pangoLayout->set_alignment(Pango::ALIGN_RIGHT);

    cr->move_to(0.5,10);
    cr->set_source_rgba(0.9, 0.9, 1.0, 1.0);

    pangoLayout->update_from_cairo_context(cr);  //gets cairo cursor position
    //std::string sConcatena = "";
    for(int i = 0; i < GREEN_BARS + YELLOW_BARS + RED_BARS; i=i+RED_BARS)
    {
      std::stringstream ss;
      ss<<abs((i/2)-24);
      cr->move_to(3, height - (i+1)*height*m_fBarStep - height*0.045);
      pangoLayout->set_text(ss.str());
      pangoLayout->show_in_cairo_context(cr);
      cr->stroke();  
    }
   
   
    cr->restore();
    cr->scale(width, height);
    cr->translate(0, 1);
    cr->set_line_width(m_fBarWidth);
    cr->set_line_cap(Cairo::LINE_CAP_ROUND);

    
 
    for(int c; c < m_iChannels; c++)
    {
      //draw active VU in green
      cr->set_source_rgba(0.0, 0.9, 0.3, 1.0);
      for(int i = 0; i< GREEN_BARS; i++)
      {
	if(fdBValue[c] >= (float)i/2.0 - 25)
	{
	  cr->move_to(MARGIN + fTextOffset + c*fChannelWidth + SPACE_BETWEEN_CHANNELS, -MARGIN -i*m_fBarStep - m_fBarWidth/2);
	  cr->line_to(MARGIN + fTextOffset + c*fChannelWidth + fChannelWidth - SPACE_BETWEEN_CHANNELS, -MARGIN -i*m_fBarStep - m_fBarWidth/2);
	}
      }
      cr->stroke();
	  
      //draw inactive VU in green
      cr->set_source_rgba(0.0, 0.9, 0.3, 0.4);
      for(int i = 0; i< GREEN_BARS; i++)
      {
	if(fdBValue[c] < (float)i/2.0 - 25)
	{
	  cr->move_to(MARGIN + fTextOffset + c*fChannelWidth + SPACE_BETWEEN_CHANNELS, -MARGIN -i*m_fBarStep - m_fBarWidth/2);
	  cr->line_to(MARGIN + fTextOffset + c*fChannelWidth + fChannelWidth - SPACE_BETWEEN_CHANNELS, -MARGIN -i*m_fBarStep - m_fBarWidth/2);
	}
      }
      cr->stroke();
      
      //draw active VU in yellow
      cr->set_source_rgba(0.9, 0.9, 0.0, 1.0);
      for(int i = GREEN_BARS; i<GREEN_BARS + YELLOW_BARS; i++)
      { 
	if(fdBValue[c] >= (float)i/2.0 - 25)
	{
	  cr->move_to(MARGIN + fTextOffset + c*fChannelWidth + SPACE_BETWEEN_CHANNELS, -MARGIN -i*m_fBarStep - m_fBarWidth/2);
	  cr->line_to(MARGIN + fTextOffset + c*fChannelWidth + fChannelWidth - SPACE_BETWEEN_CHANNELS, -MARGIN -i*m_fBarStep - m_fBarWidth/2);
	}
      }
      cr->stroke();
      
      //draw inactive VU in yellow
      cr->set_source_rgba(0.9, 0.9, 0.0, 0.4);
      for(int i = GREEN_BARS; i<GREEN_BARS + YELLOW_BARS; i++)
      { 
	if(fdBValue[c] < (float)i/2.0 - 25)
	{
	  cr->move_to(MARGIN + fTextOffset + c*fChannelWidth + SPACE_BETWEEN_CHANNELS, -MARGIN -i*m_fBarStep - m_fBarWidth/2);
	  cr->line_to(MARGIN + fTextOffset + c*fChannelWidth + fChannelWidth - SPACE_BETWEEN_CHANNELS, -MARGIN -i*m_fBarStep - m_fBarWidth/2);
	}
      }
      cr->stroke();
      
      //draw active VU in red
      cr->set_source_rgba(0.9, 0.1, 0.0, 1.0);
      for(int i = GREEN_BARS + YELLOW_BARS; i<GREEN_BARS + YELLOW_BARS + RED_BARS; i++)
      {
	if(fdBValue[c] >= (float)i/2.0 - 25)
	{
	  cr->move_to(MARGIN + fTextOffset + c*fChannelWidth + SPACE_BETWEEN_CHANNELS, -MARGIN -i*m_fBarStep - m_fBarWidth/2);
	  cr->line_to(MARGIN + fTextOffset + c*fChannelWidth + fChannelWidth - SPACE_BETWEEN_CHANNELS, -MARGIN -i*m_fBarStep - m_fBarWidth/2);
	}
      }
      cr->stroke();
      
      //draw inactive VU in red
      cr->set_source_rgba(0.9, 0.1, 0.0, 0.4);
      for(int i = GREEN_BARS + YELLOW_BARS; i<GREEN_BARS + YELLOW_BARS + RED_BARS; i++)
      {
	if(fdBValue[c] < (float)i/2.0 - 25)
	{
	  cr->move_to(MARGIN + fTextOffset + c*fChannelWidth + SPACE_BETWEEN_CHANNELS, -MARGIN -i*m_fBarStep - m_fBarWidth/2);
	  cr->line_to(MARGIN + fTextOffset + c*fChannelWidth + fChannelWidth - SPACE_BETWEEN_CHANNELS, -MARGIN -i*m_fBarStep - m_fBarWidth/2);
	}
      }
      cr->stroke();
      
      //draw peak VU
      if (2*(fdBPeak[c] + 25) < GREEN_BARS) cr->set_source_rgba(0.0, 0.9, 0.3, 1.0);
      else if(2*(fdBPeak[c] + 25) < GREEN_BARS + YELLOW_BARS) cr->set_source_rgba(0.9, 0.9, 0.0, 1.0);
      else  cr->set_source_rgba(0.9, 0.1, 0.0, 1.0);
      cr->move_to(MARGIN + fTextOffset + c*fChannelWidth + SPACE_BETWEEN_CHANNELS, -MARGIN -2*((int)fdBPeak[c]+25)*m_fBarStep - m_fBarWidth/2);
      cr->line_to(MARGIN + fTextOffset + c*fChannelWidth + fChannelWidth - SPACE_BETWEEN_CHANNELS, -MARGIN -2*((int)fdBPeak[c]+25)*m_fBarStep - m_fBarWidth/2);
      cr->stroke();
    }
    
    /*
    //draw a rectangle arrounf the VU
    cr->set_line_cap(Cairo::LINE_CAP_SQUARE);
    //light horitzontal line
    cr->set_source_rgb(0.9, 0.9, 0.9);
    cr->set_line_width(0.008);
    cr->move_to(MARGIN + fTextOffset, -MARGIN); 
    cr->line_to(1 - MARGIN, -MARGIN);
    cr->stroke();
    
    //ligth vertical line
    cr->set_line_width(0.04);
    cr->move_to(1 - MARGIN, -MARGIN);
    cr->line_to(1 - MARGIN, -1 + MARGIN);
    cr->stroke();
    
    //Dark Horitzontal line
    cr->set_line_width(0.008);
    cr->set_source_rgb(0.5, 0.5, 0.5);
    cr->move_to(1 - MARGIN, -1 + MARGIN);
    cr->line_to(MARGIN + fTextOffset, -1 + MARGIN);
    cr->stroke();
    
    //Dark Vertical line_to
    cr->set_line_width(0.04);
    cr->move_to(MARGIN + fTextOffset, -1 + MARGIN);
    cr->line_to(MARGIN + fTextOffset, -MARGIN);
    cr->stroke();
    */
    
    /*
    //draw a rectangle arround the VU Widget
    cr->set_source_rgb(0.22, 0.30, 0.53);

     //top horitzontal line
    cr->set_line_width(0.008);
    cr->move_to(0, 0); 
    cr->line_to(1, 0);
    cr->stroke();
    
    //right vertical line
    cr->set_line_width(0.04);
    cr->move_to(1, 0);
    cr->line_to(1, -1);
    cr->stroke();
    
    //bottom Horitzontal line
    cr->set_line_width(0.008);
    cr->move_to(1, -1);
    cr->line_to(0, -1);
    cr->stroke();
    
    //left Vertical line_to
    cr->set_line_width(0.04);
    cr->move_to(0, -1);
    cr->line_to(0, 0);
    cr->stroke();
    */
    
  }
  return true;  
}