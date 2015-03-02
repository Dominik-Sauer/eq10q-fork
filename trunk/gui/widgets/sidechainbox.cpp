/***************************************************************************
 *   Copyright (C) 2015 by Pere Ràfols Soler                               *
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
 
#include "sidechainbox.h"
#include "colors.h"

#define TOP_PADDING 20
#define MARGIN 6
#define RADIUS 4

SideChainBox::SideChainBox()
{

}

SideChainBox::~SideChainBox()
{

}


bool SideChainBox::on_expose_event(GdkEventExpose* event)
{
  bool ret = Gtk::EventBox::on_expose_event(event); //Call parent redraw()
  
  Glib::RefPtr<Gdk::Window> window = get_window();
  if(window)
  {
    Gtk::Allocation allocation = get_allocation();
    const int width = allocation.get_width();
    const int height = allocation.get_height();
    
    Cairo::RefPtr<Cairo::Context> cr = window->create_cairo_context();

    //Paint backgroud
    cr->save();
    cr->set_source_rgb(BACKGROUND_R, BACKGROUND_G, BACKGROUND_B);
    cr->paint(); //Fill all with background color
    cr->restore();
    
    //Draw a box
    cr->save();
    cr->arc( MARGIN + 0.5, MARGIN + TOP_PADDING + 0.5, RADIUS, M_PI, -0.5*M_PI);
    cr->line_to(width/2 - 34 , MARGIN + TOP_PADDING + 0.5 - RADIUS);
    cr->move_to(width/2 + 34 , MARGIN + TOP_PADDING + 0.5 - RADIUS);
    cr->line_to(width - 1 - MARGIN - 0.5, MARGIN + TOP_PADDING + 0.5 - RADIUS);
    cr->arc( width - 1 - MARGIN - 0.5, MARGIN + TOP_PADDING + 0.5, RADIUS, -0.5*M_PI, 0);
    cr->line_to(width - 1 - MARGIN - 0.5 + RADIUS, height - 1 - MARGIN  - 0.5);
    cr->arc( width - 1 - MARGIN - 0.5, height - 1 - MARGIN  - 0.5, RADIUS, 0.0,  0.5*M_PI);
    cr->line_to( MARGIN + 0.5, height - 1 - MARGIN  - 0.5 + RADIUS);
    cr->arc( MARGIN  + 0.5, height - 1 - MARGIN  - 0.5, RADIUS, 0.5*M_PI, M_PI);
    cr->line_to( MARGIN + 0.5 - RADIUS,  MARGIN + TOP_PADDING + 0.5 );
    cr->set_line_width(1);
    cr->set_source_rgba(1,1,1, 0.3);
    cr->stroke();
    cr->restore();
    
     //Draw Text FFT
    cr->save();
    Glib::RefPtr<Pango::Layout> pangoLayout = Pango::Layout::create(cr);
    Pango::FontDescription font_desc("sans 12px");
    pangoLayout->set_font_description(font_desc);
    pangoLayout->set_text("Side-Chain");

    //and text
    cr->move_to(width/2 - 32, MARGIN + TOP_PADDING/2 );
    cr->set_source_rgba(0.9, 0.9, 0.9, 0.7);
    pangoLayout->show_in_cairo_context(cr);
    cr->stroke();  
    cr->restore();
  }

  return ret;
}

