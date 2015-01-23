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
 
#include "button.h"
#include "colors.h"

Button::Button ( const Glib::ustring& label ):
m_label(label),
m_state(Button::RELEASE)
{
  set_size_request( 8 + 8*m_label.length(), 20);
  
  //Connect mouse signals
  add_events(Gdk::BUTTON_PRESS_MASK | Gdk::BUTTON_RELEASE_MASK | Gdk::POINTER_MOTION_MASK | Gdk::LEAVE_NOTIFY_MASK);
  signal_button_press_event().connect(sigc::mem_fun(*this, &Button::on_button_press_event),true);
  signal_button_release_event().connect(sigc::mem_fun(*this, &Button::on_button_release_event),true);
  signal_motion_notify_event().connect(sigc::mem_fun(*this, &Button::on_mouse_motion_event),true);
  signal_leave_notify_event().connect(sigc::mem_fun(*this, &Button::on_mouse_leave_widget),true);
}

Button::~Button()
{

}

bool Button::on_button_press_event ( GdkEventButton* event )
{
  //Check click type
  if(event->button == 1 && event->type == GDK_BUTTON_PRESS &&
     event->x > OUTER_BORDER &&
     event->x < (width - OUTER_BORDER) &&
     event->y > OUTER_BORDER &&
     event->y < (height - OUTER_BORDER))
  { 
    m_state = Button::PRESS;
    redraw();
  }
  return true;
}

bool Button::on_button_release_event ( GdkEventButton* event )
{
  if( event->x > OUTER_BORDER &&
    event->x < (width - OUTER_BORDER) &&
    event->y > OUTER_BORDER &&
    event->y < (height - OUTER_BORDER))
  {
    m_sigClick.emit();  
  }
 
  //Coords captured after the event cause mouse can change position during the event
  int x, y;
  get_pointer(x,y);
  
  if(x > OUTER_BORDER &&
     x < (width - OUTER_BORDER) &&
     y > OUTER_BORDER &&
     y < (height - OUTER_BORDER))
  {
    m_state = Button::FOCUS;
  }
  else
  {
    m_state = Button::RELEASE;
  }

  redraw();
  return true;
}

bool Button::on_mouse_motion_event ( GdkEventMotion* event )
{
  if( event->x > OUTER_BORDER &&
      event->x < (width - OUTER_BORDER) &&
      event->y > OUTER_BORDER &&
      event->y < (height - OUTER_BORDER))
  {
    if(m_state == Button::RELEASE)
    {
      m_state = Button::FOCUS;
    }
    else if(m_state == Button::PRESS)
    {
      m_state = Button::FOCUS_PRESS;
    }
    redraw();
  } 
  return true;
}

bool Button::on_mouse_leave_widget ( GdkEventCrossing* event )
{
  m_state = Button::RELEASE;
  redraw();
  return true;
}


void Button::set_label ( const Glib::ustring& label )
{
  m_label = label;
  redraw();
}

Button::signal_Click Button::signal_clicked()
{
  return m_sigClick;
}

void Button::redraw()
{
  Glib::RefPtr<Gdk::Window> win = get_window();
  if(win)
  {
    Gdk::Rectangle r(0, 0, get_allocation().get_width(), get_allocation().get_height());
    win->invalidate_rect(r, false);
  }
}

bool Button::on_expose_event ( GdkEventExpose* event )
{
  Glib::RefPtr<Gdk::Window> window = get_window();
  if(window)
  {
    Gtk::Allocation allocation = get_allocation();
    width = allocation.get_width();
    height = allocation.get_height();
    
    Cairo::RefPtr<Cairo::Context> cr = window->create_cairo_context();

    //Paint backgroud
    cr->save();
    cr->set_source_rgb(BACKGROUND_R, BACKGROUND_G, BACKGROUND_B);
    cr->paint(); //Fill all with background color
    cr->restore();
    
    //Draw button rectangle
    cr->save();         
    double radius = height / 5.0;
    double degrees = M_PI / 180.0;
    cr->begin_new_sub_path();
    cr->arc (width - OUTER_BORDER - radius, OUTER_BORDER + radius, radius, -90 * degrees, 0 * degrees);
    cr->arc (width - OUTER_BORDER - radius, height - OUTER_BORDER - radius, radius, 0 * degrees, 90 * degrees);
    cr->arc (OUTER_BORDER + radius, height- OUTER_BORDER - radius, radius, 90 * degrees, 180 * degrees);
    cr->arc ( OUTER_BORDER + radius, OUTER_BORDER + radius, radius, 180 * degrees, 270 * degrees);
    cr->close_path();
    switch(m_state)
    {      
      case Button::PRESS:
      cr->set_source_rgb(0.5, 0.7, 0.8);
      break;
      
      case Button::RELEASE:
      cr->set_source_rgb(0.2, 0.4, 0.5);
      break;
      
      case Button::FOCUS:
      case Button::FOCUS_PRESS:
      cr->set_source_rgb(0.2, 0.6, 0.5);
      break;
    }
    cr->set_line_width(1.8);
    cr->stroke_preserve();
      
    switch(m_state)
    {      
      case Button::PRESS:
      case Button::FOCUS_PRESS:   
        cr->set_source_rgb(0.1, 0.2, 0.3);
      break;
      
      case Button::RELEASE:
      case Button::FOCUS: 
        cr->set_source_rgb(0.2, 0.3, 0.3);
      break;  
    }
    cr->fill();
    
    cr->restore();
    
    //Label
    cr->save();
    
    switch(m_state)
    {      
      case Button::PRESS:
      cr->set_source_rgb(0.7, 0.7, 0.9);
      break;
      
      case Button::RELEASE:
      cr->set_source_rgb(TEXT_R, TEXT_G, TEXT_B);
      break;
      
      case Button::FOCUS:
      case Button::FOCUS_PRESS: 
      cr->set_source_rgb(0.2, 0.6, 0.5);
      break;
    }

    Glib::RefPtr<Pango::Layout> pangoLayout = Pango::Layout::create(cr);
    Pango::FontDescription font_desc("sans 8");
    pangoLayout->set_font_description(font_desc);
    pangoLayout->set_width(Pango::SCALE * (width - OUTER_BORDER));
    pangoLayout->set_height(Pango::SCALE * (height - OUTER_BORDER));
    pangoLayout->set_alignment(Pango::ALIGN_CENTER);
    if(m_state == Button::FOCUS_PRESS || m_state == Button::PRESS)
    {
      cr->move_to(OUTER_BORDER + 1, OUTER_BORDER + 3);
    }
    else
    {
      cr->move_to(OUTER_BORDER, OUTER_BORDER + 2);
    }
    pangoLayout->set_text(m_label.c_str());
    pangoLayout->show_in_cairo_context(cr);
    cr->stroke();  
    cr->restore();
    
  }
  
  return true;
}
