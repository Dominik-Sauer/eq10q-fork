/***************************************************************************
 *   Copyright (C) 2009 by Pere R�fols Soler                               *
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
 
#include "eqbutton.h"

EQButton::EQButton(int iType, float *fPtr, sigc::slot<void> slot, int *iSemafor):
m_ButtonAlign(Gtk::ALIGN_CENTER, Gtk::ALIGN_CENTER, 0.0, 0.0)
{
  m_fValue=0;
  m_ptr_f=fPtr;
  m_iStop=iSemafor;
  
  ///TODO: Un cop mes!!! Pero que collons esta fent el punter a objecte de nivell superior!?!�!?!�?!
  m_ptr_CtlButton=Gtk::manage(new CtlButton(type, &value, f_ptr, this)); //Aki el isX indica si llegirem valors de X
  
  m_TextEntry.set_numeric(true); //nomes permet numeros
  m_iFilterType = iType;
  
  switch (m_iFilterType)
  {
    case GAIN_TYPE:
      m_TextEntry.set_range(GAIN_MIN, GAIN_MAX); ///TODO: Aquest limits han de sortir d'algun lloc, el mes logic seria usar el fitxer de config del LV2 *.ttl
      m_TextEntry.set_digits(1);
    break;
    
    case FREQ_TYPE:
      m_TextEntry.set_range(FREQ_MIN, FREQ_MAX); ///TODO: Aquest limits han de sortir d'algun lloc, el mes logic seria usar el fitxer de config del LV2 *.ttl
      m_TextEntry.set_digits(1);
    break;
    
    case Q_TYPE:
      m_TextEntry.set_range(PEAK_Q_MIN, PEAK_Q_MAX); ///TODO: Aquest limits han de sortir d'algun lloc, el mes logic seria usar el fitxer de config del LV2 *.ttl
      m_TextEntry.set_digits(2);
    break;
  }
  
  m_TextEntry.set_increments(0.1, 1.0);
  

  if(m_iFilterType == FREQ_TYPE)
  {
    set_size_request(70,25);
    m_ptr_CtlButton->set_size_request(70,25); ///TODO: Un cop mes el putu punter que no se quin sentit te!!!!!!!!!!!!!!!!!!!
    m_TextEntry.set_size_request(70,25);
  }

  else 
  {
    set_size_request(45,25);
    m_ptr_CtlButton->set_size_request(50,25); ///TODO: Un cop mes el putu punter que no se quin sentit te!!!!!!!!!!!!!!!!!!!
    m_TextEntry.set_size_request(50,25);
  }
  
  m_ButtonAlign.add(*m_ptr_CtlButton); ///TODO: Un cop mes el putu punter que no se quin sentit te!!!!!!!!!!!!!!!!!!!
  pack_start(m_TextEntry,Gtk::PACK_EXPAND_PADDING);
  pack_start(m_ButtonAlign,Gtk::PACK_EXPAND_PADDING);
  
  m_TextEntry.hide();
  m_ptr_CtlButton->show(); ///TODO: Un cop mes el putu punter que no se quin sentit te!!!!!!!!!!!!!!!!!!!
  m_ButtonAlign.show();
  
  m_ptr_CtlButton->signal_button_press_event().connect(sigc::mem_fun(*this, &EQButton::onButtonDoubleClicked),false); ///TODO: Un cop mes el putu punter que no se quin sentit te!!!!!!!!!!!!!!!!!!!
  m_TextEntry.signal_activate().connect(sigc::mem_fun(*this, &EQButton::onEnterPressed));	
  m_TextEntry.signal_value_changed().connect(slot); ///TODO: Buscar una explicacio a perque cal passar el punter a slot des de fora!
  m_TextEntry.signal_value_changed().connect(sigc::mem_fun(*this, &EQButton::onSpinChange));

}

EQButton::~EQButton()
{
	//Empty distructor
}

void EQButton::setValue(float fVal)
{
  bool bOut = false;
  fValue = fVal;
  
  //Limit the value between rang acording filter type
  switch(m_iFilterType)
  {
    case GAIN_TYPE:
      if (m_fValue > GAIN_MAX) m_fValue = GAIN_MAX;
      else if (m_fValue < GAIN_MIN) m_fValue = GAIN_MIN;
    break;
    
    case FREQ_TYPE:
	  if (m_fValue > FREQ_MAX) m_fValue = FREQ_MAX;
      else if (m_fValue < FREQ_MIN) m_fValue = FREQ_MIN;
    break;
    
    case Q_TYPE:
      if (m_fValue > PEAK_Q_MAX) m_fValue = PEAK_Q_MAX;
      else if (m_fValue < PEAK_Q_MIN) m_fValue = PEAK_Q_MIN;
    break;
  }
  
   m_ptr_CtlButton->set_button_number(m_fValue); ///TODO: Pero que fot!??????!?!!?#######################

   //m_TextEntry.set_value(m_fValue); ///TODO: Revisar PQ aixo esta comentat
   setSpinNumber();
//std::cout<<"EQButton::set_value(float val)"<<"  Type: "<<m_iFilterType<<" VAL: "<<val<<std::endl;
}

float EQButton::getValue()
{
//std::cout<<"EQButton::get_value()"<<"  Type: "<<m_iFilterType<<" VALUE: "<<m_fValue<<std::endl;
	return m_fValue;
}

bool EQButton::onButtonDoubleClicked(GdkEventButton* event)
{
 if(*m_iStop == 0){
   
      if((event->type == GDK_2BUTTON_PRESS) && (event->button == 1)){
       *stop = 1;
      m_ptr_CtlButton->hide(); ///TODO: El punter a nivell superior!!! perque perque!!!!!!!!!!!!!
      m_ptr_CtlButton->set_depress(); ///TODO: El punter a nivell superior!!! perque perque!!!!!!!!!!!!!
      m_TextEntry.setValue((double)m_fValue);
      m_TextEntry.show();
      m_TextEntry.grab_focus();
      }
 
    else if(event->button == 1){
      m_ptr_CtlButton->set_press();///TODO: El punter a nivell superior!!! perque perque!!!!!!!!!!!!!
    }
    
  }
 return true; //Sempre retorna true!!! a mi que m'ho expliquin
}

///TODO: Aquesta funcio esta molt malament, la variables fAux no te cap sentit i el comentari (1) tampoc!
void EQButton::onEnterPressed()
{
  float fAux = m_fValue;
  //float val = (float)m_TextEntry.get_value(); ///TODO: Estudiar pq esta comentat
  //set_value(val); ///TODO: Estudiar pq esta comentat
  m_ptr_CtlButton->show(); ///TODO: El punter a nivell superior!!! perque perque!!!!!!!!!!!!!
  m_TextEntry.hide();
  *m_iStop = 0;
  
  ///TODO: Aixo es el comentari (1)
  /*El SLOT te un problema amb la tecla enter, el cas es ke
  En premer enter no senvia al valor actual capturat per value
  sino el valor anterior k ha pillat el spinbutton.
  Amb la seguent instruccio forzem el spin a moures a un valor
  k segur k es diferent k el seu valor actual, aixi la connexio 
  capura el valor anterio (k es el k ens interesa)  osigui "value"
  al final fem un set_value per restaurar la resta de witgets i variables
  amb el valor correcte de value 
  */
  //Fi del comentari(1)
  
  m_TextEntry.setValue((double)(value*1.1+0.1)); ///TODO: ?�?�?�?
  set_value(aux);
  
}

void EQButton::hide_spin()
{
  m_TextEntry.hide();
  m_ptr_CtlButton->show();  ///TODO: El punter a nivell superior!!! perque perque!!!!!!!!!!!!!
}

void EQButton::onSpinChange()
{
  setValue((float)m_TextEntry.get_value());
}

void EQButton::setSpinNumber(){
  m_TextEntry.set_value((double)m_fValue);
}