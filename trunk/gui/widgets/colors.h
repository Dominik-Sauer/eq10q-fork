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
#include <glibmm/ustring.h>

#ifndef EQ10Q_COLORS_H
  #define EQ10Q_COLORS_H
  
//Wdiget background colors 
#define BACKGROUND_R 0.07
#define BACKGROUND_G 0.08
#define BACKGROUND_B 0.15

//Wdiget foreground colors 
#define FOREGROUND_R 0.0
#define FOREGROUND_G 0.9
#define FOREGROUND_B 0.0

//Text Label Color
#define TEXT_R 0.9
#define TEXT_G 0.9
#define TEXT_B 0.9

//Buttons background colors 
#define BUTTON_BACKGROUND_R 0.22
#define BUTTON_BACKGROUND_G 0.62
#define BUTTON_BACKGROUND_B 0.75

//Buttons Active background colors 
#define BUTTON_ACTIVE_BG_R 0.31
#define BUTTON_ACTIVE_BG_G 0.76
#define BUTTON_ACTIVE_BG_B 0.39

//Buttons inactive background colors 
#define BUTTON_INACTIVE_BG_R 0.15
#define BUTTON_INACTIVE_BG_G 0.35
#define BUTTON_INACTIVE_BG_B 0.45

//Buttons Mouse Over background colors 
#define BUTTON_OVER_BG_R 0.31
#define BUTTON_OVER_BG_G 0.76
#define BUTTON_OVER_BG_B 0.39

//Bands colors LUT
const  Glib::ustring bandColorLUT[] = {"#A52A2A","#FFFF00","#FFA500","#CD6E53","#FF01FF","#FF0000","#902CEE","#0000FB","#B2DFEE","#00FF00" };

//Convert to Gdk::Color macro
#define GDK_COLOR_MACRO(_color) ((gushort)floor(_color * (double)G_MAXUSHORT))

#endif