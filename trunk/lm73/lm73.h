/*
        Molole - Mobots Low Level library
        An open source toolkit for robot programming using DsPICs

        Copyright (C) 2009 Mobots group (http://mobots.epfl.ch)
        Robotics system laboratory (http://lsro.epfl.ch)
        EPFL Ecole polytechnique federale de Lausanne (http://www.epfl.ch)

        See AUTHORS for more details about other contributors.

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _MOLOLE_LM73_H
#define _MOLOLE_LM73_H

#define LM73_FAULT_POLARITY_HIGH 1
#define LM73_FAULT_POLARITY_LOW 0

#define LM73_RESOLUTION_11 0
#define LM73_RESOLUTION_12 1
#define LM73_RESOLUTION_13 2
#define LM73_RESOLUTION_14 3

typedef void (*lm73_temp_cb)(int temperature);

void lm73_set_fault_condition(int i2c_bus, int addr, int templow, int temphigh, int pol);
void lm73_set_resolution(int i2c_bus, int addr, int res);
int  lm73_temp_read_b(int i2c_bus, int addr);
void lm73_temp_read_a(int i2c_bus, int addr, lm73_temp_cb cb);


#endif

