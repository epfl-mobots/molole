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

#include "../error/error.h"
#include "../i2c/i2c.h"


#include "lm73.h"

lm73_temp_cb temp_cb;

// For the async temperature updated
static unsigned char swdata[1] = {0x0};
static unsigned char srdata[2];

void cb_i2c(int i2c_id, bool result) {
	temp_cb((((int) srdata[0]) << 8) | srdata[1]);
}

void lm73_set_fault_condition(int i2c_bus, int addr, int templow, int temphigh, int pol) {
	unsigned char wdata[3];
	
	wdata[0] = 0x02;
	wdata[1] = temphigh >> 8;
	wdata[2] = temphigh & 0xF0;
	i2c_master_transfert_block(i2c_bus, addr, wdata, 3, NULL, 0);
	
	wdata[0] = 0x03;
	wdata[1] = templow >> 8;
	wdata[2] = templow & 0xF0;
	i2c_master_transfert_block(i2c_bus, addr, wdata, 3, NULL, 0);
	
	
	wdata[0] = 0x01;
	wdata[1] = 0x40 | ((pol & 0x1) << 4) ; // Enable fault pin
	i2c_master_transfert_block(i2c_bus, addr, wdata, 2, NULL, 0);
}


void lm73_set_resolution(int i2c_bus, int addr, int res) {
	unsigned char wdata[2];

	wdata[0] = 0x4;
	wdata[1] = (res & 0x3) << 5;
	i2c_master_transfert_block(i2c_bus, addr, wdata, 2, NULL, 0);
}

int  lm73_temp_read_b(int i2c_bus, int addr) {
	unsigned char wdata[1];
	unsigned char rdata[2];
	wdata[0] = 0x0;
	i2c_master_transfert_block(i2c_bus, addr, wdata, 1, rdata, 2);
	return (((int) rdata[0]) << 8) | (rdata[1]);
}


void lm73_temp_read_a(int i2c_bus, int addr, lm73_temp_cb cb) {
	temp_cb = cb;
	i2c_master_transfert_async(i2c_bus, addr, swdata, 1, srdata, 2, cb_i2c);
}


