/*
 * Copyright (C) 2014 Sebastian Mai
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "modules/finken_model/finken_model_actuators.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/electrical.h"

struct actuators_model_s finken_actuators_model;
struct actuators_model_s finken_actuators_set_point;


void finken_actuators_model_init(void) {
	finken_actuators_model.pitch  = 0;
	finken_actuators_model.roll   = 0;
	finken_actuators_model.yaw    = 0;
	finken_actuators_model.thrust = 0;

	finken_actuators_set_point.pitch  = 0;
	finken_actuators_set_point.roll   = 0;
	finken_actuators_set_point.yaw    = 0;
	finken_actuators_set_point.thrust = 0;

	register_periodic_telemetry(DefaultPeriodic, "FINKEN_ACTUATORS_MODEL", send_finken_actuators_model_telemetry);
	register_periodic_telemetry(DefaultPeriodic, "FINKEN_ACTUATORS_SET_POINT", send_finken_actuators_set_point_telemetry);
}

void finken_actuators_model_periodic(void) {
	finken_actuators_model.pitch  = finken_actuators_set_point.pitch;
	finken_actuators_model.roll   = finken_actuators_set_point.roll;
	finken_actuators_model.yaw    = finken_actuators_set_point.yaw;
	finken_actuators_model.thrust = compensate_battery_drop(finken_actuators_set_point.thrust);
}


float compensate_battery_drop(float thrust_setpoint) {
	if(electrical.vsupply >= (MAX_BAT_LEVEL*10) - 3) // Solange die Akkuspannung groesser als maximale Akkuspannung -3 ist
		return thrust_setpoint; 
	uint16_t vsup_loss_procent = 1 - vsup/120;
	float compThrust = 0.7793256*pow(vsup_loss_procent,3)-0.0512096*pow(vsup_loss_procent,2)+0.0094144*vsup_loss_procent+0.3680203;	
	float thrust = thrust_setpoint compThrust;
	
	if(thrust < 0.2)
		return 0.2;
	else if(thrust > 1.0)
		return 1.0;
	return thrust;
}



void send_finken_actuators_model_telemetry(struct transport_tx *trans, struct link_device* link)
{
	trans=trans;
	link=link;
	DOWNLINK_SEND_FINKEN_ACTUATORS_MODEL(
		DefaultChannel,
		DefaultDevice,
		&finken_actuators_model.pitch,
		&finken_actuators_model.roll,
		&finken_actuators_model.yaw,
		&finken_actuators_model.thrust
	);
}
void send_finken_actuators_set_point_telemetry(struct transport_tx *trans, struct link_device* link)
{
	trans=trans;
	link=link;
	DOWNLINK_SEND_FINKEN_ACTUATORS_SET_POINT(
		DefaultChannel,
		DefaultDevice,
		&finken_actuators_set_point.pitch,
		&finken_actuators_set_point.roll,
		&finken_actuators_set_point.yaw,
		&finken_actuators_set_point.thrust
	);
}
