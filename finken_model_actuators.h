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

/** @file finken_model_actuators.h
 *  @brief module for actuators
 */

#ifndef FINKEN_MODEL_ACTUATORS_H
#define FINKEN_MODEL_ACTUATORS_H

#include "mcu_periph/link_device.h"
#include "subsystems/datalink/transport.h"
#include "subsystems/datalink/telemetry.h"

struct actuators_model_s {
	float pitch;
	float roll;
	float yaw;
	float thrust;
	// int vsupply_tel; //noch nicht Funktionstauglich
};

extern struct actuators_model_s finken_actuators_model;
extern struct actuators_model_s finken_actuators_set_point;

extern void finken_actuators_model_init(void);
extern void finken_actuators_model_periodic(void);

extern void send_finken_actuators_model_telemetry(struct transport_tx *trans, struct link_device* link);
extern void send_finken_actuators_set_point_telemetry(struct transport_tx *trans, struct link_device* link);
extern float compensate_battery_drop(float thrust_setpoint);
#endif
