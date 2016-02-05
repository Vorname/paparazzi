/*
 * Copyright (C) 2014 Andreas Pfohl
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

/** @file finken_model_sensors.h
 *  @brief module for incoming sensor data
 */

#ifndef FINKEN_MODEL_SENSORS_H
#define FINKEN_MODEL_SENSORS_H

#include "std.h"
#include "mcu_periph/link_device.h"
#include "subsystems/datalink/transport.h"
#include "subsystems/datalink/telemetry.h"
#include "math/pprz_algebra_int.h"

struct sensor_model_s {
  int16_t distance_d_front;
  int16_t distance_d_right;
  int16_t distance_d_back;
  int16_t distance_d_left;
  struct Int32Vect3 pos;
  struct Int32Quat attitude;
  struct Int32Vect3 velocity;
  struct Int32Rates rates;
  struct Int32Vect3  acceleration;
};

extern float pos_z;

extern struct sensor_model_s finken_sensor_model;

extern void finken_sensor_model_init(void);
extern void finken_sensor_model_periodic(void);

extern void send_finken_sensor_model_telemetry(struct transport_tx *trans, struct link_device* link);
void send_finken_hc_debug(struct transport_tx *trans, struct link_device* link);

#endif
