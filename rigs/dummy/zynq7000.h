/*
 *  Hamlib TRXManager backend - main header
 *  Copyright (c) 2017 by Michael Black W9MDB
 *  Copyright (c) 2018 by Michael Black W9MDB
 *  Derived from flrig.h
 *
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifndef _ZYNQ7000_H
#define _ZYNQ7000_H 1

#include "hamlib/rig.h"

#define BACKEND_VER "20230703"

#define EOM "\r"
#define TRUE 1
#define FALSE 0

extern struct rig_caps zynq7000_caps;

extern int zynq7000_set_freq(RIG *rig, vfo_t vfo, freq_t freq);
extern HAMLIB_EXPORT(int) zynq7000_set_mode(RIG *rig, vfo_t vfo, rmode_t mode, pbwidth_t width);
extern HAMLIB_EXPORT(int) zynq7000_set_level(RIG *rig, vfo_t vfo, setting_t level, value_t val);
extern HAMLIB_EXPORT(int) zynq7000_set_ptt(RIG *rig, vfo_t vfo, ptt_t ptt);
extern HAMLIB_EXPORT(int) zynq7000_get_ptt(RIG *rig, vfo_t vfo, ptt_t *ptt);


#endif /* _ZYNQ7000_H */
