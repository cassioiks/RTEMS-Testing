/*
 *  Copyright (c) 2003 by Matt Cross <matt@dragonflyhollow.org>
 *
 *  This file is part of the firemarshalbill package.
 *
 *  Firemarshalbill is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Firemarshalbill is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Firemarshalbill; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Trace entries for high-level robot control. */

#ifndef _ROBOT_TRACE_H
#define _ROBOT_TRACE_H

#include "trace.h"

TRACE_DEFINE(ROBOT, 1000);

TRACE_ENTRIES_BEGIN(ROBOT)

     TRACE_ENTRY(ROBOT, POF, "In put_out_fire\n")

     TRACE_ENTRY(ROBOT, FIRE_OUT, "I put out the candle!\n")

     TRACE_ENTRY(ROBOT, POF_FAILED, "put_out_fire: I seem to have lost the candle :(\n")

     TRACE_ENTRY(ROBOT, STOP_TIMEOUT, "wait_for_mot_stopped: timed out after %d seconds\n")

     TRACE_ENTRY(ROBOT, STOP_HD_TIMEOUT, "wait_for_mot_heading_stopped: timed out after %d seconds\n")

     TRACE_ENTRY(ROBOT, TURN_TO, "robot_turn_to(%d.%02d, %d.%02d)\n")

     TRACE_ENTRY(ROBOT, FIND_CANDLE_RET, "find_candle() returning %d\n")

     TRACE_ENTRY(ROBOT, FIND_CANDLE, "find_candle() ret 0, real_anglel %d real_angler %d x %d y %d dist %d angle %d\n")

     TRACE_ENTRY(ROBOT, FIND_CANDLE_BADCALC, "find_candle() returning -1, angle out of range!\n")

     TRACE_ENTRY(ROBOT, POF_INIT_HD, "initial heading: %d.%02d\n")

     TRACE_ENTRY(ROBOT, HEADING, "current heading: %d.%02d\n")

     TRACE_ENTRY(ROBOT, POF_NEW_BEST, "new best w/ candle sensors: %d heading %d.%02d\n")

     TRACE_ENTRY(ROBOT, POF_NEW_BEST_UV, "new best w/ uv sensor: %d heading %d.%02d\n")

     TRACE_ENTRY(ROBOT, HUNTING_CANDLE, "got heading, hunting candle.\n")

     TRACE_ENTRY(ROBOT, FLAME_READ, "flame_read(0) = %d\n")

     TRACE_ENTRY(ROBOT, NUM_NOCANDLE, "num_nocandle_checks now %d\n")

     TRACE_ENTRY(ROBOT, HUNT_STATE, "uv_only %d, front %ld, left %ld, right %ld\n")

     TRACE_ENTRY(ROBOT, STOP_MOTORS, "stop_motors()\n")

     TRACE_ENTRY(ROBOT, HEAD_ADJ, "adjusting heading to %d.%02d\n")

     TRACE_ENTRY(ROBOT, ATTEMPT_TO_EXTINGUISH, "attempting to extinguish!\n")

     TRACE_ENTRY(ROBOT, ATTEMPT_FAILED, "attempt failed\n")

     TRACE_ENTRY(ROBOT, REFLECTION, "chasing a reflection...\n")

     TRACE_ENTRY(ROBOT, TOO_CLOSE_LEFT, "too close to left wall\n")

     TRACE_ENTRY(ROBOT, TOO_CLOSE_RIGHT, "too close to right wall\n")

     TRACE_ENTRY(ROBOT, INIT_MOVE, "Initiating move towards candle\n")

     TRACE_ENTRY(ROBOT, ALREADY_MOVE, "Already moving towards candle\n")

     TRACE_ENTRY(ROBOT, DETECT_STOP, "we've stopped!\n")

     TRACE_ENTRY(ROBOT, F16_16_DIV_ZERO, "div_f16_16(%d,%d) divide by zero!\n")

     TRACE_ENTRY(ROBOT, HEADING_UPDATE, "do_heading_update: lf %d lr %d rf %d rr %d angle %d\n")

     TRACE_ENTRY(ROBOT, ACCEPT_HEADING_UPDATE, "mot_heading_update: updating heading from %d.%02d to %d.%02d\n")

     TRACE_ENTRY(ROBOT, SET_FRONT_DIST, "set_front_dist(%d)\n")

     TRACE_ENTRY(ROBOT, FRONT_DIST, "front distance = %d\n")

     TRACE_ENTRY(ROBOT, SFD_OVERSHOT, "set_front_dist: overshot!\n")

     TRACE_ENTRY(ROBOT, SFD_ADJUSTING, "set_front_dist: adjusting dist_left %d stop_dist %d m0.pos %d startpos0 %d diff0 %d\n")

     TRACE_ENTRY(ROBOT, SFD_MOVING, "set_front_dist: moving diff=%d dir=%d\n")

     TRACE_ENTRY(ROBOT, SFD_DONE, "set_front_dist: done!\n")

     TRACE_ENTRY(ROBOT, CFFW, "check_for_front_wall: front %d speed %d moving %d m0s 0x%x dist_left 0x%x\n")

     TRACE_ENTRY(ROBOT, CFFW_WALL, "check_for_front_wall: wall in front!\n")

     TRACE_ENTRY(ROBOT, CFFW_SLOWING, "check_for_front_wall: slowing down\n")

     TRACE_ENTRY(ROBOT, CFFW_COMPLETE, "check_for_front_wall: distance complete\n")

     TRACE_ENTRY(ROBOT, DRIVE_STRAIGHT, "drive_straight desired_dir=%d.%02d stop=%d dist=%d start_speed=%d\n")

     TRACE_ENTRY(ROBOT, DRIVE_STRAIGHT_UPDATE, "drive_straight: f %d l %d r %d\n")

     TRACE_ENTRY(ROBOT, DS_WOL, "drive_straight: wall on left\n")

     TRACE_ENTRY(ROBOT, DS_WOR, "drive_straight: wall on right\n")

     TRACE_ENTRY(ROBOT, DS_NWOL, "drive_straight: no wall on left\n")

     TRACE_ENTRY(ROBOT, DS_NWOR, "drive_straight: no wall on right\n")

     TRACE_ENTRY(ROBOT, DS_NWOLR, "drive_straight: no wall on left or right\n")

     TRACE_ENTRY(ROBOT, DS_DIST_COMPLETE, "drive_straight: distance complete\n")
     TRACE_ENTRY(ROBOT, DS_DESIRED_DIR, "drive_straight: desired_dir is %d.%02d\n")

     TRACE_ENTRY(ROBOT, DS_KEEP_PARALLEL, "drive_straight: keep_parallel = 1\n")

     TRACE_ENTRY(ROBOT, ROOM1, "checking for candle in room 1.\n")
     TRACE_ENTRY(ROBOT, ROOM2, "checking for candle in room 2.\n")
     TRACE_ENTRY(ROBOT, ROOM3, "checking for candle in room 3.\n")
     TRACE_ENTRY(ROBOT, ROOM4, "checking for candle in room 4.\n")

     TRACE_ENTRY(ROBOT, MOVE, "mot_move s=%d a=%d v=%d tick=%d\n")
     TRACE_ENTRY(ROBOT, SET_VEL, "mot_move a=%d v=%d tick=%d\n")
     TRACE_ENTRY(ROBOT, SET_HD, "mot_set_heading: hd=%d.%02d hdvel=%d.%02d\n")

     TRACE_ENTRY(ROBOT, EMERGENCY, "motor task: emergency recovery mode! preverr %d pending %d\n")
     TRACE_ENTRY(ROBOT, EMERGENCY_CLEAR, "motor task: emergency recovery complete\n")
     TRACE_ENTRY(ROBOT, EMERGENCY_IGNORED, "ignored motor command due to emergency mode\n")

     TRACE_ENTRY(ROBOT, START_DECEL, "mot_do_motion: starting decel mot_v=%d mot_a=%d rampdown_t=%d rampdown_d=%d despos=0x%08x stopat=0x%08x\n")

     TRACE_ENTRY(ROBOT, DECEL_COMPLETE, "mot_do_motion: decel complete pos 0x%08x stop_at 0x%08x, off by %d\n")

TRACE_ENTRIES_END(ROBOT)

#endif /* _ROBOT_TRACE_H */
