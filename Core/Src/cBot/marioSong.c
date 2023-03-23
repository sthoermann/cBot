/*
 * marioSong.c
 *
 * Copyright (C) 2021  Stefan Hoermann (mail@stefan-hoermann.de)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "marioSong.h"

#include "cBot.h"

void playMario() {
	playNote(f_e5, d_o);
	playPause(d_o/20);
	playNote(f_e5, d_o);
	playPause(d_o);
	playNote(f_e5, d_o);
	playPause(d_o);
	playNote(f_c5, d_o);
	playNote(f_e5, d_q);
	playNote(f_g5, d_q);
	playPause(d_q);
	playNote(f_g4, d_q);
	playPause(d_q);

	playNote(f_c5, d_q);
	playPause(d_o);
	playNote(f_g4, d_q);
	playPause(d_o);
	playNote(f_e4, d_q);
	playPause(d_o);
	playNote(f_a4, d_q);
	playNote(f_h4, d_q);
	playNote(f_b4, d_o);
	playNote(f_a4, d_q);
	playNote(f_g4, d_h/3);
	playNote(f_e5, d_h/3);
	playNote(f_g5, d_h/3);
	playNote(f_a5, d_q);
	playNote(f_f5, d_o);
	playNote(f_g5, d_o);
	playPause(d_o);
	playNote(f_e5, d_q);
	playNote(f_c5, d_o);
	playNote(f_d5, d_o);
	playNote(f_h4, d_q);
	playPause(d_o);

	playNote(f_c5, d_q);
	playPause(d_o);
	playNote(f_g4, d_q);
	playPause(d_o);
	playNote(f_e4, d_q);
	playPause(d_o);
	playNote(f_a4, d_q);
	playNote(f_h4, d_q);
	playNote(f_b4, d_o);
	playNote(f_a4, d_q);
	playNote(f_g4, d_h/3);
	playNote(f_e5, d_h/3);
	playNote(f_g5, d_h/3);
	playNote(f_a5, d_q);
	playNote(f_f5, d_o);
	playNote(f_g5, d_o);
	playPause(d_o);
	playNote(f_e5, d_q);
	playNote(f_c5, d_o);
	playNote(f_d5, d_o);
	playNote(f_h4, d_q);
	playPause(d_o);

	playPause(d_q);
	playNote(f_g5, d_o);
	playNote(f_fis5, d_o);
	playNote(f_f5, d_o);
	playNote(f_dis5, d_q);
	playNote(f_e5, d_o);
	playPause(d_o);
	playNote(f_gis4, d_o);
	playNote(f_a4, d_o);
	playNote(f_c5, d_o);
	playPause(d_o);
	playNote(f_a4, d_o);
	playNote(f_c5, d_o);
	playNote(f_d5, d_o);
	playPause(d_q);
	playNote(f_g5, d_o);
	playNote(f_fis5, d_o);
	playNote(f_f5, d_o);
	playNote(f_dis5, d_q);
	playNote(f_e5, d_o);
	playPause(d_o);
	playNote(f_c6, d_q);
	playPause(d_o/20);
	playNote(f_c6, d_o);
	playPause(d_o/20);
	playNote(f_c6, d_o);
	playPause(d_q);
	playPause(d_q);
	playNote(f_g5, d_o);
	playNote(f_fis5, d_o);
	playNote(f_f5, d_o);
	playNote(f_dis5, d_q);
	playNote(f_e5, d_o);
	playPause(d_o);
	playNote(f_gis4, d_o);
	playNote(f_a4, d_o);
	playNote(f_c5, d_o);
	playPause(d_o);
	playNote(f_a4, d_o);
	playNote(f_c5, d_o);
	playNote(f_d5, d_o);
	playPause(d_q);
	playNote(f_dis5, d_q);
	playPause(d_o);
	playNote(f_d5, d_q);
	playPause(d_o);
	playNote(f_c5, d_q);
	playPause(d_q);
	playPause(d_h);

	playPause(d_q);
	playNote(f_g5, d_o);
	playNote(f_fis5, d_o);
	playNote(f_f5, d_o);
	playNote(f_dis5, d_q);
	playNote(f_e5, d_o);
	playPause(d_o);
	playNote(f_gis4, d_o);
	playNote(f_a4, d_o);
	playNote(f_c5, d_o);
	playPause(d_o);
	playNote(f_a4, d_o);
	playNote(f_c5, d_o);
	playNote(f_d5, d_o);
	playPause(d_q);
	playNote(f_g5, d_o);
	playNote(f_fis5, d_o);
	playNote(f_f5, d_o);
	playNote(f_dis5, d_q);
	playNote(f_e5, d_o);
	playPause(d_o);
	playNote(f_c6, d_q);
	playPause(d_o/20);
	playNote(f_c6, d_o);
	playPause(d_o/20);
	playNote(f_c6, d_o);
	playPause(d_q);
	playPause(d_q);
	playNote(f_g5, d_o);
	playNote(f_fis5, d_o);
	playNote(f_f5, d_o);
	playNote(f_dis5, d_q);
	playNote(f_e5, d_o);
	playPause(d_o);
	playNote(f_gis4, d_o);
	playNote(f_a4, d_o);
	playNote(f_c5, d_o);
	playPause(d_o);
	playNote(f_a4, d_o);
	playNote(f_c5, d_o);
	playNote(f_d5, d_o);
	playPause(d_q);
	playNote(f_dis5, d_q);
	playPause(d_o);
	playNote(f_d5, d_q);
	playPause(d_o);
	playNote(f_c5, d_q);
	playPause(d_q);
	playPause(d_h);

	playNote(f_c5, d_o);
	playPause(d_o/20);
	playNote(f_c5, d_o);
	playPause(d_o);
	playNote(f_c5, d_o);
	playPause(d_o);
	playNote(f_c5, d_o);
	playNote(f_d5, d_q);
	playNote(f_e5, d_o);
	playNote(f_c5, d_o);
	playPause(d_o);
	playNote(f_a4, d_o);
	playNote(f_g4, d_h);
	playNote(f_c5, d_o);
	playPause(d_o/20);
	playNote(f_c5, d_o);
	playPause(d_o);
	playNote(f_c5, d_o);
	playPause(d_o);
	playNote(f_c5, d_o);
	playNote(f_d5, d_o);
	playNote(f_e5, d_o);
	playPause(d_f);
	playNote(f_c5, d_o);
	playPause(d_o/20);
	playNote(f_c5, d_o);
	playPause(d_o);
	playNote(f_c5, d_o);
	playPause(d_o);
	playNote(f_c5, d_o);
	playNote(f_d5, d_q);
	playNote(f_e5, d_o);
	playNote(f_c5, d_o);
	playPause(d_o);
	playNote(f_a4, d_o);
	playNote(f_g4, d_h);
	playNote(f_e5, d_o);
	playPause(d_o/20);
	playNote(f_e5, d_o);
	playPause(d_o);
	playNote(f_e5, d_o);
	playPause(d_o);
	playNote(f_c5, d_o);
	playNote(f_e5, d_q);
	playNote(f_g5, d_q);
	playPause(d_q);
	playNote(f_g4, d_q);
	playPause(d_q);

	playNote(f_c5, d_q);
	playPause(d_o);
	playNote(f_g4, d_q);
	playPause(d_o);
	playNote(f_e4, d_q);
	playPause(d_o);
	playNote(f_a4, d_q);
	playNote(f_h4, d_q);
	playNote(f_b4, d_o);
	playNote(f_a4, d_q);
	playNote(f_g4, d_h/3);
	playNote(f_e5, d_h/3);
	playNote(f_g5, d_h/3);
	playNote(f_a5, d_q);
	playNote(f_f5, d_o);
	playNote(f_g5, d_o);
	playPause(d_o);
	playNote(f_e5, d_q);
	playNote(f_c5, d_o);
	playNote(f_d5, d_o);
	playNote(f_h4, d_q);
	playPause(d_o);

	playNote(f_c5, d_q);
	playPause(d_o);
	playNote(f_g4, d_q);
	playPause(d_o);
	playNote(f_e4, d_q);
	playPause(d_o);
	playNote(f_a4, d_q);
	playNote(f_h4, d_q);
	playNote(f_b4, d_o);
	playNote(f_a4, d_q);
	playNote(f_g4, d_h/3);
	playNote(f_e5, d_h/3);
	playNote(f_g5, d_h/3);
	playNote(f_a5, d_q);
	playNote(f_f5, d_o);
	playNote(f_g5, d_o);
	playPause(d_o);
	playNote(f_e5, d_q);
	playNote(f_c5, d_o);
	playNote(f_d5, d_o);
	playNote(f_h4, d_q);
	playPause(d_o);

	playPause(d_f);
}
