/* Copyright (c) 2017, Waterloo Autonomous Vehicles Laboratory (WAVELab),
 * Waterloo Intelligent Systems Engineering Lab (WISELab),
 * University of Waterloo.
 *
 * Refer to the accompanying LICENSE file for license information.
 *
 * ############################################################################
 ******************************************************************************
 |                                                                            |
 |                         /\/\__/\_/\      /\_/\__/\/\                       |
 |                         \          \____/          /                       |
 |                          '----________________----'                        |
 |                              /                \                            |
 |                            O/_____/_______/____\O                          |
 |                            /____________________\                          |
 |                           /    (#UNIVERSITY#)    \                         |
 |                           |[**](#OFWATERLOO#)[**]|                         |
 |                           \______________________/                         |
 |                            |_""__|_,----,_|__""_|                          |
 |                            ! !                ! !                          |
 |                            '-'                '-'                          |
 |       __    _   _  _____  ___  __  _  ___  _    _  ___  ___   ____  ____   |
 |      /  \  | | | ||_   _|/ _ \|  \| |/ _ \| \  / |/ _ \/ _ \ /     |       |
 |     / /\ \ | |_| |  | |  ||_||| |\  |||_|||  \/  |||_||||_|| \===\ |====   |
 |    /_/  \_\|_____|  |_|  \___/|_| \_|\___/|_|\/|_|\___/\___/ ____/ |____   |
 |                                                                            |
 ******************************************************************************
 * ############################################################################
 *
 * File: angles.hpp
 * Desc: Header file for angle wrapping functions
 * Auth: Michael Smart <michael.smart@uwaterloo.ca>
 *       Pranav Ganti <nav.ganti@uwaterloo.ca>
 *
 * ############################################################################
*/

#ifndef WAVE_UTILS_ANGLES_HPP
#define WAVE_UTILS_ANGLES_HPP

#include <cmath>

namespace wave {

/** Wraps input angle to the interval [-PI, PI).
 *
 * @param[in] angle the original angle.
 * @return the wrapped angle.
 */
double wrapToPi(double angle);

/** Wraps input angle to the interval [0, 2*PI).
 *
 * @param[in] angle the original angle.
 * @return the wrapped angle.
 */
double wrapToTwoPi(double angle);

}  // namespace wave
#endif  // WAVE_UTILS_ANGLES_HPP
