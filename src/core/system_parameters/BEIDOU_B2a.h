/*!
 * \file BEIDOU_B2a.h
 * \brief  Defines system parameters for BEIDOU B2a signal
 * \author Sara Hrbek, 2018. sara.hrbek(at)gmail.com
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2018  (see AUTHORS file for a list of contributors)
 *
 * GNSS-SDR is a software defined Global Navigation
 *          Satellite Systems receiver
 *
 * This file is part of GNSS-SDR.
 *
 * GNSS-SDR is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNSS-SDR is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_BEIDOU_B2a_H_
#define GNSS_SDR_BEIDOU_B2a_H_

#include "gnss_frequencies.h"
#include "GPS_CNAV.h"
#include "MATH_CONSTANTS.h"
#include <cstdint>
#include <string>


// Physical constants//TODO this is for DK to fill in if they are indeed needed.
/*
const double BEIDOU_B2a_C_m_s = 299792458.0;                 //!< The speed of light, [m/s]
;const double BEIDOU_B2a_C_m_ms = 299792.4580;               //!< The speed of light, [m/ms]
;const double BEIDOU_B2a_PI = 3.1415926535898;               //!< Pi as defined in IS-GPS-200E
;const double BEIDOU_B2a_TWO_PI = 6.283185307179586;         //!< 2Pi as defined in IS-GPS-200E
;const double BEIDOU_B2a_OMEGA_EARTH_DOT = 7.2921151467e-5;  //!< Earth rotation rate, [rad/s]
;const double BEIDOU_B2a_GM = 3.986005e14;                   //!< Universal gravitational constant times the mass of the Earth, [m^3/s^2]
;const double BEIDOU_B2a_F = -4.442807633e-10;               //!< Constant, [s/(m)^(1/2)]
*/

// carrier and code frequencies
const double BEIDOU_B2a_FREQ_HZ = FREQ5;  //!< L5 [Hz]

const double BEIDOU_B2ad_CODE_RATE_HZ = 10.23e6;  //!< BEIDOU_B2a data code rate [chips/s]
const int BEIDOU_B2ad_CODE_LENGTH_CHIPS = 10230;  //!< BEIDOU_B2a data  code length [chips]
const double BEIDOU_B2ad_PERIOD = 0.001;          //!< BEIDOU_B2a data code period [seconds]
const double BEIDOU_B2ad_SYMBOL_PERIOD = 0.01;    //!< BEIDOU_B2a data symbol period [seconds]

const double BEIDOU_B2ap_CODE_RATE_HZ = 10.23e6;  //!< BEIDOU_B2a pilot code rate [chips/s]
const int BEIDOU_B2ap_CODE_LENGTH_CHIPS = 10230;  //!< BEIDOU_B2a pilot code length [chips]
const double BEIDOU_B2ap_PERIOD = 0.001;          //!< BEIDOU_B2a pilot code period [seconds]

const int BEIDOU_B2a_HISTORY_DEEP = 5;

//Initialization registers for the primary codes for B2a data signal
const int32_t BEIDOU_B2ad_INIT_REG[63][13]=
{
		{1,	0,	0,	0,	0,	0,	0,	1,	0,	0,	1,	0,	1},
		{1,	0,	0,	0,	0,	0,	0,	1,	1,	0,	1,	0,	0},
		{1,	0,	0,	0,	0,	1,	0,	1,	0,	1,	1,	0,	1},
		{1,	0,	0,	0,	1,	0,	1,	0,	0,	1,	1,	1,	1},
		{1,	0,	0,	0,	1,	0,	1,	0,	1,	0,	1,	0,	1},
		{1,	0,	0,	0,	1,	1,	0,	1,	0,	1,	1,	1,	0},
		{1,	0,	0,	0,	1,	1,	1,	1,	0,	1,	1,	1,	0},
		{1,	0,	0,	0,	1,	1,	1,	1,	1,	1,	0,	1,	1},
		{1,	0,	0,	1,	1,	0,	0,	1,	0,	1,	0,	0,	1},
		{1,	0,	0,	1,	1,	1,	1,	0,	1,	1,	0,	1,	0},
		{1,	0,	1,	0,	0,	0,	0,	1,	1,	0,	1,	0,	1},
		{1,	0,	1,	0,	0,	0,	1,	0,	0,	0,	1,	0,	0},
		{1,	0,	1,	0,	0,	0,	1,	0,	1,	0,	1,	0,	1},
		{1,	0,	1,	0,	0,	0,	1,	0,	1,	1,	0,	1,	1},
		{1,	0,	1,	0,	0,	0,	1,	0,	1,	1,	1,	0,	0},
		{1,	0,	1,	0,	0,	1,	0,	1,	0,	0,	0,	1,	1},
		{1,	0,	1,	0,	0,	1,	1,	1,	1,	0,	1,	1,	1},
		{1,	0,	1,	0,	1,	0,	0,	0,	0,	0,	0,	0,	1},
		{1,	0,	1,	0,	1,	0,	0,	1,	1,	1,	1,	1,	0},
		{1,	0,	1,	0,	1,	1,	0,	1,	0,	1,	0,	1,	1},
		{1,	0,	1,	0,	1,	1,	0,	1,	1,	0,	0,	0,	1},
		{1,	0,	1,	1,	0,	0,	1,	0,	1,	0,	0,	1,	1},
		{1,	0,	1,	1,	0,	0,	1,	1,	0,	0,	0,	1,	0},
		{1,	0,	1,	1,	0,	1,	0,	0,	1,	1,	0,	0,	0},
		{1,	0,	1,	1,	0,	1,	0,	1,	1,	0,	1,	1,	0},
		{1,	0,	1,	1,	0,	1,	1,	1,	1,	0,	0,	1,	0},
		{1,	0,	1,	1,	0,	1,	1,	1,	1,	1,	1,	1,	1},
		{1,	0,	1,	1,	1,	0,	0,	0,	1,	0,	0,	1,	0},
		{1,	0,	1,	1,	1,	0,	0,	1,	1,	1,	1,	0,	0},
		{1,	0,	1,	1,	1,	1,	0,	1,	0,	0,	0,	0,	1},
		{1,	0,	1,	1,	1,	1,	1,	0,	0,	1,	0,	0,	0},
		{1,	0,	1,	1,	1,	1,	1,	0,	1,	0,	1,	0,	0},
		{1,	0,	1,	1,	1,	1,	1,	1,	0,	1,	0,	1,	1},
		{1,	0,	1,	1,	1,	1,	1,	1,	1,	0,	0,	1,	1},
		{1,	1,	0,	0,	0,	0,	1,	0,	1,	0,	0,	0,	1},
		{1,	1,	0,	0,	0,	1,	0,	0,	1,	0,	1,	0,	0},
		{1,	1,	0,	0,	0,	1,	0,	1,	1,	0,	1,	1,	1},
		{1,	1,	0,	0,	1,	0,	0,	0,	1,	0,	0,	0,	1},
		{1,	1,	0,	0,	1,	0,	0,	0,	1,	1,	0,	0,	1},
		{1,	1,	0,	0,	1,	1,	0,	1,	0,	1,	0,	1,	1},
		{1,	1,	0,	0,	1,	1,	0,	1,	1,	0,	0,	0,	1},
		{1,	1,	0,	0,	1,	1,	1,	0,	1,	0,	0,	1,	0},
		{1,	1,	0,	1,	0,	0,	1,	0,	1,	0,	1,	0,	1},
		{1,	1,	0,	1,	0,	0,	1,	1,	1,	0,	1,	0,	0},
		{1,	1,	0,	1,	0,	1,	1,	0,	0,	1,	0,	1,	1},
		{1,	1,	0,	1,	1,	0,	1,	0,	1,	0,	1,	1,	1},
		{1,	1,	1,	0,	0,	0,	0,	1,	1,	0,	1,	0,	0},
		{1,	1,	1,	0,	0,	1,	0,	0,	0,	0,	0,	1,	1},
		{1,	1,	1,	0,	0,	1,	0,	0,	0,	1,	0,	1,	1},
		{1,	1,	1,	0,	0,	1,	0,	1,	0,	0,	0,	1,	1},
		{1,	1,	1,	0,	0,	1,	0,	1,	0,	1,	0,	0,	0},
		{1,	1,	1,	0,	1,	0,	0,	1,	1,	1,	0,	1,	1},
		{1,	1,	1,	0,	1,	1,	0,	0,	1,	0,	1,	1,	1},
		{1,	1,	1,	1,	0,	0,	1,	0,	0,	1,	0,	0,	0},
		{1,	1,	1,	1,	0,	1,	0,	0,	1,	0,	1,	0,	0},
		{1,	1,	1,	1,	0,	1,	0,	0,	1,	1,	0,	0,	1},
		{1,	1,	1,	1,	0,	1,	1,	0,	1,	1,	0,	1,	0},
		{1,	1,	1,	1,	0,	1,	1,	1,	1,	1,	0,	0,	0},
		{1,	1,	1,	1,	0,	1,	1,	1,	1,	1,	1,	1,	1},
		{1,	1,	1,	1,	1,	1,	0,	1,	1,	0,	1,	0,	1},
		{0,	0,	1,	0,	0,	0,	0,	0,	0,	0,	0,	1,	0},
		{1,	1,	0,	1,	1,	1,	1,	1,	1,	0,	1,	0,	1},
		{0,	0,	0,	1,	1,	1,	1,	0,	1,	0,	0,	1,	0},
		};

const int32_t BEIDOU_B2ap_INIT_REG[63] =
    {
            5249,
            1409,
            5793,
            7761,
            5457,
            3761,
            3825,
            7153,
            4761,
            2937,
            5509,
            1093,
            5445,
            6981,
            1861,
            6309,
            7653,
            4117,
            3989,
            6837,
            4533,
            6477,
            2253,
             813,
            3501,
            2541,
            8173,
            2333,
            1949,
            4285,
             637,
            1405,
            6909,
            6653,
            4419,
            1315,
            7587,
            4371,
            4883,
            6835,
            4531,
            2419,
            5451,
            1483,
            6763,
            7515,
            1415,
            6183,
            6695,
            6311,
             679,
            7063,
            7479,
             591,
            1327,
            4911,
            2927,
            1007,
            8175,
            5567,
            3109,
            1012,
            5464};

//TODO, these are for DK to fill in.
const int BEIDOU_B2a_CNAV_DATA_PAGE_BITS = 300;  //!< GPS L5 CNAV page length, including preamble and CRC [bits]
const int BEIDOU_B2a_SYMBOLS_PER_BIT = 2;
const int BEIDOU_B2a_SAMPLES_PER_SYMBOL = 10;
const int BEIDOU_B2a_CNAV_DATA_PAGE_SYMBOLS = 600;
//const int BEIDOU_B2a_HISTORY_DEEP = 6;

//Beidou secondary codes. Data component has a fixed sequence as secondary code which are the same for every satellite
const int BEIDOU_B2ad_SECONDARY_CODE_LENGTH = 5;// Each bit is 1 ms (one primary code sequence)
const int BEIDOU_B2ad_SECONDARY_CODE[10] = {0, 0, 0, 1, 0,};
const std::string BEIDOU_B2ad_SECONDARY_CODE_STR = "00010";

//TODO Beidou pilot code is an actual code which is currently not implemented
//Beidou secondary codes. Pilot component has a truncated Weill sequence, each satellite has it's own code
const int BEIDOU_B2ap_SECONDARY_CODE_LENGTH = 20;//100; Each bit is 1 ms (one primary code sequence)
const int BEIDOU_B2ap_SECONDARY_CODE[20] = {0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 1, 0};
const std::string BEIDOU_B2ap_SECONDARY_CODE_STR = "00000100110101001110";

#endif /* GNSS_SDR_BEIDOU_B2a_H_ */