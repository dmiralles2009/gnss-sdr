/*!
 * \file BEIDOU_B2a_CA.h
 * \brief  Defines system parameters for Beidou B2a signal and CNAV2 data
 * \author Dong Kyeong Lee, 2018. dole7890(at)colorado.edu
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2015  (see AUTHORS file for a list of contributors)
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
 * along with GNSS-SDR. If not, see <http://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#ifndef GNSS_SDR_BEIDOU_B2a_CA_H_
#define GNSS_SDR_BEIDOU_B2a_CA_H_

#include "gnss_frequencies.h"
#include "MATH_CONSTANTS.h"
#include <map>
#include <vector>
#include <utility>  // std::pair


// Physical constants
const double BEIDOU_C_m_s = SPEED_OF_LIGHT;                   //!< The speed of light, [m/s]
const double BEIDOU_C_m_ms = 299792.4580;                     //!< The speed of light, [m/ms]
const double BEIDOU_PI = 3.1415926535898;                     //!< Pi as defined in IS-GPS-200E
const double BEIDOU_TWO_PI = 6.283185307179586;               //!< 2Pi as defined in IS-GPS-200E
const double BEIDOU_OMEGA_EARTH_DOT = 7.292115e-5;            //!< Earth rotation rate, [rad/s]
const double BEIDOU_GM = 398600.4418e9;                       //!< Geocentric gravitational constant, [m^3/s^2]
const double BEIDOU_fM_a = 0.35e9;                            //!< Gravitational constant of atmosphere [m^3/s^2]
const double BEIDOU_SEMI_MAJOR_AXIS = 6378137;                //!< Semi-major axis of Earth [m]
const double BEIDOU_FLATTENING = 1 / 298.257222101;                //!< Flattening parameter
const double BEIDOU_GRAVITY = 97803284;                       //!< Equatorial acceleration of gravity [mGal]
const double BEIDOU_GRAVITY_CORRECTION = 0.87;                //!< Correction to acceleration of gravity at sea-level due to Atmosphere[мGal]
const double BEIDOU_J2 = 1082625.75e-9;                       //!< Second zonal harmonic of the geopotential
const double BEIDOU_J4 = -2370.89e-9;                         //!< Fourth zonal harmonic of the geopotential
const double BEIDOU_J6 = 6.08e-9;                             //!< Sixth zonal harmonic of the geopotential
const double BEIDOU_J8 = 1.40e-11;                            //!< Eighth zonal harmonic of the geopotential
const double BEIDOU_U0 = 62636861.4;                          //!< Normal potential at surface of common terrestrial ellipsoid [m^2/s^2]
const double BEIDOU_C20 = -1082.63e-6;                        //!< Second zonal coefficient of spherical harmonic expansion
const double BEIDOU_EARTH_RADIUS = 6378.136;                  //!< Equatorial radius of Earth [km]
const double BEIDOU_EARTH_INCLINATION = 0.000409148809899e3;  //!< Mean inclination of ecliptic to equator (23 deg 26 min 33 sec) [rad]

const double BEIDOU_TAU_0 = -0.005835151531174e3;  //!< (-334 deg 19 min 46.40 sec) [rad];
const double BEIDOU_TAU_1 = 0.071018041257371e3;   //!< (4069 deg 02 min 02.52 sec) [rad];

const double BEIDOU_MOON_Q0 = -0.001115184961435e3;          //!< (-63 deg 53 min 43.41 sec) [rad]
const double BEIDOU_MOON_Q1 = 8.328691103668023e3;           //!< (477198 deg 50 min 56.79 sec) [rad]
const double BEIDOU_MOON_OMEGA_0 = 0.004523601514852e3;      //!< (259 deg 10 min 59.79 sec) [rad]
const double BEIDOU_MOON_OMEGA_1 = -0.033757146246552e3;     //!< (-1934 deg 08 min 31.23 sec) [rad]
const double BEIDOU_MOON_GM = 4902.835;                      //!< Lunar gravitational constant [km^3/s^2]
const double BEIDOU_MOON_SEMI_MAJOR_AXIS = 3.84385243e5;     //!< Semi-major axis of lunar orbit [km];
const double BEIDOU_MOON_ECCENTRICITY = 0.054900489;         //!< Eccentricity of lunar orbit
const double BEIDOU_MOON_INCLINATION = 0.000089803977407e3;  //!< Inclination of lunar orbit to ecliptic plane (5 deg 08 min 43.4 sec) [rad]

const double BEIDOU_SUN_OMEGA = 0.004908229466869e3;  //!< TODO What is this operation in the seconds with T?(281 deg 13 min 15.0 + 6189.03*Т sec) [rad]
const double BEIDOU_SUN_Q0 = 0.006256583774423e3;     //!< (358 deg 28 min 33.04 sec) [rad]
const double BEIDOU_SUN_Q1 = 0e3;                     //!< TODO Why is the value greater than 60?(129596579.10 sec) [rad]
const double BEIDOU_SUN_GM = 0.1325263e12;            //!< Solar gravitational constant [km^3/s^2]
const double BEIDOU_SUN_SEMI_MAJOR_AXIS = 1.49598e8;  //!< Semi-major axis of solar orbit [km];
const double BEIDOU_SUN_ECCENTRICITY = 0.016719;      //!< Eccentricity of solar orbit

const double BEIDOU_B2a_FREQ_HZ = FREQ2_BDS;        //!< BEIDOU B2a carrier frequency [Hz]
const double BEIDOU_B2a_CODE_RATE_HZ = 10.23e6;     //!< BEIDOU B2a code rate [chips/s]
const double BEIDOU_B2a_CODE_LENGTH_CHIPS = 10230;  //!< BEIDOU B2a C/A code length [chips]
const double BEIDOU_B2a_CODE_PERIOD = 0.001;        //!< BEIDOU B2a C/A code period [seconds]
const double BEIDOU_B2a_CHIP_PERIOD = 9.775171065493646e-08;   //!< BEIDOU B2a C/A chip period [seconds]
const double BEIDOU_B2a_SYMBOL_RATE_SPS = 200;		//BEIDOU symbol rate
//const double BEIDOU_B2a_CA_SYMBOL_Length = 600;

const int BEIDOU_NBR_SATS = 63;  // Total number of satellites

/*!
 * \brief Record of leap seconds definition for BDT to UTC conversion and vice versa
 */
const double BEIDOU_LEAP_SECONDS = -33; // uniform scale and 33 seconds behind TAI. However, this should be in the broadcast message

//const double BEIDOU_STARTOFFSET_ms = 68.802;  //[ms] Initial sign. travel time (this cannot go here)

// OBSERVABLE HISTORY DEEP FOR INTERPOLATION
//const int BEIDOU_L1_CA_HISTORY_DEEP = 100;

// NAVIGATION MESSAGE DEMODULATION AND DECODING
#define BEIDOU_CNAV2_PREAMBLE                                                                    \
    {                                                                                            \
        1, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0 \
    }
const double BEIDOU_CNAV2_PREAMBLE_DURATION_S = 0.120;	//[s]
const int BEIDOU_CNAV2_PREAMBLE_LENGTH_BITS = 12;		//[bits]
const int BEIDOU_CNAV2_PREAMBLE_LENGTH_SYMBOLS = 24;	//[symbols]

//const int BEIDOU_CNAV2_PREAMBLE_PERIOD_SYMBOLS = 2000;
const int BEIDOU_CNAV2_TELEMETRY_RATE_BITS_SECOND = 100;																					//bps
const int BEIDOU_CNAV2_TELEMETRY_SYMBOLS_PER_BIT = 2;																						//spb
const int BEIDOU_CNAV2_TELEMETRY_SYMBOLS_PER_PREAMBLE_BIT = 2;																				//spb
const int BEIDOU_CNAV2_TELEMETRY_RATE_SYMBOLS_SECOND = BEIDOU_CNAV2_TELEMETRY_RATE_BITS_SECOND * BEIDOU_CNAV2_TELEMETRY_SYMBOLS_PER_BIT;	//sps
const int BEIDOU_CNAV2_STRING_SYMBOLS = 600;																								//Number of symbols per string in the CNAV2 message
const int BEIDOU_CNAV2_STRING_BITS = 300;																									//Number of bits per string in the CNAV2 message
const int BEIDOU_CNAV2_DATA_SYMBOLS = 576;																									//STRING DATA WITHOUT PREAMBLE

// BEIDOU CNAV2 NAVIGATION MESSAGE STRUCTURE
// NAVIGATION MESSAGE FIELDS POSITIONS

// Types

// Common
const std::vector<std::pair<int, int>> PRN({{1, 6}});
const std::vector<std::pair<int, int>> MesType({{7, 6}});
const std::vector<std::pair<int, int>> SOW({{13, 18}});
const std::vector<std::pair<int, int>> CRC({{265, 24}});

// Type 10 (288 bits)
const std::vector<std::pair<int, int>> WN_10({{31, 13}});
const std::vector<std::pair<int, int>> DIF_10({{44, 1}});
const std::vector<std::pair<int, int>> SIF_10({{45, 1}});
const std::vector<std::pair<int, int>> AIF_10({{46, 1}});
const std::vector<std::pair<int, int>> SISMAI_10({{47, 4}});
const std::vector<std::pair<int, int>> DIF_B1C_10({{51, 1}});
const std::vector<std::pair<int, int>> SIF_B1C_10({{52, 1}});
const std::vector<std::pair<int, int>> AIF_B1C_10({{53, 1}});
const std::vector<std::pair<int, int>> IODE_10({{54, 8}});
// Ephemeris I (203 bits)
const std::vector<std::pair<int, int>> t_oe_10({{62, 11}});
const std::vector<std::pair<int, int>> SatType_10({{73, 2}});
const std::vector<std::pair<int, int>> dA_10({{75, 26}});
const std::vector<std::pair<int, int>> A_dot_10({{101, 25}});
const std::vector<std::pair<int, int>> dn_0_10({{126, 17}});
const std::vector<std::pair<int, int>> dn_0_dot_10({{143, 23}});
const std::vector<std::pair<int, int>> M_0_10({{166, 33}});
const std::vector<std::pair<int, int>> e_10({{199, 33}});
const std::vector<std::pair<int, int>> omega_10({{232, 33}});
// Ephemeris I End

// Type 11 (288 bits)
const std::vector<std::pair<int, int>> HS_11({{31, 2}});
const std::vector<std::pair<int, int>> DIF_11({{33, 1}});
const std::vector<std::pair<int, int>> SIF_11({{34, 1}});
const std::vector<std::pair<int, int>> AIF_11({{35, 1}});
const std::vector<std::pair<int, int>> SISMAI_11({{36, 4}});
const std::vector<std::pair<int, int>> DIF_B1C_11({{40, 1}});
const std::vector<std::pair<int, int>> SIF_B1C_11({{41, 1}});
const std::vector<std::pair<int, int>> AIF_B1C_11({{42, 1}});
// Ephemeris II (222 bits)
const std::vector<std::pair<int, int>> Omega_0_11({{43, 33}});
const std::vector<std::pair<int, int>> i_0_11({{76, 33}});
const std::vector<std::pair<int, int>> Omega_dot_11({{109, 19}});
const std::vector<std::pair<int, int>> i_0_dot_11({{128, 15}});
const std::vector<std::pair<int, int>> C_IS_11({{143, 16}});
const std::vector<std::pair<int, int>> C_IC_11({{159, 16}});
const std::vector<std::pair<int, int>> C_RS_11({{175, 24}});
const std::vector<std::pair<int, int>> C_RC_11({{199, 24}});
const std::vector<std::pair<int, int>> C_US_11({{223, 21}});
const std::vector<std::pair<int, int>> C_UC_11({{244, 21}});
// Ephemeris II End


// Type 30 (288 bits)
const std::vector<std::pair<int, int>> HS_30({{31, 2}});
const std::vector<std::pair<int, int>> DIF_30({{33, 1}});
const std::vector<std::pair<int, int>> SIF_30({{34, 1}});
const std::vector<std::pair<int, int>> AIF_30({{35, 1}});
const std::vector<std::pair<int, int>> SISMAI_30({{36, 4}});
const std::vector<std::pair<int, int>> DIF_B1C_30({{40, 1}});
const std::vector<std::pair<int, int>> SIF_B1C_30({{41, 1}});
const std::vector<std::pair<int, int>> AIF_B1C_30({{42, 1}});
// Clock Correction Parameters (69 bits)
const std::vector<std::pair<int, int>> t_oc_30({{43, 11}});
const std::vector<std::pair<int, int>> a_0_30({{54, 25}});
const std::vector<std::pair<int, int>> a_1_30({{79, 22}});
const std::vector<std::pair<int, int>> a_2_30({{101, 11}});
// Clock Correction Parameters End
const std::vector<std::pair<int, int>> IODC_30({{112, 10}});
const std::vector<std::pair<int, int>> T_GDB2ap_30({{122, 12}});
const std::vector<std::pair<int, int>> ISC_B2ad_30({{134, 12}});
// Ionospheric Delay Correction Model Parameters (74 bits)
const std::vector<std::pair<int, int>> alpha_1_30({{146, 10}});
const std::vector<std::pair<int, int>> alpha_2_30({{156, 8}});
const std::vector<std::pair<int, int>> alpha_3_30({{164, 8}});
const std::vector<std::pair<int, int>> alpha_4_30({{172, 8}});
const std::vector<std::pair<int, int>> alpha_5_30({{180, 8}});
const std::vector<std::pair<int, int>> alpha_6_30({{188, 8}});
const std::vector<std::pair<int, int>> alpha_7_30({{196, 8}});
const std::vector<std::pair<int, int>> alpha_8_30({{204, 8}});
const std::vector<std::pair<int, int>> alpha_9_30({{212, 8}});
// Ionospheric Delay Correction Model Parameters End
const std::vector<std::pair<int, int>> T_GDB1Cp_30({{220, 12}});
const std::vector<std::pair<int, int>> Rev_30({{232, 33}});


// Type 31 (288 bits)
const std::vector<std::pair<int, int>> HS_31({{31, 2}});
const std::vector<std::pair<int, int>> DIF_31({{33, 1}});
const std::vector<std::pair<int, int>> SIF_31({{34, 1}});
const std::vector<std::pair<int, int>> AIF_31({{35, 1}});
const std::vector<std::pair<int, int>> SISMAI_31({{36, 4}});
const std::vector<std::pair<int, int>> DIF_B1C_31({{40, 1}});
const std::vector<std::pair<int, int>> SIF_B1C_31({{41, 1}});
const std::vector<std::pair<int, int>> AIF_B1C_31({{42, 1}});
// Clock Correction Parameters (69 bits)
const std::vector<std::pair<int, int>> t_oc_31({{43, 11}});
const std::vector<std::pair<int, int>> a_0_31({{54, 25}});
const std::vector<std::pair<int, int>> a_1_31({{79, 22}});
const std::vector<std::pair<int, int>> a_2_31({{101, 11}});
// Clock Correction Parameters End
const std::vector<std::pair<int, int>> IODC_31({{112, 10}});
const std::vector<std::pair<int, int>> WN_a_31({{122, 13}});
const std::vector<std::pair<int, int>> t_oa_31({{135, 8}});
// Reduced Almanac Parameters Sat 1(38 bits)
const std::vector<std::pair<int, int>> PRN_a1_31({{143, 6}});
const std::vector<std::pair<int, int>> SatType1_31({{149, 2}});
const std::vector<std::pair<int, int>> delta_A1_31({{151, 8}});
const std::vector<std::pair<int, int>> Omega_01_31({{159, 7}});
const std::vector<std::pair<int, int>> Phi_01_31({{166, 7}});
const std::vector<std::pair<int, int>> Health1_31({{173, 8}});
// Reduced Almanac Parameters End
// Reduced Almanac Parameters Sat 2(38 bits)
const std::vector<std::pair<int, int>> PRN_a2_31({{181, 6}});
const std::vector<std::pair<int, int>> SatType2_31({{187, 2}});
const std::vector<std::pair<int, int>> delta_A2_31({{189, 8}});
const std::vector<std::pair<int, int>> Omega_02_31({{197, 7}});
const std::vector<std::pair<int, int>> Phi_02_31({{204, 7}});
const std::vector<std::pair<int, int>> Health2_31({{211, 8}});
// Reduced Almanac Parameters End
// Reduced Almanac Parameters Sat 3(38 bits)
const std::vector<std::pair<int, int>> PRN_a3_31({{219, 6}});
const std::vector<std::pair<int, int>> SatType3_31({{225, 2}});
const std::vector<std::pair<int, int>> delta_A3_31({{227, 8}});
const std::vector<std::pair<int, int>> Omega_03_31({{235, 7}});
const std::vector<std::pair<int, int>> Phi_03_31({{242, 7}});
const std::vector<std::pair<int, int>> Health3_31({{249, 8}});
// Reduced Almanac Parameters End
const std::vector<std::pair<int, int>> Rev_31({{257, 8}});


// Type 32 (288 bits)
const std::vector<std::pair<int, int>> HS_32({{31, 2}});
const std::vector<std::pair<int, int>> DIF_32({{33, 1}});
const std::vector<std::pair<int, int>> SIF_32({{34, 1}});
const std::vector<std::pair<int, int>> AIF_32({{35, 1}});
const std::vector<std::pair<int, int>> SISMAI_32({{36, 4}});
const std::vector<std::pair<int, int>> DIF_B1C_32({{40, 1}});
const std::vector<std::pair<int, int>> SIF_B1C_32({{41, 1}});
const std::vector<std::pair<int, int>> AIF_B1C_32({{42, 1}});
// Clock Correction Parameters (69 bits)
const std::vector<std::pair<int, int>> t_oc_32({{43, 11}});
const std::vector<std::pair<int, int>> a_0_32({{54, 25}});
const std::vector<std::pair<int, int>> a_1_32({{79, 22}});
const std::vector<std::pair<int, int>> a_2_32({{101, 11}});
// Clock Correction Parameters End
const std::vector<std::pair<int, int>> IODC_32({{112, 10}});
// EOP Parameters (138 bits)
const std::vector<std::pair<int, int>> t_EOP_32({{122, 16}});
const std::vector<std::pair<int, int>> PM_X_32({{138, 21}});
const std::vector<std::pair<int, int>> PM_X_dot_32({{159, 15}});
const std::vector<std::pair<int, int>> PM_Y_32({{174, 21}});
const std::vector<std::pair<int, int>> PM_Y_dot_32({{195, 15}});
const std::vector<std::pair<int, int>> dUT1_32({{210, 31}});
const std::vector<std::pair<int, int>> dUT1_dot_32({{241, 19}});
// EOP Parameters End
const std::vector<std::pair<int, int>> Rev_32({{260, 5}});


// Type 33 (288 bits)
const std::vector<std::pair<int, int>> HS_33({{31, 2}});
const std::vector<std::pair<int, int>> DIF_33({{33, 1}});
const std::vector<std::pair<int, int>> SIF_33({{34, 1}});
const std::vector<std::pair<int, int>> AIF_33({{35, 1}});
const std::vector<std::pair<int, int>> SISMAI_33({{36, 4}});
const std::vector<std::pair<int, int>> DIF_B1C_33({{40, 1}});
const std::vector<std::pair<int, int>> SIF_B1C_33({{41, 1}});
const std::vector<std::pair<int, int>> AIF_B1C_33({{42, 1}});
// Clock Correction Parameters (69 bits)
const std::vector<std::pair<int, int>> t_oc_33({{43, 11}});
const std::vector<std::pair<int, int>> a_0_33({{54, 25}});
const std::vector<std::pair<int, int>> a_1_33({{79, 22}});
const std::vector<std::pair<int, int>> a_2_33({{101, 11}});
// Clock Correction Parameters End
// BGTO Parameters (68 bits)
const std::vector<std::pair<int, int>> GNSS_ID_33({{112, 3}});
const std::vector<std::pair<int, int>> WN_0BGTO_33({{115, 13}});
const std::vector<std::pair<int, int>> t_0BGTO_33({{128, 16}});
const std::vector<std::pair<int, int>> A_0BGTO_33({{144, 16}});
const std::vector<std::pair<int, int>> A_1BGTO_33({{160, 13}});
const std::vector<std::pair<int, int>> A_2BGTO_33({{173, 7}});
// BGTO Parameters End
// Reduced Almanac Parameters (38 bits)
const std::vector<std::pair<int, int>> PRN_a_33({{180, 6}});
const std::vector<std::pair<int, int>> SatType_33({{186, 2}});
const std::vector<std::pair<int, int>> delta_A_33({{188, 8}});
const std::vector<std::pair<int, int>> Omega_0_33({{196, 7}});
const std::vector<std::pair<int, int>> Phi_0_33({{203, 7}});
const std::vector<std::pair<int, int>> Health_33({{210, 8}});
// Reduced Almanac Parameters End
const std::vector<std::pair<int, int>> IODC_33({{218, 10}});
const std::vector<std::pair<int, int>> WN_a_33({{228, 13}});
const std::vector<std::pair<int, int>> t_oa_33({{241, 8}});
const std::vector<std::pair<int, int>> Rev_33({{249, 16}});


// Type 34 (288 bits)
const std::vector<std::pair<int, int>> HS_34({{31, 2}});
const std::vector<std::pair<int, int>> DIF_34({{33, 1}});
const std::vector<std::pair<int, int>> SIF_34({{34, 1}});
const std::vector<std::pair<int, int>> AIF_34({{35, 1}});
const std::vector<std::pair<int, int>> SISMAI_34({{36, 4}});
const std::vector<std::pair<int, int>> DIF_B1C_34({{40, 1}});
const std::vector<std::pair<int, int>> SIF_B1C_34({{41, 1}});
const std::vector<std::pair<int, int>> AIF_B1C_34({{42, 1}});
// SISAI_OC (22 bits)
const std::vector<std::pair<int, int>> t_op_34({{43, 11}});
const std::vector<std::pair<int, int>> SISAI_ocb_34({{54, 5}});
const std::vector<std::pair<int, int>> SISAI_oc1_34({{59, 3}});
const std::vector<std::pair<int, int>> SISAI_oc2_34({{62, 3}});
// SISAI_OC End
// Clock Correction Parameters (69 bits)
const std::vector<std::pair<int, int>> t_oc_34({{65, 11}});
const std::vector<std::pair<int, int>> a_0_34({{76, 25}});
const std::vector<std::pair<int, int>> a_1_34({{101, 22}});
const std::vector<std::pair<int, int>> a_2_34({{123, 11}});
// Clock Correction Parameters End
const std::vector<std::pair<int, int>> IODC_34({{134, 10}});
// BDT-UTC Time Offset Parameters (97 bits)
const std::vector<std::pair<int, int>> A_0UTC_34({{144, 16}});
const std::vector<std::pair<int, int>> A_1UTC_34({{160, 13}});
const std::vector<std::pair<int, int>> A_2UTC_34({{173, 7}});
const std::vector<std::pair<int, int>> dt_LS_34({{180, 8}});
const std::vector<std::pair<int, int>> t_ot_34({{188, 16}});
const std::vector<std::pair<int, int>> WN_ot_34({{204, 13}});
const std::vector<std::pair<int, int>> WN_LSF_34({{217, 13}});
const std::vector<std::pair<int, int>> DN_34({{230, 3}});
const std::vector<std::pair<int, int>> dt_LSF_34({{233, 8}});
// BDT-UTC Time Offset Parameters End
const std::vector<std::pair<int, int>> Rev_34({{241, 24}});


// Type 40 (288 bits)
const std::vector<std::pair<int, int>> HS_40({{31, 2}});
const std::vector<std::pair<int, int>> DIF_40({{33, 1}});
const std::vector<std::pair<int, int>> SIF_40({{34, 1}});
const std::vector<std::pair<int, int>> AIF_40({{35, 1}});
const std::vector<std::pair<int, int>> SISMAI_40({{36, 4}});
const std::vector<std::pair<int, int>> DIF_B1C_40({{40, 1}});
const std::vector<std::pair<int, int>> SIF_B1C_40({{41, 1}});
const std::vector<std::pair<int, int>> AIF_B1C_40({{42, 1}});
const std::vector<std::pair<int, int>> SISAI_OE_40({{43, 5}});
// SISAI_OC (22 bits)
const std::vector<std::pair<int, int>> t_op_40({{48, 11}});
const std::vector<std::pair<int, int>> SISAI_ocb_40({{59, 5}});
const std::vector<std::pair<int, int>> SISAI_oc1_40({{64, 3}});
const std::vector<std::pair<int, int>> SISAI_oc2_40({{67, 3}});
// SISAI_OC End
// Midi Almanac Parameters (156 bits)
const std::vector<std::pair<int, int>> PRN_a_40({{70, 6}});
const std::vector<std::pair<int, int>> SatType_40({{76, 2}});
const std::vector<std::pair<int, int>> WN_a_40({{78, 13}});
const std::vector<std::pair<int, int>> t_oa_40({{91, 8}});
const std::vector<std::pair<int, int>> e_40({{99, 11}});
const std::vector<std::pair<int, int>> delta_i_40({{110, 11}});
const std::vector<std::pair<int, int>> sqrt_A_40({{121, 17}});
const std::vector<std::pair<int, int>> Omega_0_40({{138, 16}});
const std::vector<std::pair<int, int>> Omega_dot_40({{154, 11}});
const std::vector<std::pair<int, int>> omega_40({{165, 16}});
const std::vector<std::pair<int, int>> M_0_40({{181, 16}});
const std::vector<std::pair<int, int>> a_f0_40({{197, 11}});
const std::vector<std::pair<int, int>> a_f1_40({{208, 10}});
const std::vector<std::pair<int, int>> Health_40({{218, 8}});
// Midi Almanac Parameters End
const std::vector<std::pair<int, int>> Rev_40({{226, 39}});


#endif /* GNSS_SDR_BEIDOU_B2a_CA_H_ */
