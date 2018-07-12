/*
 * \file beidou_cnav2_utc_model.cc
 * \brief  Interface of a BEIDOU CNAV2 UTC MODEL storage
 * \note Code added as part of GSoC 2018 program
 * \author Dong Kyeong Lee, 2018. dole7890(at)colorado.edu
 * \see <a href="http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf">BEIDOU ICD</a>
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

#include "beidou_cnav2_utc_model.h"

Beidou_Cnav2_Utc_Model::Beidou_Cnav2_Utc_Model()
{
    valid = false;

    // BDT-UTC Time Offset Parameters
    A_0UTC = 0.0;		//Bias coefficient of BDT time scale relative to UTC time scale [s]
	A_1UTC = 0.0;		//Drift coefficient of BDT time scale relative to UTC time scale [s/s]
	A_2UTC = 0.0;		//Drift rate coefficient of BDT time scale relative to UTC time scale [s/s^2]
	dt_LS = 0.0;		//Current of past leap second count [s]
	t_ot = 0.0;			//Reference time of week [s]
	WN_ot = 0.0;		//Reference week number [week]
	WN_LSF = 0.0;		//Leap second reference week number [week]
	DN = 0.0;			//Leap second reference day number [day]
	dt_LSF = 0.0;		//Current of future leap second count [s]

	// BDT-GNSS Time Offset Parameters
	GNSS_ID = 0.0;		//GNSS type identification [dimensionless]
	WN_0BGTO = 0.0;		//Reference week number [week]
	t_0BGTO = 0.0;		//Reference time of week [s]
	A_0BGTO = 0.0;		//Bias coefficient of BDT time scale relative to GNSS time scale [s]
	A_1BGTO = 0.0;		//Drift coefficient of BDT time scale relative to GNSS time scale [s/s]
	A_2BGTO = 0.0;		//Drift rate coefficient of BDT time scale relative to GNSS time scale [s/s^2]

	// Clock Correction Parameters
	t_oc = 0.0;
	a_0 = 0.0;
	a_1 = 0.0;
	a_2 = 0.0;
}


double Beidou_Cnav2_Utc_Model::utc_time(double beidou_time_corrected)
{
    double t_utc;

    // BEIDOU Time is relative to UTC Moscow, so we simply add its time difference
    t_utc = beidou_time_corrected + 3 * 3600 + d_tau_c;

    return t_utc;
}
