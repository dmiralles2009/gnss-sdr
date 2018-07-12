/*!
 * \file beidou_cnav2_navigation_message.cc
 * \brief  Implementation of a beidou cnav2 Data message decoder as described in BEIDOU ICD
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

#include "beidou_cnav2_navigation_message.h"
#include "gnss_satellite.h"
#include <glog/logging.h>


void Beidou_Cnav2_Navigation_Message::reset()
{
    // Satellite Identification
    i_satellite_PRN = 0;
    i_alm_satellite_slot_number = 0;  //!< SV Orbit Slot Number
    flag_update_slot_number = false;

    // Ephemeris Flags
    flag_all_ephemeris = false;
    flag_ephemeris_str_1 = false;
    flag_ephemeris_str_2 = false;

    // Almanac Flags
    flag_almanac_str_31 = false;

    // UTC and System Clocks Flags
    flag_utc_model_valid = false;   //!< If set, it indicates that the UTC model parameters are filled
    flag_utc_model_str_5 = false;   //!< Clock info send in string 5 of navigation data
    flag_utc_model_str_15 = false;  //!< Clock info send in string 15 of frame 5 of navigation data

    // broadcast orbit 1
    flag_TOW_set = false;
    flag_TOW_new = false;

    flag_CRC_test = false;
    d_frame_ID = 0;
    d_string_ID = 0;
    i_channel_ID = 0;

    // Clock terms
    d_satClkCorr = 0.0;
    d_dtr = 0.0;
    d_satClkDrift = 0.0;

    // Data update information
    d_previous_tb = 0.0;
    for (unsigned int i = 0; i < BEIDOU_CA_NBR_SATS; i++)
        d_previous_Na[i] = 0.0;

    std::map<int, std::string> satelliteBlock;  //!< Map that stores to which block the PRN belongs http://www.navcen.uscg.gov/?Do=constellationStatus

    auto gnss_sat = Gnss_Satellite();
    std::string _system("BEIDOU");
    //TODO SHould number of channels be hardcoded?
    for (unsigned int i = 1; i < 14; i++)
        {
            satelliteBlock[i] = gnss_sat.what_block(_system, i);
        }
}


Beidou_Cnav2_Navigation_Message::Beidou_Cnav2_Navigation_Message()
{
    reset();
}


bool Beidou_Cnav2_Navigation_Message::CRC_test(std::bitset<BEIDOU_CNAV2_STRING_BITS> bits, d_string_ID)
{
    int sum_bits = 0;
    std::vector<int> string_bits(BEIDOU_CNAV2_STRING_BITS);

    //!< Populate data
    for (int i = 0; i < static_cast<int>(BEIDOU_CNAV2_STRING_BITS); i++)
        {
            string_bits[i] = static_cast<int>(bits[i]);
        }

    // This is very inefficient. Need a better method
    if (d_string_ID==10)
    {
    	for(int i=0; i<13; i++)
    		{
    		temp = 1;
    			for (int j=0; j<i; j++)
    			{
    				temp = temp*2;
    			}
    			sum_bits += static_cast<int>(string_bits[31+i])*temp;
    		}
    	for(int i=0; i<1; i++)
			{
			temp = 1;
				for (int j=0; j<i; j++)
				{
					temp = temp*2;
				}
				sum_bits += static_cast<int>(string_bits[44+i])*temp;
			}

    }
    //

    if (sum_bits == static_cast<double>(read_navigation_unsigned(string_bits, CRC)))
        {
            return true;
        }
    else
        // All other conditions are assumed errors.
        {
            return false;
        }
}


bool Beidou_Cnav2_Navigation_Message::read_navigation_bool(std::bitset<BEIDOU_CNAV2_STRING_BITS> bits, const std::vector<std::pair<int, int>> parameter)
{
    bool value;

    if (bits[BEIDOU_CNAV2_STRING_BITS - parameter[0].first] == 1)
        {
            value = true;
        }
    else
        {
            value = false;
        }
    return value;
}


unsigned long int Beidou_Cnav2_Navigation_Message::read_navigation_unsigned(std::bitset<BEIDOU_CNAV2_STRING_BITS> bits, const std::vector<std::pair<int, int>> parameter)
{
    unsigned long int value = 0;
    int num_of_slices = parameter.size();
    for (int i = 0; i < num_of_slices; i++)
        {
            for (int j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1;  //shift left
                    if (bits[BEIDOU_CNAV2_STRING_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1;  // insert the bit
                        }
                }
        }
    return value;
}


signed long int Beidou_Cnav2_Navigation_Message::read_navigation_signed(std::bitset<BEIDOU_CNAV2_STRING_BITS> bits, const std::vector<std::pair<int, int>> parameter)
{
    signed long int value = 0;
    signed long int sign = 0;
    int num_of_slices = parameter.size();
    // read the MSB and perform the sign extension
    if (bits[BEIDOU_CNAV2_STRING_BITS - parameter[0].first] == 1)
        {
            sign = -1;
        }
    else
        {
            sign = 1;
        }
    for (int i = 0; i < num_of_slices; i++)
        {
            for (int j = 1; j < parameter[i].second; j++)
                {
                    value <<= 1;  //shift left
                    if (bits[BEIDOU_CNAV2_STRING_BITS - parameter[i].first - j] == 1)
                        {
                            value += 1;  // insert the bit
                        }
                }
        }
    return (sign * value);
}

int Beidou_Cnav2_Navigation_Message::string_decoder(std::string frame_string)
{
    int J = 0;
    d_string_ID = 0;
    d_frame_ID = 0;

    // Unpack bytes to bits
    std::bitset<BEIDOU_CNAV2_STRING_BITS> string_bits(frame_string);


    d_string_ID = static_cast<unsigned int>(read_navigation_unsigned(string_bits, MesType));

    // Perform data verification and exit code if error in bit sequence
	flag_CRC_test = CRC_test(string_bits,d_string_ID);


	if (flag_CRC_test == false)
		return 0;

	// Decode all 8 string messages
	switch (d_string_ID)
        {
        case 10:
            //--- It is Type 10 -----------------------------------------------

        	cnav2_ephemeris.PRN = static_cast<unsigned int>(read_navigation_unsigned(string_bits, PRN));
        	//cnav2_ephemeris.MesType = static_cast<double>(read_navigation_unsigned(string_bits, MesType));
        	cnav2_ephemeris.SOW = static_cast<double>(read_navigation_unsigned(string_bits, SOW))*3;	//[s] effective range 0~604797
        	cnav2_ephemeris.WN = static_cast<double>(read_navigation_unsigned(string_bits, WN_10));		//[week] effective range 0~8191
        	cnav2_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(string_bits, DIF_10));
        	cnav2_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(string_bits, SIF_10));
        	cnav2_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(string_bits, AIF_10));
        	cnav2_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(string_bits, SISMAI_10));
        	cnav2_ephemeris.DIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, DIF_B1C_10));
        	cnav2_ephemeris.SIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, SIF_B1C_10));
        	cnav2_ephemeris.AIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, AIF_B1C_10));
        	cnav2_ephemeris.IODE = static_cast<double>(read_navigation_unsigned(string_bits, IODE_10));

			// Ephemeris I Start
        	cnav2_ephemeris.t_oe = static_cast<double>(read_navigation_unsigned(string_bits, t_oe_10))*300;			//[s] effective range 0~604500
        	cnav2_ephemeris.SatType = static_cast<double>(read_navigation_unsigned(string_bits, SatType_10));			//[dimensionless] binary, 01:GEO, 10:IGSO, 11:MEO, 00:Reserved
        	cnav2_ephemeris.dA = static_cast<double>(read_navigation_signed(string_bits, dA_10))*TWO_N9;				//[m] reference MEO: 27906100m, IGSO/GEO: 42162200m
        	cnav2_ephemeris.A_dot = static_cast<double>(read_navigation_signed(string_bits, A_dot_10))*TWO_N21;		//[m/s]
        	cnav2_ephemeris.dn_0 = static_cast<double>(read_navigation_signed(string_bits, dn_0_10))*TWO_N44;			//[pi/s]
        	cnav2_ephemeris.dn_0_dot = static_cast<double>(read_navigation_signed(string_bits, dn_0_dot_10))*TWO_N57;	//[pi/s^2]
        	cnav2_ephemeris.M_0 = static_cast<double>(read_navigation_signed(string_bits, M_0_10))*TWO_N32;			//[pi]
        	cnav2_ephemeris.e = static_cast<double>(read_navigation_unsigned(string_bits, e_10))*TWO_N34;				//[dimensionless]
        	cnav2_ephemeris.omega = static_cast<double>(read_navigation_signed(string_bits, omega_10))*TWO_N32;		//[pi]
        	// Ephemeris I End

            flag_ephemeris_str_1 = true;

            break;

        case 11:
            //--- It is Type 11 -----------------------------------------------
            if (flag_ephemeris_str_1 == true)
                {

            	cnav2_ephemeris.PRN = static_cast<unsigned int>(read_navigation_unsigned(string_bits, PRN));
            	//cnav2_ephemeris.MesType = static_cast<double>(read_navigation_unsigned(string_bits, MesType));
            	cnav2_ephemeris.SOW = static_cast<double>(read_navigation_unsigned(string_bits, SOW))*3;		//[s] effective range 0~604797
            	cnav2_ephemeris.HS = static_cast<double>(read_navigation_unsigned(string_bits, HS_11));
            	cnav2_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(string_bits, DIF_11));
				cnav2_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(string_bits, SIF_11));
				cnav2_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(string_bits, AIF_11));
				cnav2_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(string_bits, SISMAI_11));
				cnav2_ephemeris.DIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, DIF_B1C_11));
				cnav2_ephemeris.SIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, SIF_B1C_11));
				cnav2_ephemeris.AIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, AIF_B1C_11));

            	// Ephemeris II Start
				cnav2_ephemeris.Omega_0 = static_cast<double>(read_navigation_signed(string_bits, Omega_0_11))*TWO_N32;		//[pi]
				cnav2_ephemeris.i_0 = static_cast<double>(read_navigation_signed(string_bits, i_0_11))*TWO_N32;				//[pi]
				cnav2_ephemeris.Omega_dot = static_cast<double>(read_navigation_signed(string_bits, Omega_dot_11))*TWO_N44;	//[pi/s]
				cnav2_ephemeris.i_0_dot = static_cast<double>(read_navigation_signed(string_bits, i_0_dot_11))*TWO_N44;		//[pi/s]
				cnav2_ephemeris.C_IS = static_cast<double>(read_navigation_signed(string_bits, C_IS_11))*TWO_N30;				//[rad]
				cnav2_ephemeris.C_IC = static_cast<double>(read_navigation_signed(string_bits, C_IC_11))*TWO_N30;				//[rad]
				cnav2_ephemeris.C_RS = static_cast<double>(read_navigation_signed(string_bits, C_RS_11))*TWO_N8;				//[m]
				cnav2_ephemeris.C_RC = static_cast<double>(read_navigation_signed(string_bits, C_RC_11))*TWO_N8;				//[m]
				cnav2_ephemeris.C_US = static_cast<double>(read_navigation_signed(string_bits, C_US_11))*TWO_N30;				//[rad]
				cnav2_ephemeris.C_UC = static_cast<double>(read_navigation_signed(string_bits, C_UC_11))*TWO_N30;				//[rad]
				// Ephemeris II End

				cnav2_ephemeris.CRC = static_cast<double>(read_navigation_unsigned(string_bits, CRC_11));

                flag_ephemeris_str_2 = true;
                }

            break;

        case 30:
            // --- It is Type 30 ----------------------------------------------

        	cnav2_ephemeris.PRN = static_cast<unsigned int>(read_navigation_unsigned(string_bits, PRN));
        	//cnav2_ephemeris.MesType = static_cast<double>(read_navigation_unsigned(string_bits, MesType));
        	cnav2_ephemeris.SOW = static_cast<double>(read_navigation_unsigned(string_bits, SOW))*3;	//[s] effective range 0~604797
        	cnav2_ephemeris.HS = static_cast<double>(read_navigation_unsigned(string_bits, HS_30));
        	cnav2_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(string_bits, DIF_30));
        	cnav2_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(string_bits, SIF_30));
        	cnav2_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(string_bits, AIF_30));
        	cnav2_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(string_bits, SISMAI_30));
        	cnav2_ephemeris.DIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, DIF_B1C_30));
        	cnav2_ephemeris.SIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, SIF_B1C_30));
        	cnav2_ephemeris.AIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, AIF_B1C_30));

        	// Clock Correction Parameters (69 bits)
        	cnav2_utc_model.t_oc = static_cast<double>(read_navigation_unsigned(string_bits, t_oc_30))*300;				//[s]
        	cnav2_utc_model.a_0 = static_cast<double>(read_navigation_signed(string_bits, a_0_30))*TWO_N34;				//[s]
        	cnav2_utc_model.a_1 = static_cast<double>(read_navigation_signed(string_bits, a_1_30))*TWO_N50;				//[s/s]
        	cnav2_utc_model.a_2 = static_cast<double>(read_navigation_signed(string_bits, a_2_30))*TWO_N66;				//[s/s^2]
        	// Clock Correction Parameters End

        	cnav2_ephemeris.IODC = static_cast<double>(read_navigation_unsigned(string_bits, IODC_30));
        	cnav2_ephemeris.T_GDB2ap = static_cast<double>(read_navigation_signed(string_bits, T_GDB2ap_30))*TWO_N34;		//[s]
        	cnav2_ephemeris.ISC_B2ad = static_cast<double>(read_navigation_unsigned(string_bits, ISC_B2ad_30))*TWO_N34;	//[s]

        	// Ionospheric Delay Correction Model Parameters (74 bits)
        	cnav2_ephemeris.alpha_1 = static_cast<double>(read_navigation_unsigned(string_bits, alpha_1_30))*TWO_N3;		//[TECu]
        	cnav2_ephemeris.alpha_2 = static_cast<double>(read_navigation_signed(string_bits, alpha_2_30))*TWO_N3;			//[TECu]
        	cnav2_ephemeris.alpha_3 = static_cast<double>(read_navigation_unsigned(string_bits, alpha_3_30))*TWO_N3;		//[TECu]
        	cnav2_ephemeris.alpha_4 = static_cast<double>(read_navigation_unsigned(string_bits, alpha_4_30))*TWO_N3;		//[TECu]
        	cnav2_ephemeris.alpha_5 = static_cast<double>(read_navigation_unsigned(string_bits, alpha_5_30))*TWO_N3*(-1);	//[TECu]
        	cnav2_ephemeris.alpha_6 = static_cast<double>(read_navigation_signed(string_bits, alpha_6_30))*TWO_N3;			//[TECu]
        	cnav2_ephemeris.alpha_7 = static_cast<double>(read_navigation_signed(string_bits, alpha_7_30))*TWO_N3;			//[TECu]
        	cnav2_ephemeris.alpha_8 = static_cast<double>(read_navigation_signed(string_bits, alpha_8_30))*TWO_N3;			//[TECu]
        	cnav2_ephemeris.alpha_9 = static_cast<double>(read_navigation_signed(string_bits, alpha_9_30))*TWO_N3;			//[TECu]
        	// Ionospheric Delay Correction Model Parameters End

        	cnav2_ephemeris.T_GDB1Cp = static_cast<double>(read_navigation_signed(string_bits, T_GDB1Cp_30))*TWO_N34;		//[s]
        	cnav2_ephemeris.Rev = static_cast<double>(read_navigation_unsigned(string_bits, Rev_30));
        	cnav2_ephemeris.CRC = static_cast<double>(read_navigation_unsigned(string_bits, CRC_30));

            break;

        case 31:
            // --- It is Type 31 ----------------------------------------------

        	cnav2_ephemeris.PRN = static_cast<unsigned int>(read_navigation_unsigned(string_bits, PRN));
			//cnav2_ephemeris.MesType = static_cast<double>(read_navigation_unsigned(string_bits, MesType));
			cnav2_ephemeris.SOW = static_cast<double>(read_navigation_unsigned(string_bits, SOW))*3;	//[s] effective range 0~604797
			cnav2_ephemeris.HS = static_cast<double>(read_navigation_unsigned(string_bits, HS_31));
			cnav2_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(string_bits, DIF_31));
			cnav2_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(string_bits, SIF_31));
			cnav2_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(string_bits, AIF_31));
			cnav2_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(string_bits, SISMAI_31));
			cnav2_ephemeris.DIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, DIF_B1C_31));
			cnav2_ephemeris.SIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, SIF_B1C_31));
			cnav2_ephemeris.AIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, AIF_B1C_31));

			// Clock Correction Parameters (69 bits)
			cnav2_utc_model.t_oc = static_cast<double>(read_navigation_unsigned(string_bits, t_oc_31))*300; //[s]
			cnav2_utc_model.a_0 = static_cast<double>(read_navigation_signed(string_bits, a_0_31))*TWO_N34; //[s]
			cnav2_utc_model.a_1 = static_cast<double>(read_navigation_signed(string_bits, a_1_31))*TWO_N50; //[s/s]
			cnav2_utc_model.a_2 = static_cast<double>(read_navigation_signed(string_bits, a_2_31))*TWO_N66; //[s/s^2]
			// Clock Correction Parameters End

			cnav2_ephemeris.IODC = static_cast<double>(read_navigation_unsigned(string_bits, IODC_31));

			// Reduced Almanac Parameters Sat 1(38 bits)
			i_alm_satellite_slot_number = static_cast<unsigned int>(read_navigation_unsigned(string_bits, PRN_a1_31));								//[dimensionless] effective range 1~63
			cnav2_almanac[i_alm_satellite_slot_number-1].WN_a = static_cast<double>(read_navigation_unsigned(string_bits, WN_a_31));				//[week] effective range 0~8191
			cnav2_almanac[i_alm_satellite_slot_number-1].t_oa = static_cast<double>(read_navigation_unsigned(string_bits, t_oa_31))*TWO_P12;		//[s] effective range 0~602112
			cnav2_almanac[i_alm_satellite_slot_number-1].SatType = static_cast<double>(read_navigation_unsigned(string_bits, SatType1_31));			//[dimensionless] binary, 01:GEO, 10:IGSO, 11:MEO, 00:Reserved
			cnav2_almanac[i_alm_satellite_slot_number-1].delta_A = static_cast<double>(read_navigation_signed(string_bits, delta_A1_31))*TWO_P9;	//[m] reference MEO: 27906100m, IGSO/GEO: 42162200m
			cnav2_almanac[i_alm_satellite_slot_number-1].Omega_0 = static_cast<double>(read_navigation_signed(string_bits, Omega_01_31))*TWO_N6;	//[pi]
			cnav2_almanac[i_alm_satellite_slot_number-1].Phi_0 = static_cast<double>(read_navigation_signed(string_bits, Phi_01_31))*TWO_N6;		//[pi] Phi = M0 + omega, e=0, delta_i=0, MEO/IGSO i=55, GEO i=0
			cnav2_almanac[i_alm_satellite_slot_number-1].Health = static_cast<double>(read_navigation_unsigned(string_bits, Health1_31));			//[dimensionless]
			// Reduced Almanac Parameters End

			// Reduced Almanac Parameters Sat 2(38 bits)
			i_alm_satellite_slot_number = static_cast<unsigned int>(read_navigation_unsigned(string_bits, PRN_a2_31));								//[dimensionless] effective range 1~63
			cnav2_almanac[i_alm_satellite_slot_number-1].WN_a = static_cast<double>(read_navigation_unsigned(string_bits, WN_a_31));				//[week] effective range 0~8191
			cnav2_almanac[i_alm_satellite_slot_number-1].t_oa = static_cast<double>(read_navigation_unsigned(string_bits, t_oa_31))*TWO_P12;		//[s] effective range 0~602112
			cnav2_almanac[i_alm_satellite_slot_number-1].SatType = static_cast<double>(read_navigation_unsigned(string_bits, SatType2_31));			//[dimensionless] binary, 01:GEO, 10:IGSO, 11:MEO, 00:Reserved
			cnav2_almanac[i_alm_satellite_slot_number-1].delta_A = static_cast<double>(read_navigation_signed(string_bits, delta_A2_31))*TWO_P9;	//[m] reference MEO: 27906100m, IGSO/GEO: 42162200m
			cnav2_almanac[i_alm_satellite_slot_number-1].Omega_0 = static_cast<double>(read_navigation_signed(string_bits, Omega_02_31))*TWO_N6;	//[pi]
			cnav2_almanac[i_alm_satellite_slot_number-1].Phi_0 = static_cast<double>(read_navigation_signed(string_bits, Phi_02_31))*TWO_N6;		//[pi] Phi = M0 + omega, e=0, delta_i=0, MEO/IGSO i=55, GEO i=0
			cnav2_almanac[i_alm_satellite_slot_number-1].Health = static_cast<double>(read_navigation_unsigned(string_bits, Health2_31));			//[dimensionless]
			// Reduced Almanac Parameters End

			// Reduced Almanac Parameters Sat 3(38 bits)
			i_alm_satellite_slot_number = static_cast<unsigned int>(read_navigation_unsigned(string_bits, PRN_a3_31));								//[dimensionless] effective range 1~63
			cnav2_almanac[i_alm_satellite_slot_number-1].WN_a = static_cast<double>(read_navigation_unsigned(string_bits, WN_a_31));				//[week] effective range 0~8191
			cnav2_almanac[i_alm_satellite_slot_number-1].t_oa = static_cast<double>(read_navigation_unsigned(string_bits, t_oa_31))*TWO_P12;		//[s] effective range 0~602112
			cnav2_almanac[i_alm_satellite_slot_number-1].SatType = static_cast<double>(read_navigation_unsigned(string_bits, SatType3_31));			//[dimensionless] binary, 01:GEO, 10:IGSO, 11:MEO, 00:Reserved
			cnav2_almanac[i_alm_satellite_slot_number-1].delta_A = static_cast<double>(read_navigation_signed(string_bits, delta_A3_31))*TWO_P9;	//[m] reference MEO: 27906100m, IGSO/GEO: 42162200m
			cnav2_almanac[i_alm_satellite_slot_number-1].Omega_0 = static_cast<double>(read_navigation_signed(string_bits, Omega_03_31))*TWO_N6;	//[pi]
			cnav2_almanac[i_alm_satellite_slot_number-1].Phi_0 = static_cast<double>(read_navigation_signed(string_bits, Phi_03_31))*TWO_N6;		//[pi] Phi = M0 + omega, e=0, delta_i=0, MEO/IGSO i=55, GEO i=0
			cnav2_almanac[i_alm_satellite_slot_number-1].Health = static_cast<double>(read_navigation_unsigned(string_bits, Health3_31));			//[dimensionless]
			// Reduced Almanac Parameters End

			cnav2_ephemeris.Rev = static_cast<double>(read_navigation_unsigned(string_bits, Rev_31));
			cnav2_ephemeris.CRC = static_cast<double>(read_navigation_unsigned(string_bits, CRC_31));

			flag_almanac_str_31 = true;
            break;

        case 32:
            // --- It is Type 32 ----------------------------------------------

        	cnav2_ephemeris.PRN = static_cast<unsigned int>(read_navigation_unsigned(string_bits, PRN));
			//cnav2_ephemeris.MesType = static_cast<double>(read_navigation_unsigned(string_bits, MesType));
			cnav2_ephemeris.SOW = static_cast<double>(read_navigation_unsigned(string_bits, SOW))*3;	//[s] effective range 0~604797
			cnav2_ephemeris.HS = static_cast<double>(read_navigation_unsigned(string_bits, HS_32));
			cnav2_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(string_bits, DIF_32));
			cnav2_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(string_bits, SIF_32));
			cnav2_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(string_bits, AIF_32));
			cnav2_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(string_bits, SISMAI_32));
			cnav2_ephemeris.DIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, DIF_B1C_32));
			cnav2_ephemeris.SIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, SIF_B1C_32));
			cnav2_ephemeris.AIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, AIF_B1C_32));

        	// Clock Correction Parameters (69 bits)
			cnav2_utc_model.t_oc = static_cast<double>(read_navigation_unsigned(string_bits, t_oc_32))*300; //[s]
			cnav2_utc_model.a_0 = static_cast<double>(read_navigation_signed(string_bits, a_0_32))*TWO_N34; //[s]
			cnav2_utc_model.a_1 = static_cast<double>(read_navigation_signed(string_bits, a_1_32))*TWO_N50; //[s/s]
			cnav2_utc_model.a_2 = static_cast<double>(read_navigation_signed(string_bits, a_2_32))*TWO_N66; //[s/s^2]
        	// Clock Correction Parameters End

			cnav2_ephemeris.IODC = static_cast<double>(read_navigation_unsigned(string_bits, IODC_32));

        	// EOP Parameters (138 bits)
			cnav2_ephemeris.t_EOP = static_cast<double>(read_navigation_unsigned(string_bits, t_EOP_32))*TWO_P4;			//[s] effective range 0~604784
			cnav2_ephemeris.PM_X = static_cast<double>(read_navigation_signed(string_bits, PM_X_32))*TWO_N20;				//[arc-seconds]
			cnav2_ephemeris.PM_X_dot = static_cast<double>(read_navigation_signed(string_bits, PM_X_dot_32))*TWO_N21;		//[arc-seconds/day]
			cnav2_ephemeris.PM_Y = static_cast<double>(read_navigation_signed(string_bits, PM_Y_32))*TWO_N20;				//[arc-seconds]
			cnav2_ephemeris.PM_Y_dot = static_cast<double>(read_navigation_signed(string_bits, PM_Y_dot_32))*TWO_N21;		//[arc-seconds/day]
			cnav2_ephemeris.dUT1 = static_cast<double>(read_navigation_signed(string_bits, dUT1_32))*TWO_N24;				//[s]
			cnav2_ephemeris.dUT1_dot = static_cast<double>(read_navigation_signed(string_bits, dUT1_dot_32))*TWO_N25; 		//[s/day]
        	// EOP Parameters End

			cnav2_ephemeris.Rev = static_cast<double>(read_navigation_unsigned(string_bits, Rev_32));
			cnav2_ephemeris.CRC = static_cast<double>(read_navigation_unsigned(string_bits, CRC_32));

            break;

        case 33:
            // --- It is Type 33 ----------------------------------------------

        	cnav2_ephemeris.PRN = static_cast<unsigned int>(read_navigation_unsigned(string_bits, PRN));
			//cnav2_ephemeris.MesType = static_cast<double>(read_navigation_unsigned(string_bits, MesType));
			cnav2_ephemeris.SOW = static_cast<double>(read_navigation_unsigned(string_bits, SOW))*3;	//[s] effective range 0~604797
			cnav2_ephemeris.HS = static_cast<double>(read_navigation_unsigned(string_bits, HS_33));
			cnav2_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(string_bits, DIF_33));
			cnav2_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(string_bits, SIF_33));
			cnav2_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(string_bits, AIF_33));
			cnav2_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(string_bits, SISMAI_33));
			cnav2_ephemeris.DIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, DIF_B1C_33));
			cnav2_ephemeris.SIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, SIF_B1C_33));
			cnav2_ephemeris.AIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, AIF_B1C_33));

        	// Clock Correction Parameters (69 bits)
			cnav2_utc_model.t_oc = static_cast<double>(read_navigation_unsigned(string_bits, t_oc_33))*300; //[s]
			cnav2_utc_model.a_0 = static_cast<double>(read_navigation_signed(string_bits, a_0_33))*TWO_N34; //[s]
			cnav2_utc_model.a_1 = static_cast<double>(read_navigation_signed(string_bits, a_1_33))*TWO_N50; //[s/s]
			cnav2_utc_model.a_2 = static_cast<double>(read_navigation_signed(string_bits, a_2_33))*TWO_N66; //[s/s^2]
        	// Clock Correction Parameters End

        	// BGTO Parameters (68 bits)
			cnav2_utc_model.GNSS_ID = static_cast<double>(read_navigation_unsigned(string_bits, GNSS_ID_33)); 				//[dimensionless]
			cnav2_utc_model.WN_0BGTO = static_cast<double>(read_navigation_unsigned(string_bits, WN_0BGTO_33));				//[week]
			cnav2_utc_model.t_0BGTO = static_cast<double>(read_navigation_unsigned(string_bits, t_0BGTO_33))*TWO_P4;		//[s] effective range 0~604784
			cnav2_utc_model.A_0BGTO = static_cast<double>(read_navigation_unsigned(string_bits, A_0BGTO_33))*TWO_N35;		//[s]
			cnav2_utc_model.A_1BGTO = static_cast<double>(read_navigation_unsigned(string_bits, A_1BGTO_33))*TWO_N51;		//[s/s]
			cnav2_utc_model.A_2BGTO = static_cast<double>(read_navigation_unsigned(string_bits, A_2BGTO_33))*TWO_N68;		//[s/s^2]
        	// BGTO Parameters End

        	// Reduced Almanac Parameters (38 bits)
			i_alm_satellite_slot_number = static_cast<unsigned int>(read_navigation_unsigned(string_bits, PRN_a_33));								//[dimensionless] effective range 1~63
			cnav2_almanac[i_alm_satellite_slot_number-1].SatType = static_cast<double>(read_navigation_unsigned(string_bits, SatType_33));			//[dimensionless] binary, 01:GEO, 10:IGSO, 11:MEO, 00:Reserved
			cnav2_almanac[i_alm_satellite_slot_number-1].delta_A = static_cast<double>(read_navigation_unsigned(string_bits, delta_A_33))*TWO_P9;	//[m] reference MEO: 27906100m, IGSO/GEO: 42162200m
			cnav2_almanac[i_alm_satellite_slot_number-1].Omega_0 = static_cast<double>(read_navigation_unsigned(string_bits, Omega_0_33))*TWO_N6;	//[pi]
			cnav2_almanac[i_alm_satellite_slot_number-1].Phi_0 = static_cast<double>(read_navigation_unsigned(string_bits, Phi_0_33))*TWO_N6;		//[pi] Phi = M0 + omega, e=0, delta_i=0, MEO/IGSO i=55, GEO i=0
			cnav2_almanac[i_alm_satellite_slot_number-1].Health = static_cast<double>(read_navigation_unsigned(string_bits, Health_33));			//[dimensionless]
        	// Reduced Almanac Parameters End

			cnav2_ephemeris.IODC = static_cast<double>(read_navigation_unsigned(string_bits, IODC_33));
			cnav2_almanac[i_alm_satellite_slot_number-1].WN_a = static_cast<double>(read_navigation_unsigned(string_bits, WN_a_33));			//[week] effective range 0~8191
			cnav2_almanac[i_alm_satellite_slot_number-1].t_oa = static_cast<double>(read_navigation_unsigned(string_bits, t_oa_33))*TWO_P12;	//[s] effective range 0~602112
			cnav2_ephemeris.Rev = static_cast<double>(read_navigation_unsigned(string_bits, Rev_33));
			cnav2_ephemeris.CRC = static_cast<double>(read_navigation_unsigned(string_bits, CRC_33));

            break;

        case 34:
            // --- It is Type 34 ----------------------------------------------

        	cnav2_ephemeris.PRN = static_cast<unsigned int>(read_navigation_unsigned(string_bits, PRN));
			//cnav2_ephemeris.MesType = static_cast<double>(read_navigation_unsigned(string_bits, MesType));
			cnav2_ephemeris.SOW = static_cast<double>(read_navigation_unsigned(string_bits, SOW))*3;	//[s] effective range 0~604797
			cnav2_ephemeris.HS = static_cast<double>(read_navigation_unsigned(string_bits, HS_34));
			cnav2_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(string_bits, DIF_34));
			cnav2_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(string_bits, SIF_34));
			cnav2_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(string_bits, AIF_34));
			cnav2_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(string_bits, SISMAI_34));
			cnav2_ephemeris.DIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, DIF_B1C_34));
			cnav2_ephemeris.SIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, SIF_B1C_34));
			cnav2_ephemeris.AIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, AIF_B1C_34));

        	// SISAI_OC (22 bits)
			cnav2_ephemeris.t_op = static_cast<double>(read_navigation_unsigned(string_bits, t_op_34));
			cnav2_ephemeris.SISAI_ocb = static_cast<double>(read_navigation_unsigned(string_bits, SISAI_ocb_34));
			cnav2_ephemeris.SISAI_oc1 = static_cast<double>(read_navigation_unsigned(string_bits, SISAI_oc1_34));
			cnav2_ephemeris.SISAI_oc2 = static_cast<double>(read_navigation_unsigned(string_bits, SISAI_oc2_34));
        	// SISAI_OC End

        	// Clock Correction Parameters (69 bits)
			cnav2_utc_model.t_oc = static_cast<double>(read_navigation_unsigned(string_bits, t_oc_34))*300; 		//[s]
			cnav2_utc_model.a_0 = static_cast<double>(read_navigation_signed(string_bits, a_0_34))*TWO_N34; 		//[s]
			cnav2_utc_model.a_1 = static_cast<double>(read_navigation_signed(string_bits, a_1_34))*TWO_N50; 		//[s/s]
			cnav2_utc_model.a_2 = static_cast<double>(read_navigation_signed(string_bits, a_2_34))*TWO_N66; 		//[s/s^2]
        	// Clock Correction Parameters End

			cnav2_ephemeris.IODC = static_cast<double>(read_navigation_unsigned(string_bits, IODC_34));

        	// BDT-UTC Time Offset Parameters (97 bits)
			cnav2_utc_model.A_0UTC = static_cast<double>(read_navigation_signed(string_bits, A_0UTC_34))*TWO_N35;	//[s]
        	cnav2_utc_model.A_1UTC = static_cast<double>(read_navigation_signed(string_bits, A_1UTC_34))*TWO_N51;	//[s/s]
        	cnav2_utc_model.A_2UTC = static_cast<double>(read_navigation_signed(string_bits, A_2UTC_34))*TWO_N68;	//[s/s^2]
        	cnav2_utc_model.dt_LS = static_cast<double>(read_navigation_signed(string_bits, dt_LS_34));			//[s]
        	cnav2_utc_model.t_ot = static_cast<double>(read_navigation_unsigned(string_bits, t_ot_34))*TWO_P4;		//[s] effective range 0~604784
        	cnav2_utc_model.WN_ot = static_cast<double>(read_navigation_unsigned(string_bits, WN_ot_34));			//[week]
        	cnav2_utc_model.WN_LSF = static_cast<double>(read_navigation_unsigned(string_bits, WN_LSF_34));		//[week]
        	cnav2_utc_model.DN = static_cast<double>(read_navigation_unsigned(string_bits, DN_34));				//[day] effective range 0~6
        	cnav2_utc_model.dt_LSF = static_cast<double>(read_navigation_signed(string_bits, dt_LSF_34));			//[s]
        	// BDT-UTC Time Offset Parameters End

        	cnav2_ephemeris.Rev = static_cast<double>(read_navigation_unsigned(string_bits, Rev_34));
        	cnav2_ephemeris.CRC = static_cast<double>(read_navigation_unsigned(string_bits, CRC_34));

            break;

        case 40:
            // --- It is Type 40 ----------------------------------------------

        	cnav2_ephemeris.PRN = static_cast<unsigned int>(read_navigation_unsigned(string_bits, PRN));
			//cnav2_ephemeris.MesType = static_cast<double>(read_navigation_unsigned(string_bits, MesType));
			cnav2_ephemeris.SOW = static_cast<double>(read_navigation_unsigned(string_bits, SOW))*3;	//[s] effective range 0~604797
			cnav2_ephemeris.HS = static_cast<double>(read_navigation_unsigned(string_bits, HS_40));
			cnav2_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(string_bits, DIF_40));
			cnav2_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(string_bits, SIF_40));
			cnav2_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(string_bits, AIF_40));
			cnav2_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(string_bits, SISMAI_40));
			cnav2_ephemeris.DIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, DIF_B1C_40));
			cnav2_ephemeris.SIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, SIF_B1C_40));
			cnav2_ephemeris.AIF_B1C = static_cast<double>(read_navigation_unsigned(string_bits, AIF_B1C_40));
			cnav2_ephemeris.SISAI_OE = static_cast<double>(read_navigation_unsigned(string_bits, SISAI_OE_40));

        	// SISAI_OC (22 bits)
			cnav2_ephemeris.t_op = static_cast<double>(read_navigation_unsigned(string_bits, t_op_40));
			cnav2_ephemeris.SISAI_ocb = static_cast<double>(read_navigation_unsigned(string_bits, SISAI_ocb_40));
			cnav2_ephemeris.SISAI_oc1 = static_cast<double>(read_navigation_unsigned(string_bits, SISAI_oc1_40));
			cnav2_ephemeris.SISAI_oc2 = static_cast<double>(read_navigation_unsigned(string_bits, SISAI_oc2_40));
        	// SISAI_OC End

        	// Midi Almanac Parameters (156 bits)
			i_alm_satellite_slot_number = static_cast<unsigned int>(read_navigation_unsigned(string_bits, PRN_a_40));					//[dimensionless] effective range 1~63
        	cnav2_almanac[i_alm_satellite_slot_number-1].SatType = static_cast<double>(read_navigation_unsigned(string_bits, SatType_40));				//[dimensionless] binary, 01:GEO, 10:IGSO, 11:MEO, 00:Reserved
        	cnav2_almanac[i_alm_satellite_slot_number-1].WN_a = static_cast<double>(read_navigation_unsigned(string_bits, WN_a_40));					//[week] effective range 0~8191
        	cnav2_almanac[i_alm_satellite_slot_number-1].t_oa = static_cast<double>(read_navigation_unsigned(string_bits, t_oa_40))*TWO_P12;			//[s] effective range 0~602112
        	cnav2_almanac[i_alm_satellite_slot_number-1].e = static_cast<double>(read_navigation_unsigned(string_bits, e_40))*TWO_N16;					//[dimensionless]
        	cnav2_almanac[i_alm_satellite_slot_number-1].delta_i = static_cast<double>(read_navigation_signed(string_bits, delta_i_40))*TWO_N14;		//[pi]
        	cnav2_almanac[i_alm_satellite_slot_number-1].sqrt_A = static_cast<double>(read_navigation_unsigned(string_bits, sqrt_A_40))*TWO_N4; 		//[m^0.5]
        	cnav2_almanac[i_alm_satellite_slot_number-1].Omega_0 = static_cast<double>(read_navigation_signed(string_bits, Omega_0_40))*TWO_N15;		//[pi]
        	cnav2_almanac[i_alm_satellite_slot_number-1].Omega_dot = static_cast<double>(read_navigation_signed(string_bits, Omega_dot_40))*TWO_N33;	//[pi/s]
        	cnav2_almanac[i_alm_satellite_slot_number-1].omega = static_cast<double>(read_navigation_signed(string_bits, omega_40))*TWO_N15;			//[pi]
        	cnav2_almanac[i_alm_satellite_slot_number-1].M_0 = static_cast<double>(read_navigation_signed(string_bits, M_0_40))*TWO_N15;				//[pi]
        	cnav2_almanac[i_alm_satellite_slot_number-1].a_f0 = static_cast<double>(read_navigation_signed(string_bits, a_f0_40))*TWO_N20;				//[s]
        	cnav2_almanac[i_alm_satellite_slot_number-1].a_f1 = static_cast<double>(read_navigation_signed(string_bits, a_f1_40))*TWO_N37;				//[s/s]
        	cnav2_almanac[i_alm_satellite_slot_number-1].Health = static_cast<double>(read_navigation_unsigned(string_bits, Health_40));				//[dimensionless] 8th(MSB):Satellite, 7th:B1C, 6th:B2a, 5th~1st:reserve, 0:normal/health, 1:abnormal
        	// Midi Almanac Parameters End

        	cnav2_ephemeris.Rev = static_cast<double>(read_navigation_unsigned(string_bits, Rev_40));
        	cnav2_ephemeris.CRC = static_cast<double>(read_navigation_unsigned(string_bits, CRC_40));

            break;


        default:
            LOG(INFO) << "BEIDOU CNAV2: Invalid String ID of received. Received " << d_string_ID
                      << ", but acceptable range is from 10 11 30 31 32 33 34 40";

            break;
        }  // switch string ID

    return d_string_ID;
}


Beidou_Cnav2_Ephemeris Beidou_Cnav2_Navigation_Message::get_ephemeris()
{
    return cnav2_ephemeris;
}

Beidou_Cnav2_Utc_Model Beidou_Cnav2_Navigation_Message::get_utc_model()
{
    return cnav2_utc_model;
}

Beidou_Cnav2_Almanac Beidou_Cnav2_Navigation_Message::get_almanac(unsigned int satellite_slot_number)
{
    return cnav2_almanac[satellite_slot_number - 1];
}


bool Beidou_Cnav2_Navigation_Message::have_new_ephemeris()  //Check if we have a new ephemeris stored in the galileo navigation class
{
    bool new_eph = false;
    // We need to make sure we have received the ephemeris info plus the time info
    if ((flag_ephemeris_str_1 == true) and (flag_ephemeris_str_2 == true) and
        (flag_utc_model_str_5 == true))
    	/*
        {
            if (d_previous_tb != cnav2_ephemeris.d_t_b)
                {
                    flag_ephemeris_str_1 = false;  // clear the flag
                    flag_ephemeris_str_2 = false;  // clear the flag
                    flag_ephemeris_str_3 = false;  // clear the flag
                    flag_ephemeris_str_4 = false;  // clear the flag
                    flag_all_ephemeris = true;
                    // Update the time of ephemeris information
                    d_previous_tb = cnav2_ephemeris.d_t_b;
                    DLOG(INFO) << "beidou cnav2 Ephemeris (1, 2, 3, 4) have been received and belong to the same batch" << std::endl;
                    new_eph = true;
                }
        }
        */

    return new_eph;
}


bool Beidou_Cnav2_Navigation_Message::have_new_utc_model()  // Check if we have a new utc data set stored in the beidou navigation class
{
    if (flag_utc_model_str_5 == true)
        {
            flag_utc_model_str_5 = false;  // clear the flag
            return true;
        }
    else
        return false;
}

/*
bool Beidou_Cnav2_Navigation_Message::have_new_almanac()  //Check if we have a new almanac data set stored in the beidou navigation class
{
    bool new_alm = false;
    if ((flag_almanac_str_6 == true) and (flag_almanac_str_7 == true))
        {
            if (d_previous_Na[i_alm_satellite_slot_number] != cnav2_utc_model.d_N_A)
                {
                    //All almanac have been received for this satellite
                    flag_almanac_str_6 = false;
                    flag_almanac_str_7 = false;
                    new_alm = true;
                }
        }
    if ((flag_almanac_str_8 == true) and (flag_almanac_str_9 == true))
        {
            if (d_previous_Na[i_alm_satellite_slot_number] != cnav2_utc_model.d_N_A)
                {
                    flag_almanac_str_8 = false;
                    flag_almanac_str_9 = false;
                    new_alm = true;
                }
        }
    if ((flag_almanac_str_10 == true) and (flag_almanac_str_11 == true))
        {
            if (d_previous_Na[i_alm_satellite_slot_number] != cnav2_utc_model.d_N_A)
                {
                    flag_almanac_str_10 = false;
                    flag_almanac_str_11 = false;
                    new_alm = true;
                }
        }
    if ((flag_almanac_str_12 == true) and (flag_almanac_str_13 == true))
        {
            if (d_previous_Na[i_alm_satellite_slot_number] != cnav2_utc_model.d_N_A)
                {
                    flag_almanac_str_12 = false;
                    flag_almanac_str_13 = false;
                    new_alm = true;
                }
        }
    if ((flag_almanac_str_14 == true) and (flag_almanac_str_15 == true))
        {
            if (d_previous_Na[i_alm_satellite_slot_number] != cnav2_utc_model.d_N_A)
                {
                    flag_almanac_str_14 = false;
                    flag_almanac_str_15 = false;
                    new_alm = true;
                }
        }

    return new_alm;
}
*/
