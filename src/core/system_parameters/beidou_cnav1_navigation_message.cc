/*!
 * \file beidou_cnav1_navigation_message.cc
 * \brief  Implementation of a beidou cnav1 Data message decoder as described in BEIDOU ICD
 * \author Andrew Kamble, 2019. andrewkamble88@gmail.com 
 * \note Code added as part of GSoC 2019 program
 * Detailed description of the file here if needed.
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2010-2019  (see AUTHORS file for a list of contributors)
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
 

#include "beidou_cnav1_navigation_message.h"
#include "gnss_satellite.h"
#include <boost/crc.hpp>  // for boost::crc_basic, boost::crc_optimal
#include <boost/dynamic_bitset.hpp>
#include <glog/logging.h>
#include <iostream>

typedef boost::crc_optimal<24, 0x1864CFBu, 0x0, 0x0, false, false> Crc_Beidou_Cnav1_type;


void Beidou_Cnav1_Navigation_Message::reset()
{
    // Satellite Identification
    i_satellite_PRN = 0;
    i_alm_satellite_PRN = 0;
   
    // Ephemeris Flags
    flag_all_ephemeris = false;
    flag_ephemeris_mes_SF2 = false;

    // Almanac Flags
    flag_almanac_mes_type_2 = false;
    flag_almanac_mes_type_4 = false;

    // UTC and System Clocks Flags
    flag_utc_model_valid = false;        //!< If set, it indicates that the UTC model parameters are filled
    flag_utc_model_mes_type_1 = false; //!< Clock info send in Type 1 Subframe 3
    flag_utc_model_mes_type_3 = false;   //!< Clock info send in Type 3 Subframe 3
    

    // broadcast orbit 1
    flag_TOW_set = false;
    flag_TOW_new = false;

    flag_crc_test = false;
    flag_crc_test_sf_2 = false;
    flag_crc_test_sf_3 = false;
    i_frame_mes_type = 0;
    i_channel_ID = 0;

    // Clock terms
    d_satClkCorr = 0.0;
    d_dtr = 0.0;
    d_satClkDrift = 0.0;

    // Data update information
    d_previous_tb = 0.0;
    for (uint32_t i = 0; i < BEIDOU_B1C_NBR_SATS; i++)
        d_previous_Na[i] = 0.0;

    std::map<int32_t, std::string> satelliteBlock;  //!< Map that stores to which block the PRN belongs http://www.navcen.uscg.gov/?Do=constellationStatus

    auto gnss_sat = Gnss_Satellite();
    std::string _system("BEIDOU");
}

Beidou_Cnav1_Navigation_Message::Beidou_Cnav1_Navigation_Message()
{
    reset();
}

//! The following function is made generalized for processing all the Three Subframes of CNAV1 navigation message
bool Beidou_Cnav1_Navigation_Message::read_navigation_bool(std::bitset<subframe_size> const &bits, const std::vector<std::pair<int32_t, int32_t>> &parameter)// "subframe_size" is template variable declared in "beidou_cnav1_navigation_message.h"
{
    bool value;

    // bitset::any() (bits.any())
    int32_t subframe_size_bits = bits.size();        //stores the size of the subframe
    if (bits[subframe_size_bits - parameter[0].first] == 1)
        {
            value = true;
        }
    else
        {
            value = false;
        }
    return value;
}

//! The following function is made generalized for processing all the Three Subframes of CNAV1 navigation message
uint64_t Beidou_Cnav1_Navigation_Message::read_navigation_unsigned(std::bitset<subframe_size> const &bits, const std::vector<std::pair<int32_t, int32_t>> &parameter)// "subframe_size" is template variable declared in "beidou_cnav1_navigation_message.h"
{
    uint64_t value = 0;
    int32_t num_of_slices = parameter.size();
    int32_t subframe_size_bits = bits.size();        //stores the size of the subframe
    for (int32_t i = 0; i < num_of_slices; i++)
        {
            for (int32_t j = 0; j < parameter[i].second; j++)
                {
                    value <<= 1;  //shift left
                    if (bits[subframe_size_bits - parameter[i].first - j] == 1)
                        {
                            value |= 1;  // insert the bit
                        }
                }
        }
    return value;
}

//! The following function is made generalized for processing all the Three Subframes of CNAV1 navigation message
int64_t Beidou_Cnav1_Navigation_Message::read_navigation_signed(std::bitset<subframe_size> const &bits, const std::vector<std::pair<int32_t, int32_t>> &parameter)// "subframe_size" is template variable declared in "beidou_cnav1_navigation_message.h"
{
    int64_t value = 0;
    int64_t sign = 0;
    int32_t num_of_slices = parameter.size();
    int32_t subframe_size_bits = bits.size();        //stores the size of the subframe
    // read the MSB and perform the sign extension
    if (bits[subframe_size_bits - parameter[0].first] == 1)
        {
            sign = -1;
        }
    else
        {
            sign = 1;
        }
    for (int32_t i = 0; i < num_of_slices; i++)
        {
            for (int32_t j = 1; j < parameter[i].second; j++)
                {
                    value <<= 1;  //shift left
                    if (bits[subframe_size_bits - parameter[i].first - j] == 1)
                        {
                            value |= 1;  // insert the bit
                        }
                }
        }
    return (sign * value);
}

bool Beidou_Cnav1_Navigation_Message::crc_test_for_subframe_2(std::bitset<BEIDOU_CNAV1_SUBFRAME_2_BITS> bits, uint32_t crc_decoded)
{
    Crc_Beidou_Cnav1_type Crc_Beidou_Cnav1_subframe_2;

    uint32_t crc_computed;

    boost::dynamic_bitset<uint8_t> frame_bits(std::string(bits.to_string()));
    std::vector<uint8_t> bytes;
    boost::to_block_range(frame_bits, std::back_inserter(bytes));
    std::reverse(bytes.begin(), bytes.end());

    // Only include data without 3 lsb from crc
    Crc_Beidou_Cnav1_subframe_2.process_bytes(bytes.data(), BEIDOU_CNAV1_SUBFRAME_2_BYTES - 3);
    crc_computed = Crc_Beidou_Cnav1_subframe_2.checksum();

    if (crc_decoded == crc_computed)
        {
            return true;
        }
    else
        {
            return false;
        }
}

bool Beidou_Cnav1_Navigation_Message::crc_test_for_subframe_3(std::bitset<BEIDOU_CNAV1_SUBFRAME_3_BITS> bits, uint32_t crc_decoded)
{
    Crc_Beidou_Cnav1_type Crc_Beidou_Cnav1_subframe_3;

    uint32_t crc_computed;

    boost::dynamic_bitset<uint8_t> frame_bits(std::string(bits.to_string()));
    std::vector<uint8_t> bytes;
    boost::to_block_range(frame_bits, std::back_inserter(bytes));
    std::reverse(bytes.begin(), bytes.end());

    // Only include data without 3 lsb from crc
    Crc_Beidou_Cnav1_subframe_3.process_bytes(bytes.data(), BEIDOU_CNAV1_SUBFRAME_3_BYTES - 3);
    crc_computed = Crc_Beidou_Cnav1_subframe_3.checksum();

    if (crc_decoded == crc_computed)
        {
            return true;
        }
    else
        {
            return false;
        }
}


int32_t Beidou_Cnav1_Navigation_Message::frame_decoder(std::string const &frame_string)
{
	// Gets the message data which contains Subframe 1,Subframe 2,Subframe 3
    	std::bitset<BEIDOU_CNAV1_DATA_BITS> frame_bits(frame_string);
    
        std::string subframe1_string;   					//string for storing subframe 1
	std::string subframe2_string;						//string for storing subframe 2
	std::string subframe3_string;						//string for storing subframe 3
	
	
	subframe1_string=frame_string.substr(0, BEIDOU_CNAV1_SUBFRAME_1_BITS);
	subframe2_string=frame_string.substr(14, BEIDOU_CNAV1_SUBFRAME_2_BITS);
	subframe3_string=frame_string.substr(614, BEIDOU_CNAV1_SUBFRAME_3_BITS);
	
	std::bitset<BEIDOU_CNAV1_SUBFRAME_1_BITS> subframe1(subframe1_string);  //Gets the message data which contains Subframe 1
	std::bitset<BEIDOU_CNAV1_SUBFRAME_2_BITS> subframe2(subframe2_string);  //Gets the message data which contains Subframe 2
	std::bitset<BEIDOU_CNAV1_SUBFRAME_3_BITS> subframe3(subframe3_string);  //Gets the message data which contains Subframe 3
    
    
    	
    	//! CNAV1 navigation message have CRC in two subframes i.e in Subframe 2 and Subframe 3
    	
    	// Gets the crc data for comparison, last 24 bits from 600 bit of Subframe 2
    	std::bitset<BEIDOU_CNAV1_CRC_BITS> checksum_sf_2(subframe2.substr(576, 24));
    	// Gets the crc data for comparison, last 24 bits from 264 bit of Subframe 3
    	std::bitset<BEIDOU_CNAV1_CRC_BITS> checksum_sf_3(subframe3.substr(240, 24));

    	// Perform data verification and exit code if error in bit sequence
    	flag_crc_test_sf_2 = crc_test_for_subframe_2(subframe2, checksum_sf_2.to_ulong());
    	flag_crc_test_sf_3 = crc_test_for_subframe_3(subframe3, checksum_sf_3.to_ulong());

    	if (flag_crc_test_sf_2 == false && flag_crc_test_sf_3 == false)
        	return 0;
        
        
        /*// Gets the crc data for comparison, last 24 bits from 288 bit data frame
    	std::bitset<BEIDOU_CNAV1_CRC_BITS> checksum(frame_string.substr(854, 24));

    	// Perform data verification and exit code if error in bit sequence
    	flag_crc_test = crc_test(frame_bits, checksum.to_ulong());

    	if (flag_crc_test == false)
        	return 0;*/
    	
    	//===============================================Decode string message of Subframe 1==============================================
    	
    	cnav1_ephemeris.i_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(subframe1, PRN));
    	cnav1_ephemeris.SOH = static_cast<uint32_t>(read_navigation_unsigned(subframe1, SOH)) * SECONDS_OF_HOUR_SCALE_FACTOR;   ///Scalling factor= 18, Unit= [s]
    	flag_TOW_set = true;
    	flag_TOW_SF_1 = true;
    	flag_ephemeris_mes_SF1 = true;
    	
     	
    	
    	//===============================================Decode string message of Subframe 2==============================================
    	
    	cnav1_ephemeris.i_BDS_week = static_cast<double>(read_navigation_unsigned(subframe2, WN_S_2));  //[week] effective range 0~8191
    	cnav1_ephemeris.HOW = static_cast<double>(read_navigation_unsigned(subframe2, HOW_S_2));       //[s] effective range 0~604797
    	cnav1_ephemeris.IODC = static_cast<double>(read_navigation_unsigned(subframe2, IODC_S_2));
    	cnav1_ephemeris.IODE = static_cast<double>(read_navigation_unsigned(subframe2, IODE_S_2));
    	
        // Ephemeris I Start
        cnav1_ephemeris.t_oe = static_cast<double>(read_navigation_unsigned(subframe2, t_oe_S_2)) * EPHEMERIS_REFERENCE_TIME_SCALE_FACTOR;                              //[s] effective range 0~604500
        cnav1_ephemeris.SatType = static_cast<double>(read_navigation_unsigned(subframe2, SatType_S_2));                              //[dimensionless] binary, 01:GEO, 10:IGSO, 11:MEO, 00:Reserved
        cnav1_ephemeris.dA = static_cast<double>(read_navigation_signed(subframe2, dA_S_2)) * TWO_N9;                                 //[m] reference MEO: 27906100m, IGSO/GEO: 42162200m
        cnav1_ephemeris.A_dot = static_cast<double>(read_navigation_signed(subframe2, A_dot_S_2)) * TWO_N21;                          //[m/s]
        cnav1_ephemeris.dn_0 = static_cast<double>(read_navigation_signed(subframe2, dn_0_S_2)) * BEIDOU_CNAV1_PI * TWO_N44;          //[pi/s]
        cnav1_ephemeris.dn_0_dot = static_cast<double>(read_navigation_signed(subframe2, dn_0_dot_S_2)) * BEIDOU_CNAV1_PI * TWO_N57;  //[pi/s^2]
        cnav1_ephemeris.M_0 = static_cast<double>(read_navigation_signed(subframe2, M_0_S_2)) * BEIDOU_CNAV1_PI * TWO_N32;            //[pi]
        cnav1_ephemeris.e = static_cast<double>(read_navigation_unsigned(subframe2, e_S_2)) * TWO_N34;                                //[dimensionless]
        cnav1_ephemeris.omega = static_cast<double>(read_navigation_signed(subframe2, omega_S_2)) * BEIDOU_CNAV1_PI * TWO_N32;        //[pi]
       // Ephemeris I End    	
       
       // Ephemeris II Start
       cnav1_ephemeris.Omega_0 = static_cast<double>(read_navigation_signed(subframe2, Omega_0_S_2)) * BEIDOU_CNAV1_PI * TWO_N32;      //[pi]
       cnav1_ephemeris.i_0 = static_cast<double>(read_navigation_signed(subframe2, i_0_S_2)) * BEIDOU_CNAV1_PI * TWO_N32;              //[pi]
       cnav1_ephemeris.Omega_dot = static_cast<double>(read_navigation_signed(subframe2, Omega_dot_S_2)) * BEIDOU_CNAV1_PI * TWO_N44;  //[pi/s]
       cnav1_ephemeris.i_0_dot = static_cast<double>(read_navigation_signed(subframe2, i_0_dot_S_2)) * BEIDOU_CNAV1_PI * TWO_N44;      //[pi/s]
       cnav1_ephemeris.C_IS = static_cast<double>(read_navigation_signed(subframe2, C_IS_S_2)) * TWO_N30;                              //[rad]
       cnav1_ephemeris.C_IC = static_cast<double>(read_navigation_signed(subframe2, C_IC_S_2)) * TWO_N30;                              //[rad]
       cnav1_ephemeris.C_RS = static_cast<double>(read_navigation_signed(subframe2, C_RS_S_2)) * TWO_N8;                               //[m]
       cnav1_ephemeris.C_RC = static_cast<double>(read_navigation_signed(subframe2, C_RC_S_2)) * TWO_N8;                               //[m]
       cnav1_ephemeris.C_US = static_cast<double>(read_navigation_signed(subframe2, C_US_S_2)) * TWO_N30;                              //[rad]
       cnav1_ephemeris.C_UC = static_cast<double>(read_navigation_signed(subframe2, C_UC_S_2)) * TWO_N30;                              //[rad]
       // Ephemeris II End

       // Clock Correction Parameters (69 bits)
       cnav1_ephemeris.t_oc = static_cast<double>(read_navigation_unsigned(subframe2, t_oc_S_2)) * 300;  //[s]
       cnav1_ephemeris.a_0 = static_cast<double>(read_navigation_signed(subframe2, a_0_S_2)) * TWO_N34;  //[s]
       cnav1_ephemeris.a_1 = static_cast<double>(read_navigation_signed(subframe2, a_1_S_2)) * TWO_N50;  //[s/s]
       cnav1_ephemeris.a_2 = static_cast<double>(read_navigation_signed(subframe2, a_2_S_2)) * TWO_N66;  //[s/s^2]
       // Clock Correction Parameters End	  	  	
       
       cnav1_ephemeris.T_GDB2ap = static_cast<double>(read_navigation_signed(subframe2, T_GDB2ap_S_2)) * TWO_N34;    //[s]	
       cnav1_ephemeris.ISC_B1Cd = static_cast<double>(read_navigation_unsigned(subframe2, ISC_B1Cd_S_2)) * TWO_N34;  //[s]
       cnav1_ephemeris.T_GDB1Cp = static_cast<double>(read_navigation_signed(subframe2, T_GDB1Cp_S_2)) * TWO_N34;  //[s]
       cnav1_ephemeris.Rev_S_2 = static_cast<double>(read_navigation_unsigned(subframe2, Rev_S_2));
       
       flag_TOW_set = true;
       flag_TOW_SF_2 = true;
       flag_ephemeris_mes_SF2 = true;
       

    	//===============================================Decode all 4 string messages of Subframe 3==============================================
    	
    	i_frame_mes_type = static_cast<uint32_t>(read_navigation_unsigned(subframe3, PageId));
    	
    	switch (i_frame_mes_type)
        {
        case 1:
            //--- It is Type 1 of Subframe 3 -----------------------------------------------
            //cnav1_ephemeris.PageId = static_cast<double>(read_navigation_unsigned(subframe3, PageId));
            cnav1_ephemeris.HS = static_cast<double>(read_navigation_unsigned(subframe3, HS_1));
            cnav1_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(subframe3, DIF_1));
            cnav1_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(subframe3, SIF_1));
            cnav1_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(subframe3, AIF_1));
            cnav1_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(subframe3, SISMAI_1));
            cnav1_ephemeris.SISAI_OE = static_cast<double>(read_navigation_unsigned(subframe3, SISAI_oe_1));
            
            // SISAI_OC (22 bits)
            cnav1_ephemeris.t_op = static_cast<double>(read_navigation_unsigned(subframe3, t_op_1));
            cnav1_ephemeris.SISAI_ocb = static_cast<double>(read_navigation_unsigned(subframe3, SISAI_ocb_1));
            cnav1_ephemeris.SISAI_oc1 = static_cast<double>(read_navigation_unsigned(subframe3, SISAI_oc1_1));
            cnav1_ephemeris.SISAI_oc2 = static_cast<double>(read_navigation_unsigned(subframe3, SISAI_oc2_1));
            // SISAI_OC End
            
            // Ionospheric Delay Correction Model Parameters (74 bits)
            cnav1_iono.alpha1 = static_cast<double>(read_navigation_unsigned(subframe3, alpha_1_1)) * TWO_N3;         //[TECu]
            cnav1_iono.alpha2 = static_cast<double>(read_navigation_signed(subframe3, alpha_2_1)) * TWO_N3;           //[TECu]
            cnav1_iono.alpha3 = static_cast<double>(read_navigation_unsigned(subframe3, alpha_3_1)) * TWO_N3;         //[TECu]
            cnav1_iono.alpha4 = static_cast<double>(read_navigation_unsigned(subframe3, alpha_4_1)) * TWO_N3;         //[TECu]
            cnav1_iono.alpha5 = static_cast<double>(read_navigation_unsigned(subframe3, alpha_5_1)) * TWO_N3 * (-1);  //[TECu]
            cnav1_iono.alpha6 = static_cast<double>(read_navigation_signed(subframe3, alpha_6_1)) * TWO_N3;           //[TECu]
            cnav1_iono.alpha7 = static_cast<double>(read_navigation_signed(subframe3, alpha_7_1)) * TWO_N3;           //[TECu]
            cnav1_iono.alpha8 = static_cast<double>(read_navigation_signed(subframe3, alpha_8_1)) * TWO_N3;           //[TECu]
            cnav1_iono.alpha9 = static_cast<double>(read_navigation_signed(subframe3, alpha_9_1)) * TWO_N3;           //[TECu]
            // Ionospheric Delay Correction Model Parameters End
            
            // BDT-UTC Time Offset Parameters (97 bits)
            cnav1_utc_model.A_0UTC = static_cast<double>(read_navigation_signed(subframe3, A_0UTC_1)) * TWO_N35;  //[s]
            cnav1_utc_model.A_1UTC = static_cast<double>(read_navigation_signed(subframe3, A_1UTC_1)) * TWO_N51;  //[s/s]
            cnav1_utc_model.A_2UTC = static_cast<double>(read_navigation_signed(subframe3, A_2UTC_1)) * TWO_N68;  //[s/s^2]
            cnav1_utc_model.dt_LS = static_cast<double>(read_navigation_signed(subframe3, dt_LS_1));              //[s]
            cnav1_utc_model.t_ot = static_cast<double>(read_navigation_unsigned(subframe3, t_ot_1)) * TWO_P4;     //[s] effective range 0~604784
            cnav1_utc_model.WN_ot = static_cast<double>(read_navigation_unsigned(subframe3, WN_ot_1));            //[week]
            cnav1_utc_model.WN_LSF = static_cast<double>(read_navigation_unsigned(subframe3, WN_LSF_1));          //[week]
            cnav1_utc_model.DN = static_cast<double>(read_navigation_unsigned(subframe3, DN_1));                  //[day] effective range 0~6
            cnav1_utc_model.dt_LSF = static_cast<double>(read_navigation_signed(subframe3, dt_LSF_1));            //[s]
            // BDT-UTC Time Offset Parameters End
            
            cnav1_ephemeris.Rev = static_cast<double>(read_navigation_unsigned(subframe3, Rev_1));
            

            // Set flags relative to time and message
            flag_TOW_set = true;
            flag_TOW_1 = true;
            flag_ephemeris_mes_type_1 = true;
            flag_iono_mes_type_1 = true;
            flag_utc_model_mes_type_1 = true;    

            break;

        case 2:
            //--- It is Type 2 of Subframe 3-----------------------------------------------
            //cnav1_ephemeris.PageId = static_cast<double>(read_navigation_unsigned(subframe3, PageId));
            cnav1_ephemeris.HS = static_cast<double>(read_navigation_unsigned(subframe3, HS_2));
            cnav1_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(subframe3, DIF_2));
            cnav1_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(subframe3, SIF_2));
            cnav1_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(subframe3, AIF_2));
            cnav1_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(subframe3, SISMAI_2));
            
            // SISAI_OC (22 bits)
            cnav1_ephemeris.t_op = static_cast<double>(read_navigation_unsigned(subframe3, t_op_2));
            cnav1_ephemeris.SISAI_ocb = static_cast<double>(read_navigation_unsigned(subframe3, SISAI_ocb_2));
            cnav1_ephemeris.SISAI_oc1 = static_cast<double>(read_navigation_unsigned(subframe3, SISAI_oc1_2));
            cnav1_ephemeris.SISAI_oc2 = static_cast<double>(read_navigation_unsigned(subframe3, SISAI_oc2_2));
            // SISAI_OC End
            
            // Reduced Almanac Parameters Sat 1(38 bits)
            i_alm_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(subframe3, PRN_a1_2));  //[dimensionless] effective range 1~63
            cnav1_almanac[i_alm_satellite_PRN - 1].i_satellite_PRN = i_alm_satellite_PRN;
            cnav1_almanac[i_alm_satellite_PRN - 1].i_BDS_week = static_cast<int32_t>(read_navigation_unsigned(subframe3, WN_a_2));                           //[week] effective range 0~8191
            cnav1_almanac[i_alm_satellite_PRN - 1].t_oa = static_cast<double>(read_navigation_unsigned(subframe3, t_oa_2)) * TWO_P12;                        //[s] effective range 0~602112
            cnav1_almanac[i_alm_satellite_PRN - 1].SatType = static_cast<double>(read_navigation_unsigned(subframe3, SatType1_2));                           //[dimensionless] binary, 01:GEO, 10:IGSO, 11:MEO, 00:Reserved
            cnav1_almanac[i_alm_satellite_PRN - 1].delta_A = static_cast<double>(read_navigation_signed(subframe3, delta_A1_2)) * TWO_P9;                    //[m] reference MEO: 27906100m, IGSO/GEO: 42162200m
            cnav1_almanac[i_alm_satellite_PRN - 1].Omega_0 = static_cast<double>(read_navigation_signed(subframe3, Omega_01_2)) * BEIDOU_CNAV1_PI * TWO_N6;  //[pi]
            cnav1_almanac[i_alm_satellite_PRN - 1].Phi_0 = static_cast<double>(read_navigation_signed(subframe3, Phi_01_2)) * BEIDOU_CNAV1_PI * TWO_N6;      //[pi] Phi = M0 + omega, e=0, delta_i=0, MEO/IGSO i=55, GEO i=0
            cnav1_almanac[i_alm_satellite_PRN - 1].Health = static_cast<double>(read_navigation_unsigned(subframe3, Health1_2));                             //[dimensionless]
            // Reduced Almanac Parameters End

            // Reduced Almanac Parameters Sat 2(38 bits)
            i_alm_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(subframe3, PRN_a2_2));  //[dimensionless] effective range 1~63
            cnav1_almanac[i_alm_satellite_PRN - 1].i_satellite_PRN = i_alm_satellite_PRN;
            cnav1_almanac[i_alm_satellite_PRN - 1].i_BDS_week = static_cast<int32_t>(read_navigation_unsigned(subframe3, WN_a_2));                           //[week] effective range 0~8191
            cnav1_almanac[i_alm_satellite_PRN - 1].t_oa = static_cast<double>(read_navigation_unsigned(subframe3, t_oa_2)) * TWO_P12;                        //[s] effective range 0~602112
            cnav1_almanac[i_alm_satellite_PRN - 1].SatType = static_cast<double>(read_navigation_unsigned(subframe3, SatType2_2));                           //[dimensionless] binary, 01:GEO, 10:IGSO, 11:MEO, 00:Reserved
            cnav1_almanac[i_alm_satellite_PRN - 1].delta_A = static_cast<double>(read_navigation_signed(subframe3, delta_A2_2)) * TWO_P9;                    //[m] reference MEO: 27906100m, IGSO/GEO: 42162200m
            cnav1_almanac[i_alm_satellite_PRN - 1].Omega_0 = static_cast<double>(read_navigation_signed(subframe3, Omega_02_2)) * BEIDOU_CNAV1_PI * TWO_N6;  //[pi]
            cnav1_almanac[i_alm_satellite_PRN - 1].Phi_0 = static_cast<double>(read_navigation_signed(subframe3, Phi_02_2)) * BEIDOU_CNAV1_PI * TWO_N6;      //[pi] Phi = M0 + omega, e=0, delta_i=0, MEO/IGSO i=55, GEO i=0
            cnav1_almanac[i_alm_satellite_PRN - 1].Health = static_cast<double>(read_navigation_unsigned(subframe3, Health2_2));                             //[dimensionless]
            // Reduced Almanac Parameters End

            // Reduced Almanac Parameters Sat 3(38 bits)
            i_alm_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(subframe3, PRN_a3_2));  //[dimensionless] effective range 1~63
            cnav1_almanac[i_alm_satellite_PRN - 1].i_satellite_PRN = i_alm_satellite_PRN;
            cnav1_almanac[i_alm_satellite_PRN - 1].i_BDS_week = static_cast<int32_t>(read_navigation_unsigned(subframe3, WN_a_2));                           //[week] effective range 0~8191
            cnav1_almanac[i_alm_satellite_PRN - 1].t_oa = static_cast<double>(read_navigation_unsigned(subframe3, t_oa_2)) * TWO_P12;                        //[s] effective range 0~602112
            cnav1_almanac[i_alm_satellite_PRN - 1].SatType = static_cast<double>(read_navigation_unsigned(subframe3, SatType3_2));                           //[dimensionless] binary, 01:GEO, 10:IGSO, 11:MEO, 00:Reserved
            cnav1_almanac[i_alm_satellite_PRN - 1].delta_A = static_cast<double>(read_navigation_signed(subframe3, delta_A3_2)) * TWO_P9;                    //[m] reference MEO: 27906100m, IGSO/GEO: 42162200m
            cnav1_almanac[i_alm_satellite_PRN - 1].Omega_0 = static_cast<double>(read_navigation_signed(subframe3, Omega_03_2)) * BEIDOU_CNAV1_PI * TWO_N6;  //[pi]
            cnav1_almanac[i_alm_satellite_PRN - 1].Phi_0 = static_cast<double>(read_navigation_signed(subframe3, Phi_03_2)) * BEIDOU_CNAV1_PI * TWO_N6;      //[pi] Phi = M0 + omega, e=0, delta_i=0, MEO/IGSO i=55, GEO i=0
            cnav1_almanac[i_alm_satellite_PRN - 1].Health = static_cast<double>(read_navigation_unsigned(subframe3, Health3_2));                             //[dimensionless]

            // Reduced Almanac Parameters End
            
            // Reduced Almanac Parameters Sat 4(38 bits)
            i_alm_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(subframe3, PRN_a4_2));  //[dimensionless] effective range 1~63
            cnav1_almanac[i_alm_satellite_PRN - 1].i_satellite_PRN = i_alm_satellite_PRN;
            cnav1_almanac[i_alm_satellite_PRN - 1].i_BDS_week = static_cast<int32_t>(read_navigation_unsigned(subframe3, WN_a_2));                           //[week] effective range 0~8191
            cnav1_almanac[i_alm_satellite_PRN - 1].t_oa = static_cast<double>(read_navigation_unsigned(subframe3, t_oa_2)) * TWO_P12;                        //[s] effective range 0~602112
            cnav1_almanac[i_alm_satellite_PRN - 1].SatType = static_cast<double>(read_navigation_unsigned(subframe3, SatType4_2));                           //[dimensionless] binary, 01:GEO, 10:IGSO, 11:MEO, 00:Reserved
            cnav1_almanac[i_alm_satellite_PRN - 1].delta_A = static_cast<double>(read_navigation_signed(subframe3, delta_A4_2)) * TWO_P9;                    //[m] reference MEO: 27906100m, IGSO/GEO: 42162200m
            cnav1_almanac[i_alm_satellite_PRN - 1].Omega_0 = static_cast<double>(read_navigation_signed(subframe3, Omega_04_2)) * BEIDOU_CNAV1_PI * TWO_N6;  //[pi]
            cnav1_almanac[i_alm_satellite_PRN - 1].Phi_0 = static_cast<double>(read_navigation_signed(subframe3, Phi_04_2)) * BEIDOU_CNAV1_PI * TWO_N6;      //[pi] Phi = M0 + omega, e=0, delta_i=0, MEO/IGSO i=55, GEO i=0
            cnav1_almanac[i_alm_satellite_PRN - 1].Health = static_cast<double>(read_navigation_unsigned(subframe3, Health4_2));                             //[dimensionless]

            // Reduced Almanac Parameters End
            
            cnav1_ephemeris.Rev = static_cast<double>(read_navigation_unsigned(subframe3, Rev_2));
            
            flag_TOW_set = true;
            flag_TOW_2 = true;
            flag_ephemeris_mes_type_2 = true;
            flag_almanac_mes_type_2 = true;
            
            break;

        case 3:
            // --- It is Type 3 of Subframe 3 ----------------------------------------------
            //cnav1_ephemeris.PageId = static_cast<double>(read_navigation_unsigned(subframe3, PageId));
            cnav1_ephemeris.HS = static_cast<double>(read_navigation_unsigned(subframe3, HS_3));
            cnav1_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(subframe3, DIF_3));
            cnav1_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(subframe3, SIF_3));
            cnav1_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(subframe3, AIF_3));
            cnav1_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(subframe3, SISMAI_3));
            cnav1_ephemeris.SISAI_OE = static_cast<double>(read_navigation_unsigned(subframe3, SISAI_oe_3));
            
            // EOP Parameters (138 bits)
            cnav1_ephemeris.t_EOP = static_cast<double>(read_navigation_unsigned(subframe3, t_EOP_3)) * TWO_P4;       //[s] effective range 0~604784
            cnav1_ephemeris.PM_X = static_cast<double>(read_navigation_signed(subframe3, PM_X_3)) * TWO_N20;          //[arc-seconds]
            cnav1_ephemeris.PM_X_dot = static_cast<double>(read_navigation_signed(subframe3, PM_X_dot_3)) * TWO_N21;  //[arc-seconds/day]
            cnav1_ephemeris.PM_Y = static_cast<double>(read_navigation_signed(subframe3, PM_Y_3)) * TWO_N20;          //[arc-seconds]
            cnav1_ephemeris.PM_Y_dot = static_cast<double>(read_navigation_signed(subframe3, PM_Y_dot_3)) * TWO_N21;  //[arc-seconds/day]
            cnav1_ephemeris.dUT1 = static_cast<double>(read_navigation_signed(subframe3, dUT1_3)) * TWO_N24;          //[s]
            cnav1_ephemeris.dUT1_dot = static_cast<double>(read_navigation_signed(subframe3, dUT1_dot_3)) * TWO_N25;  //[s/day]

            // EOP Parameters End
            
            // BGTO Parameters (68 bits)
            cnav1_utc_model.GNSS_ID = static_cast<double>(read_navigation_unsigned(subframe3, GNSS_ID_3));            //[dimensionless]
            cnav1_utc_model.WN_0BGTO = static_cast<double>(read_navigation_unsigned(subframe3, WN_0BGTO_3));          //[week]
            cnav1_utc_model.t_0BGTO = static_cast<double>(read_navigation_unsigned(subframe3, t_0BGTO_3)) * TWO_P4;   //[s] effective range 0~604784
            cnav1_utc_model.A_0BGTO = static_cast<double>(read_navigation_unsigned(subframe3, A_0BGTO_3)) * TWO_N35;  //[s]
            cnav1_utc_model.A_1BGTO = static_cast<double>(read_navigation_unsigned(subframe3, A_1BGTO_3)) * TWO_N51;  //[s/s]
            cnav1_utc_model.A_2BGTO = static_cast<double>(read_navigation_unsigned(subframe3, A_2BGTO_3)) * TWO_N68;  //[s/s^2]
            
            // BGTO Parameters End
            
            cnav1_ephemeris.Rev = static_cast<double>(read_navigation_unsigned(subframe3, Rev_3));
            
            flag_TOW_set = true;
            flag_TOW_3 = true;
            flag_ephemeris_mes_type_3 = true;
            flag_utc_model_mes_type_3 = true;
            
            break;

        case 4:
            // --- It is Type 4 of Subframe 3 ----------------------------------------------
            //cnav1_ephemeris.PageId = static_cast<double>(read_navigation_unsigned(subframe3, PageId));
            cnav1_ephemeris.HS = static_cast<double>(read_navigation_unsigned(subframe3, HS_4));
            cnav1_ephemeris.DIF = static_cast<double>(read_navigation_unsigned(subframe3, DIF_4));
            cnav1_ephemeris.SIF = static_cast<double>(read_navigation_unsigned(subframe3, SIF_4));
            cnav1_ephemeris.AIF = static_cast<double>(read_navigation_unsigned(subframe3, AIF_4));
            cnav1_ephemeris.SISMAI = static_cast<double>(read_navigation_unsigned(subframe3, SISMAI_4));
            
            // SISAI_OC (22 bits)
            cnav1_ephemeris.t_op = static_cast<double>(read_navigation_unsigned(subframe3, t_op_4));
            cnav1_ephemeris.SISAI_ocb = static_cast<double>(read_navigation_unsigned(subframe3, SISAI_ocb_4));
            cnav1_ephemeris.SISAI_oc1 = static_cast<double>(read_navigation_unsigned(subframe3, SISAI_oc1_4));
            cnav1_ephemeris.SISAI_oc2 = static_cast<double>(read_navigation_unsigned(subframe3, SISAI_oc2_4));
            // SISAI_OC End
            
            // Midi Almanac Parameters (156 bits)
            i_alm_satellite_PRN = static_cast<uint32_t>(read_navigation_unsigned(subframe3, PRN_a_4));  //[dimensionless] effective range 1~63
            cnav1_almanac[i_alm_satellite_PRN - 1].i_satellite_PRN = i_alm_satellite_PRN;
            cnav1_almanac[i_alm_satellite_PRN - 1].SatType = static_cast<double>(read_navigation_unsigned(subframe3, SatType_4));                                //[dimensionless] binary, 01:GEO, 10:IGSO, 11:MEO, 00:Reserved
            cnav1_almanac[i_alm_satellite_PRN - 1].i_BDS_week = static_cast<int32_t>(read_navigation_unsigned(subframe3, WN_a_4));                               //[week] effective range 0~8191
            cnav1_almanac[i_alm_satellite_PRN - 1].t_oa = static_cast<double>(read_navigation_unsigned(subframe3, t_oa_4)) * TWO_P12;                            //[s] effective range 0~602112
            cnav1_almanac[i_alm_satellite_PRN - 1].e = static_cast<double>(read_navigation_unsigned(subframe3, e_4)) * TWO_N16;                                  //[dimensionless]
            cnav1_almanac[i_alm_satellite_PRN - 1].delta_i = static_cast<double>(read_navigation_signed(subframe3, delta_i_4)) * BEIDOU_cnav1_PI * TWO_N14;      //[pi]
            cnav1_almanac[i_alm_satellite_PRN - 1].sqrt_A = static_cast<double>(read_navigation_unsigned(subframe3, sqrt_A_4)) * TWO_N4;                         //[m^0.5]
            cnav1_almanac[i_alm_satellite_PRN - 1].Omega_0 = static_cast<double>(read_navigation_signed(subframe3, Omega_0_4)) * BEIDOU_cnav1_PI * TWO_N15;      //[pi]
            cnav1_almanac[i_alm_satellite_PRN - 1].Omega_dot = static_cast<double>(read_navigation_signed(subframe3, Omega_dot_4)) * BEIDOU_cnav1_PI * TWO_N33;  //[pi/s]
            cnav1_almanac[i_alm_satellite_PRN - 1].omega = static_cast<double>(read_navigation_signed(subframe3, omega_4)) * BEIDOU_cnav1_PI * TWO_N15;          //[pi]
            cnav1_almanac[i_alm_satellite_PRN - 1].M_0 = static_cast<double>(read_navigation_signed(subframe3, M_0_4)) * BEIDOU_cnav1_PI * TWO_N15;              //[pi]
            cnav1_almanac[i_alm_satellite_PRN - 1].a_f0 = static_cast<double>(read_navigation_signed(subframe3, a_f0_4)) * TWO_N20;                              //[s]
            cnav1_almanac[i_alm_satellite_PRN - 1].a_f1 = static_cast<double>(read_navigation_signed(subframe3, a_f1_4)) * TWO_N37;                              //[s/s]
            cnav1_almanac[i_alm_satellite_PRN - 1].Health = static_cast<double>(read_navigation_unsigned(subframe3, Health_4));                                  //[dimensionless] 8th(MSB):Satellite, 7th:B1C, 6th:B2a, 5th~1st:reserve, 0:normal/health, 1:abnormal
            // Midi Almanac Parameters End
            
            cnav1_ephemeris.Rev = static_cast<double>(read_navigation_unsigned(subframe3, Rev_4));
            
            flag_TOW_set = true;
            flag_TOW_4 = true;
            flag_ephemeris_mes_type_4 = true;
            flag_almanac_mes_type_4 = true;
            
            break;


        default:
            LOG(INFO) << "BEIDOU CNAV1: Invalid String ID of received. Received " << i_frame_mes_type
                      << ", but acceptable values are 1, 2, 3 and 4";

            break;
        }  // switch string ID

    return i_frame_mes_type;
}

Beidou_Cnav1_Ephemeris Beidou_Cnav1_Navigation_Message::get_ephemeris()
{
    return cnav1_ephemeris;
}

Beidou_Cnav1_Utc_Model Beidou_Cnav1_Navigation_Message::get_utc_model()
{
    return cnav1_utc_model;
}

Beidou_Cnav1_Iono Beidou_Cnav1_Navigation_Message::get_iono()
{
    return cnav1_iono;
}

Beidou_Cnav1_Almanac Beidou_Cnav1_Navigation_Message::get_almanac(uint32_t satellite_slot_number)
{
    return cnav1_almanac[satellite_slot_number - 1];
}

bool Beidou_Cnav1_Navigation_Message::have_new_ephemeris()  //Check if we have a new ephemeris stored in the galileo navigation class
{
    bool new_eph = false;
    // We need to make sure we have received the ephemeris info plus the time info
    if (flag_ephemeris_mes_SF2 == true)
        {
            if (d_previous_tb != cnav1_ephemeris.IODE)
                {
                    flag_ephemeris_mes_SF2 = false;  // clear the flag
                    flag_all_ephemeris = true;
                    // Update the time of ephemeris information
                    d_previous_tb = cnav1_ephemeris.IODE;
                    DLOG(INFO) << "BeiDou Cnav1 Ephemeris from Subframe 2 have been received and belong to the same batch" << std::endl;
                    new_eph = true;
                }
        }

    return new_eph;
}

bool Beidou_Cnav1_Navigation_Message::have_new_utc_model()  // Check if we have a new utc data set stored in the beidou navigation class
{
    if ((flag_utc_model_mes_type_1 == true) and (flag_utc_model_mes_type_3 == true))
        {
            flag_utc_model_mes_type_1 = false;  // clear the flag
            flag_utc_model_mes_type_3 = false;  // clear the flag
            return true;
        }
    else
        return false;
}

bool Beidou_Cnav1_Navigation_Message::have_new_iono()  // Check if we have a new iono data set stored in the beidou navigation class
{
    if (flag_iono_mes_type_1 == true)
        {
            flag_iono_mes_type_1 = false;  // clear the flag
            return true;
        }
    else
        return false;
}

bool Beidou_Cnav1_Navigation_Message::have_new_almanac()  //Check if we have a new almanac data set stored in the beidou navigation class
{
    if (flag_almanac_mes_type_2 == true)
        {
            if (d_previous_Na[i_alm_satellite_PRN] != cnav1_almanac[i_alm_satellite_PRN - 1].t_oa)
                {
                    //All almanac have been received for this satellite
                    flag_almanac_mes_type_2 = false;
                    return true;
                }
        }
    if (flag_almanac_mes_type_4 == true)
        {
            if (d_previous_Na[i_alm_satellite_PRN] != cnav1_almanac[i_alm_satellite_PRN - 1].t_oa)
                {
                    //All almanac have been received for this satellite
                    flag_almanac_mes_type_4 = false;
                    return true;
                }
        }
    return false;
}
