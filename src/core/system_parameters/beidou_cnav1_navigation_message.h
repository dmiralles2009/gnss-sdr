/*!
 * \file beidou_cnav1_navigation_message.h
 * \brief  Interface of a BEIDOU CNAV1 Data message decoder as described in BEIDOU B1C ICD
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


#ifndef GNSS_SDR_BEIDOU_CNAV1_NAVIGATION_MESSAGE_H_
#define GNSS_SDR_BEIDOU_CNAV1_NAVIGATION_MESSAGE_H_


#include "Beidou_B1C.h"
#include "beidou_cnav1_almanac.h"
#include "beidou_cnav1_ephemeris.h"
#include "beidou_cnav1_iono.h"
#include "beidou_cnav1_utc_model.h"
#include <bitset>
#include <cstdint>


/*!
 * \brief This class decodes a BEIDOU cnav1 Data message as described in BEIDOU ICD
 */
class Beidou_Cnav1_Navigation_Message
{
public:
    /*!
     * Default constructor
     */
    Beidou_Cnav1_Navigation_Message();

    bool flag_crc_test;
    bool flag_crc_test_sf_2;         				//!< Flag indicating CRC test for Subframe 2
    bool flag_crc_test_sf_3;         				//!< Flag indicating CRC test for Subframe 3
    uint32_t i_frame_mes_type;  				//!< Flag indicating MesType
    int32_t i_alm_satellite_PRN;
    int32_t i_channel_ID;      					//!< PRN of the channel
    uint32_t i_satellite_PRN;  					//!< Satellite PRN

    Beidou_Cnav1_Ephemeris cnav1_ephemeris;                     //!< Ephemeris information decoded
    Beidou_Cnav1_Utc_Model cnav1_utc_model;                     //!< UTC model information
    Beidou_Cnav1_Iono cnav1_iono;                               //!< UTC model information
    Beidou_Cnav1_Almanac cnav1_almanac[BEIDOU_B1C_NBR_SATS];  //!< Almanac information for all 63 satellites

    // Ephemeris Flags and control variables
    bool flag_all_ephemeris;          //!< Flag indicating that all strings containing ephemeris have been received
    bool flag_ephemeris_mes_SF1;      //!< Flag indicating that Subframe 1 have been received
    bool flag_ephemeris_mes_SF2;      //!< Flag indicating that both ephemeris(Subframe 2) have been received
    bool flag_ephemeris_mes_type_1;  //!< Flag indicating that (Type 1) of Subframe 3 have been received
    bool flag_ephemeris_mes_type_2;  //!< Flag indicating that (Type 2) of Subframe 3 have been received
    bool flag_ephemeris_mes_type_3;  //!< Flag indicating that (Type 3) of Subframe 3 have been received
    bool flag_ephemeris_mes_type_4;  //!< Flag indicating that (Type 4) of Subframe 3 have been received

    // Almanac Flags
    bool flag_almanac_mes_type_2;  //!< Flag indicating that almanac of Type 2 of Subframe 3 have been received
    bool flag_almanac_mes_type_4;  //!< Flag indicating that almanac of Type 4 of Subframe 3 have been received

    // UTC and System Clocks Flags
    bool flag_utc_model_valid;        //!< If set, it indicates that the UTC model parameters are filled
    bool flag_utc_model_mes_type_1;  //!< If set, it indicates that the UTC model parameters of Type 1 have been received
    bool flag_utc_model_mes_type_3;  //!< If set, it indicates that the UTC model parameters of Type 3 have been received

    // Iono Flgas
    bool flag_iono_valid;        //!< If set, it indicates that the UTC model parameters are filled
    bool flag_iono_mes_type_1;  //!< If set, it indicates that the UTC model parameters of Type 32 have been received


    bool flag_TOW_set;  //!< Flag indicating when the TOW has been set
    bool flag_TOW_new;  //!< Flag indicating when a new TOW has been computed
    bool flag_TOW_SF_1;
    bool flag_TOW_SF_2;
    bool flag_TOW_1;
    bool flag_TOW_2;
    bool flag_TOW_3;
    bool flag_TOW_4;
   

    double d_satClkCorr;   //!<  Satellite clock error
    double d_dtr;          //!<  Relativistic clock correction term
    double d_satClkDrift;  //!<  Satellite clock drift

    double d_previous_tb;                         //!< Previous iode for the Beidou_Cnav1_Ephemeris object. Used to determine when new data arrives
    double d_previous_Na[BEIDOU_B1C_NBR_SATS];  //!< Previous time for almanac of the Beidou_Cnav1_Almanac object

    double temp;  //!< Temporary value
    template<long unsigned int subframe_size>   //to use geeneralized size according to each subframe size in "read_navigation_bool" and "read_navigation_unsigned" functions 

    double crc_compute;  //!< Value of the computed CRC

    /*!
     * \brief Compute CRC for BEIDOU CNAV1 Subframe 2 strings
     * \param bits Bits of the string message where to compute CRC
     */
    bool crc_test_for_subframe_2(std::bitset<BEIDOU_CNAV1_SUBFRAME_2_BITS> bits, uint32_t crc_decoded);
    
    /*!
     * \brief Compute CRC for BEIDOU CNAV1 Subframe 3 strings
     * \param bits Bits of the string message where to compute CRC
     */
    bool crc_test_for_subframe_3(std::bitset<BEIDOU_CNAV1_SUBFRAME_3_BITS> bits, uint32_t crc_decoded);

    /*!
     * \brief Computes the frame number being decoded given the satellite slot number
     * \param satellite_slot_number [in] Satellite slot number identifier
     * \returns Frame number being decoded, 0 if operation was not successful.
     */
    uint32_t get_frame_number(uint32_t satellite_slot_number);

    /*!
     * \brief Reset BEIDOU CNAV1 Navigation Information
     */
    void reset();

    /*!
     * \brief Obtain a BEIDOU CNAV1 SV Ephemeris class filled with current SV data
     */
    Beidou_Cnav1_Ephemeris get_ephemeris();

    /*!
     * \brief Obtain a BEIDOU CNAV1 UTC model parameters class filled with current SV data
     */
    Beidou_Cnav1_Utc_Model get_utc_model();

    /*!
     * \brief Obtain a BEIDOU CNAV1 Iono model parameters class filled with current SV data
     */
    Beidou_Cnav1_Iono get_iono();

    /*!
     * \brief Returns a Beidou_Cnav1_Almanac object filled with the latest navigation data received
     * \param satellite_slot_number Slot number identifier for the satellite
     * \returns Returns the Beidou_Cnav1_Almanac object for the input slot number
     */
    Beidou_Cnav1_Almanac get_almanac(uint32_t satellite_slot_number);

    /*!
     * \brief Returns true if a new Beidou_Cnav1_Ephemeris object has arrived.
     */
    bool have_new_ephemeris();

    /*!
     * \brief Returns true if new Beidou_Cnav1_Utc_Model object has arrived
     */
    bool have_new_utc_model();

    /*!
     * \brief Returns true if new Beidou_Cnav1_Iono object has arrived
     */
    bool have_new_iono();

    /*!
     * \brief Returns true if new Beidou_Cnav1_Almanac object has arrived.
     */
    bool have_new_almanac();

    /*!
     * \brief Decodes the BEIDOU CNAV1 string
     * \param frame_string [in] is the string message within the parsed frame
     * \returns Returns the ID of the decoded string
     */
    int32_t frame_decoder(std::string const &frame_string);

private:
    uint64_t read_navigation_unsigned(std::bitset<subframe_size> const &bits, const std::vector<std::pair<int32_t, int32_t>> &parameter);
    int64_t read_navigation_signed(std::bitset<subframe_size> const &bits, const std::vector<std::pair<int32_t, int32_t>> &parameter);
    bool read_navigation_bool(std::bitset<subframe_size> const &bits, const std::vector<std::pair<int32_t, int32_t>> &parameter);
};

#endif
