/*!
 * \file beidou_cnav2_navigation_message.h
 * \brief  Interface of a BEIDOU CNAV2 Data message decoder as described in BEIDOU ICD
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


#ifndef GNSS_SDR_BEIDOU_CNAV2_NAVIGATION_MESSAGE_H_
#define GNSS_SDR_BEIDOU_CNAV2_NAVIGATION_MESSAGE_H_


#include "beidou_cnav2_ephemeris.h"
#include "beidou_cnav2_almanac.h"
#include "beidou_cnav2_utc_model.h"
#include "BEIDOU_B2a_CA.h"
#include <bitset>


/*!
 * \brief This class decodes a BEIDOU cnav2 Data message as described in BEIDOU ICD
 * \note Code added as part of GSoC 2018 program
 * \see <a href="http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf">beidou ICD</a>
 */
class Beidou_Cnav2_Navigation_Message
{
private:
    unsigned long int read_navigation_unsigned(std::bitset<BEIDOU_CNAV2_STRING_BITS> bits, const std::vector<std::pair<int, int>> parameter);
    signed long int read_navigation_signed(std::bitset<BEIDOU_CNAV2_STRING_BITS> bits, const std::vector<std::pair<int, int>> parameter);
    bool read_navigation_bool(std::bitset<BEIDOU_CNAV2_STRING_BITS> bits, const std::vector<std::pair<int, int>> parameter);

public:
    bool flag_CRC_test;
    unsigned int d_frame_ID;
    unsigned int d_string_ID;
    bool flag_update_slot_number;

    int i_channel_ID;
    unsigned int i_satellite_PRN;

    Beidou_Cnav2_Ephemeris cnav2_ephemeris;                   //!< Ephemeris information decoded
    Beidou_Cnav2_Utc_Model cnav2_utc_model;                   //!< UTC model information
    Beidou_Cnav2_Almanac cnav2_almanac[BEIDOU_NBR_SATS];  //!< Almanac information for all 63 satellites

    // Ephemeris Flags and control variables
    bool flag_all_ephemeris;    //!< Flag indicating that all strings containing ephemeris have been received
    bool flag_ephemeris_str_1;  //!< Flag indicating that ephemeris 1/2 (Type 10) have been received
    bool flag_ephemeris_str_2;  //!< Flag indicating that ephemeris 2/2 (Type 11) have been received

    // Almanac Flags
    bool flag_almanac_str_31;                   //!< Flag indicating that almanac of Type 31 have been received

    unsigned int i_alm_satellite_slot_number;  //!< SV Orbit Slot Number

    // UTC and System Clocks Flags
    bool flag_utc_model_valid;   //!< If set, it indicates that the UTC model parameters are filled
    bool flag_utc_model_str_5;   //!< Clock info send in string 5 of navigation data
    bool flag_utc_model_str_15;  //!< Clock info send in string 15 of frame 5 of navigation data

    bool flag_TOW_set;  //!< Flag indicating when the TOW has been set
    bool flag_TOW_new;  //!< Flag indicating when a new TOW has been computed

    double d_satClkCorr;   //!<  Satellite clock error
    double d_dtr;          //!<  Relativistic clock correction term
    double d_satClkDrift;  //!<  Satellite clock drift

    double d_previous_tb;                       //!< Previous iode for the Beidou_Cnav2_Ephemeris object. Used to determine when new data arrives
    double d_previous_Na[BEIDOU_NBR_SATS];  //!< Previous time for almanac of the Beidou_Cnav2_Almanac object

    unsigned int d_string_ID;

    double temp;

    /*!
     * \brief Compute CRC for BEIDOU CNAV2 strings
     * \param bits Bits of the string message where to compute CRC
     */
    bool CRC_test(std::bitset<BEIDOU_CNAV2_STRING_BITS> bits,unsigned int d_string_ID);

    /*!
     * \brief Computes the frame number being decoded given the satellite slot number
     * \param satellite_slot_number [in] Satellite slot number identifier
     * \returns Frame number being decoded, 0 if operation was not successful.
     */
    unsigned int get_frame_number(unsigned int satellite_slot_number);

    /*!
     * \brief Reset BEIDOU CNAV2 Navigation Information
     */
    void reset();

    /*!
     * \brief Obtain a BEIDOU CNAV2 SV Ephemeris class filled with current SV data
     */
    Beidou_Cnav2_Ephemeris get_ephemeris();

    /*!
     * \brief Obtain a BEIDOU CNAV2 UTC model parameters class filled with current SV data
     */
    Beidou_Cnav2_Utc_Model get_utc_model();

    /*!
     * \brief Returns a Beidou_Cnav2_Almanac object filled with the latest navigation data received
     * \param satellite_slot_number Slot number identifier for the satellite
     * \returns Returns the Beidou_Cnav2_Almanac object for the input slot number
     */
    Beidou_Cnav2_Almanac get_almanac(unsigned int satellite_slot_number);

    /*!
     * \brief Returns true if a new Beidou_Cnav2_Ephemeris object has arrived.
     */
    bool have_new_ephemeris();

    /*!
     * \brief Returns true if new Beidou_Cnav2_Utc_Model object has arrived
     */
    bool have_new_utc_model();

    /*!
     * \brief Returns true if new Beidou_Cnav2_Almanac object has arrived.
     */
    bool have_new_almanac();

    /*!
     * \brief Decodes the BEIDOU CNAV2 string
     * \param frame_string [in] is the string message within the parsed frame
     * \returns Returns the ID of the decoded string
     */
    int string_decoder(std::string frame_string);

    /*!
     * Default constructor
     */
    Beidou_Cnav2_Navigation_Message();
};

#endif
