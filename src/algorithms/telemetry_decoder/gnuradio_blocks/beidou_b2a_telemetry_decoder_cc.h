/*!
 * \file beidou_b2a_telemetry_decoder_cc.h
 * \brief Implementation of an adapter of a BEIDOU B2a CNAV2 data decoder block
 * to a TelemetryDecoderInterface
 * \note Code added as part of GSoC 2018 program
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

#ifndef GNSS_SDR_BEIDOU_B2A_TELEMETRY_DECODER_CC_H
#define GNSS_SDR_BEIDOU_B2A_TELEMETRY_DECODER_CC_H


#include "beidou_cnav2_navigation_message.h"
#include "beidou_cnav2_ephemeris.h"
#include "beidou_cnav2_almanac.h"
#include "beidou_cnav2_utc_model.h"
#include "gnss_satellite.h"
#include "gnss_synchro.h"
#include "BEIDOU_B2a_CA.h"
#include <gnuradio/block.h>
#include <fstream>
#include <string>


class beidou_b2a_telemetry_decoder_cc;

typedef boost::shared_ptr<beidou_b2a_telemetry_decoder_cc> beidou_b2a_telemetry_decoder_cc_sptr;

beidou_b2a_telemetry_decoder_cc_sptr beidou_b2a_make_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump);

//!!!! edit
/*!
 * \brief This class implements a block that decodes the GNAV data defined in BEIDOU ICD v5.1
 * \note Code added as part of GSoC 2018 program
 * \see <a href="http://russianspacesystems.ru/wp-content/uploads/2016/08/ICD_GLONASS_eng_v5.1.pdf">GLONASS ICD</a>
 *
 */
class beidou_b2a_telemetry_decoder_cc : public gr::block
{
public:
    ~beidou_b2a_telemetry_decoder_cc();                //!< Class destructor
    void set_satellite(const Gnss_Satellite &satellite);  //!< Set satellite PRN
    void set_channel(int channel);                        //!< Set receiver's channel

    /*!
     * \brief This is where all signal processing takes place
     */
    int general_work(int noutput_items, gr_vector_int &ninput_items,
        gr_vector_const_void_star &input_items, gr_vector_void_star &output_items);

private:
    friend beidou_b2a_telemetry_decoder_cc_sptr
	beidou_b2a_make_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump);
    beidou_b2a_telemetry_decoder_cc(const Gnss_Satellite &satellite, bool dump);

    void decode_string(double *symbols, int frame_length);

    //!< Help with coherent tracking
    double d_preamble_time_samples;

    //!< Preamble decoding
    unsigned short int d_preambles_bits[BEIDOU_CNAV2_PREAMBLE_LENGTH_BITS];
    int *d_preambles_symbols;
    unsigned int d_samples_per_symbol;
    int d_symbols_per_preamble;

    //!< Storage for incoming data
    std::deque<Gnss_Synchro> d_symbol_history;

    //!< Variables for internal functionality
    long unsigned int d_sample_counter;  //!< Sample counter as an index (1,2,3,..etc) indicating number of samples processed
    long unsigned int d_preamble_index;  //!< Index of sample number where preamble was found
    unsigned int d_stat;                 //!< Status of decoder
    bool d_flag_frame_sync;              //!< Indicate when a frame sync is achieved
    bool d_flag_parity;                  //!< Flag indicating when parity check was achieved (crc check)
    bool d_flag_preamble;                //!< Flag indicating when preamble was found
    int d_CRC_error_counter;             //!< Number of failed CRC operations
    bool flag_TOW_set;                   //!< Indicates when time of week is set
    double delta_t;                      //!< GPS-GLONASS time offset !!! check

    //!< Navigation Message variable
    Beidou_Cnav2_Navigation_Message d_nav;

    //!< Values to populate gnss synchronization structure
    double d_TOW_at_current_symbol;
    bool Flag_valid_word;

    //!< Satellite Information and logging capacity
    Gnss_Satellite d_satellite;
    int d_channel;
    bool d_dump;
    std::string d_dump_filename;
    std::ofstream d_dump_file;
};

#endif
