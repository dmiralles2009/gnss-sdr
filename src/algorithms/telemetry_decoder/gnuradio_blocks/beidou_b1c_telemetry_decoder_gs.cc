/*!
 * \file beidou_b1c_telemetry_decoder_gs.cc
 * \brief Implementation of a BeiDou B1C CNAV1 data decoder block
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


#include "beidou_b1c_telemetry_decoder_gs.h"
#include "Beidou_B1C.h"
#include "beidou_cnav1_almanac.h"
#include "beidou_cnav1_ephemeris.h"
#include "beidou_cnav1_iono.h"
#include "beidou_cnav1_utc_model.h"
#include "display.h"
#include "gnss_synchro.h"
#include <glog/logging.h>
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>        // for make_any
#include <pmt/pmt_sugar.h>  // for mp
#include <cstdlib>          // for abs
#include <exception>        // for exception
#include <iostream>         // for cout
#include <memory>           // for shared_ptr, make_shared


#define CRC_ERROR_LIMIT 8


beidou_b1c_telemetry_decoder_gs_sptr
beidou_b1c_make_telemetry_decoder_gs(const Gnss_Satellite &satellite, bool dump)
{
    return beidou_b1c_telemetry_decoder_gs_sptr(new beidou_b1c_telemetry_decoder_gs(satellite, dump));
}


beidou_b1c_telemetry_decoder_gs::beidou_b1c_telemetry_decoder_gs(
    const Gnss_Satellite &satellite,
    bool dump) : gr::block("beidou_b1c_telemetry_decoder_gs",
                     gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)),
                     gr::io_signature::make(1, 1, sizeof(Gnss_Synchro)))
{
    //prevent telemetry symbols accumulation in output buffers
    this->set_max_noutput_items(1);
    // Ephemeris data port out
    this->message_port_register_out(pmt::mp("telemetry"));
    // Control messages to tracking block
    this->message_port_register_out(pmt::mp("telemetry_to_trk"));
    // initialize internal vars
    d_dump = dump;
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    LOG(INFO) << "Initializing BeiDou B1C Telemetry Decoding for satellite " << this->d_satellite;

    d_symbol_duration_ms = BEIDOU_CNAV1_TELEMETRY_SYMBOLS_PER_BIT * BEIDOU_B1Cd_CODE_PERIOD_MS;
    d_required_symbols = BEIDOU_CNAV1_FRAME_SYMBOLS;
    d_symbol_history.set_capacity(d_required_symbols);

    d_sent_tlm_failed_msg = false;
    d_flag_valid_word = false;
    // Generic settings
    d_sample_counter = 0;
    d_flag_frame_sync = false;
    d_TOW_at_current_symbol_ms = 0U;
    d_TOW_at_Frame_ms = 0U;
    Flag_valid_word = false;
    d_CRC_error_counter = 0;
    d_channel = 0;
    flag_TOW_set = false;
}


beidou_b1c_telemetry_decoder_gs::~beidou_b1c_telemetry_decoder_gs()
{
    if (d_dump_file.is_open() == true)
        {
            try
                {
                    d_dump_file.close();
                }
            catch (const std::exception &ex)
                {
                    LOG(WARNING) << "Exception in destructor closing the dump file " << ex.what();
                }
        }
}


void beidou_b1c_telemetry_decoder_gs::decode_frame(float *frame_symbols)
{
    // 1. Transform from symbols to bits
    std::string data_bits;

    // we want data_bits = frame_symbols[24:24+288]
    for (uint32_t ii = 0; ii < (BEIDOU_CNAV1_DATA_BITS); ii++)
        {
            data_bits.push_back((frame_symbols[ii] > 0) ? ('1') : ('0'));
        }

    d_nav.frame_decoder(data_bits);

    // 2. Check operation executed correctly
    if (d_nav.flag_crc_test == true)
        {
            DLOG(INFO) << "BeiDou CNAV1 CRC correct in channel " << d_channel
                       << " from satellite " << d_satellite;
        }
    else
        {
            DLOG(INFO) << "BeiDou CNAV1 CRC error in channel " << d_channel
                       << " from satellite " << d_satellite;
        }
    // 3. Push the new navigation data to the queues
    if (d_nav.have_new_ephemeris() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Beidou_Cnav1_Ephemeris> tmp_obj = std::make_shared<Beidou_Cnav1_Ephemeris>(d_nav.get_ephemeris());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            DLOG(INFO) << "BeiDou CNAV1 Ephemeris have been received in channel" << d_channel << " from satellite " << d_satellite;
            std::cout << TEXT_YELLOW << "New BeiDou B1C CNAV1 message received in channel " << d_channel << ": ephemeris from satellite " << d_satellite << TEXT_RESET << std::endl;
        }
    if (d_nav.have_new_utc_model() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Beidou_Cnav1_Utc_Model> tmp_obj = std::make_shared<Beidou_Cnav1_Utc_Model>(d_nav.get_utc_model());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            DLOG(INFO) << "BeiDou CNAV1 UTC Model have been received in channel" << d_channel << " from satellite " << d_satellite;
            std::cout << TEXT_YELLOW << "New BeiDou B1C CNAV1 UTC model message received in channel " << d_channel << ": UTC model parameters from satellite " << d_satellite << TEXT_RESET << std::endl;
        }
    if (d_nav.have_new_iono() == true)
        {
            // get object for this SV (mandatory)
            std::shared_ptr<Beidou_Cnav1_Iono> tmp_obj = std::make_shared<Beidou_Cnav1_Iono>(d_nav.get_iono());
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            DLOG(INFO) << "BeiDou CNAV1 Iono have been received in channel" << d_channel << " from satellite " << d_satellite;
            std::cout << TEXT_YELLOW << "New BeiDou B1C CNAV1 Iono message received in channel " << d_channel << ": UTC model parameters from satellite " << d_satellite << TEXT_RESET << std::endl;
        }
    if (d_nav.have_new_almanac() == true)
        {
            unsigned int slot_nbr = d_nav.i_alm_satellite_PRN;
            std::shared_ptr<Beidou_Cnav1_Almanac> tmp_obj = std::make_shared<Beidou_Cnav1_Almanac>(d_nav.get_almanac(slot_nbr));
            this->message_port_pub(pmt::mp("telemetry"), pmt::make_any(tmp_obj));
            DLOG(INFO) << "BeiDou CNAV1 Almanac have been received in channel" << d_channel << " in slot number " << slot_nbr;
            std::cout << TEXT_YELLOW << "New BeiDou B1C CNAV1 almanac received in channel " << d_channel << " from satellite " << d_satellite << TEXT_RESET << std::endl;
        }
}


void beidou_b1c_telemetry_decoder_gs::set_satellite(const Gnss_Satellite &satellite)
{
    d_satellite = Gnss_Satellite(satellite.get_system(), satellite.get_PRN());
    DLOG(INFO) << "Setting decoder Finite State Machine to satellite " << d_satellite;
    DLOG(INFO) << "Navigation Satellite set to " << d_satellite;
}


void beidou_b1c_telemetry_decoder_gs::set_channel(int32_t channel)
{
    d_channel = channel;
    LOG(INFO) << "Navigation channel set to " << channel;
    // ############# ENABLE DATA FILE LOG #################
    if (d_dump == true)
        {
            if (d_dump_file.is_open() == false)
                {
                    try
                        {
                            d_dump_filename = "telemetry";
                            d_dump_filename.append(boost::lexical_cast<std::string>(d_channel));
                            d_dump_filename.append(".dat");
                            d_dump_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
                            d_dump_file.open(d_dump_filename.c_str(), std::ios::out | std::ios::binary);
                            LOG(INFO) << "Telemetry decoder dump enabled on channel " << d_channel << " Log file: " << d_dump_filename.c_str();
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            LOG(WARNING) << "channel " << d_channel << ": exception opening Beidou TLM dump file. " << e.what();
                        }
                }
        }
}


void beidou_b1c_telemetry_decoder_gs::reset()
{
    d_TOW_at_current_symbol_ms = 0;
    d_sent_tlm_failed_msg = false;
    d_flag_valid_word = false;
    DLOG(INFO) << "BeiDou B1C Telemetry decoder reset for satellite " << d_satellite;
    return;
}


int beidou_b1c_telemetry_decoder_gs::general_work(int noutput_items __attribute__((unused)), gr_vector_int &ninput_items __attribute__((unused)),
    gr_vector_const_void_star &input_items, gr_vector_void_star &output_items)
{

    auto **out = reinterpret_cast<Gnss_Synchro **>(&output_items[0]);            // Get the output buffer pointer
    const auto **in = reinterpret_cast<const Gnss_Synchro **>(&input_items[0]);  // Get the input buffer pointer

    Gnss_Synchro current_symbol;  //structure to save the synchronization information and send the output object to the next block
    //1. Copy the current tracking output
    current_symbol = in[0][0];
    d_symbol_history.push_back(current_symbol.Prompt_I);  //add new symbol to the symbol queue
    d_sample_counter++;                                   //count for the processed samples
    consume_each(1);



    // UPDATE GNSS SYNCHRO DATA
    //1. Add the telemetry decoder information
    if (d_nav.flag_TOW_set == true)
        
        {
            //As System Time Parameter "SOH" present in only Subframe 1,hence checked for Subframe 1
            if (d_nav.flag_TOW_SF_1 == true)
                {
                    d_TOW_at_Frame_ms = static_cast<uint32_t>((d_nav.cnav1_ephemeris.SOH + BEIDOU_CNAV1_BDT2GPST_LEAP_SEC_OFFSET) * 1000.0);
                    flag_TOW_set = true;
                    d_nav.flag_TOW_SF_1 = false;
                }

            uint32_t last_d_TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
            d_TOW_at_current_symbol_ms =d_TOW_at_Frame_ms + d_required_symbols * d_symbol_duration_ms;

            if (last_d_TOW_at_current_symbol_ms != 0 and abs(static_cast<int64_t>(d_TOW_at_current_symbol_ms) - int64_t(last_d_TOW_at_current_symbol_ms)) > d_symbol_duration_ms)
                {
                    LOG(INFO) << "Warning: BeiDou B1C CNAV1 TOW update in ch " << d_channel
                              << " does not match the TLM TOW counter " << static_cast<int64_t>(d_TOW_at_current_symbol_ms) - int64_t(last_d_TOW_at_current_symbol_ms) << " ms \n";

                    d_TOW_at_current_symbol_ms = 0;
                    d_flag_valid_word = false;
                }
            else
                {
                    d_flag_valid_word = true;
                }
        }
    else  //if there is not a new preamble, we define the TOW of the current symbol
        {
            if (d_flag_valid_word)
                {
                    d_TOW_at_current_symbol_ms += d_symbol_duration_ms;
                    if (current_symbol.Flag_valid_symbol_output == false)
                        {
                            d_flag_valid_word = false;
                        }
                }
        }

    if (d_flag_valid_word == true)
        {
            current_symbol.TOW_at_current_symbol_ms = d_TOW_at_current_symbol_ms;
            current_symbol.Flag_valid_word = d_flag_valid_word;

            if (d_dump == true)
                {
                    // MULTIPLEXED FILE RECORDING - Record results to file
                    try
                        {
                            double tmp_double;
                            uint64_t tmp_ulong_int;
                            tmp_double = static_cast<double>(d_TOW_at_current_symbol_ms) / 1000.0;
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                            tmp_ulong_int = current_symbol.Tracking_sample_counter;
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_ulong_int), sizeof(uint64_t));
                            tmp_double = static_cast<double>(d_TOW_at_Frame_ms) / 1000.0;
                            d_dump_file.write(reinterpret_cast<char *>(&tmp_double), sizeof(double));
                        }
                    catch (const std::ifstream::failure &e)
                        {
                            LOG(WARNING) << "Exception writing Telemetry BeiDou B1C dump file " << e.what();
                        }
                }

            // 2. Make the output (copy the object contents to the GNURadio reserved memory)
            *out[0] = current_symbol;
            return 1;
        }
    return 0;
}
