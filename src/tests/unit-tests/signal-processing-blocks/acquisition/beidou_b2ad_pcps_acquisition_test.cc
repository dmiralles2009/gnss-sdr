/*!
 * \file beidou_b2ad_pcps_acquisition_test.cc
 * \brief  Tests a PCPS acquisition block for Beidou B2a signals
 * \author Sara Hrbek, 2018. sara.hrbek(at)gmail.com gsoc 2018
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

#include <chrono>
#include <cstdlib>
#include <boost/chrono.hpp>
#include <boost/make_shared.hpp>
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include <gtest/gtest.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "beidou_b2ad_pcps_acquisition.h"


// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class BeidouB2adPcpsAcquisitionTest_msg_rx;

typedef boost::shared_ptr<BeidouB2adPcpsAcquisitionTest_msg_rx> BeidouB2adPcpsAcquisitionTest_msg_rx_sptr;

BeidouB2adPcpsAcquisitionTest_msg_rx_sptr BeidouB2adPcpsAcquisitionTest_msg_rx_make();

class BeidouB2adPcpsAcquisitionTest_msg_rx : public gr::block
{
private:
    friend BeidouB2adPcpsAcquisitionTest_msg_rx_sptr BeidouB2adPcpsAcquisitionTest_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    BeidouB2adPcpsAcquisitionTest_msg_rx();

public:
    int rx_message;
    ~BeidouB2adPcpsAcquisitionTest_msg_rx();  //!< Default destructor
};


BeidouB2adPcpsAcquisitionTest_msg_rx_sptr BeidouB2adPcpsAcquisitionTest_msg_rx_make()
{
    return BeidouB2adPcpsAcquisitionTest_msg_rx_sptr(new BeidouB2adPcpsAcquisitionTest_msg_rx());
}


void BeidouB2adPcpsAcquisitionTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            long int message = pmt::to_long(msg);
            rx_message = message;
        }
    catch (boost::bad_any_cast& e)
        {
            std::cout << "msg_handler_telemetry Bad any cast!" << std::endl;
            rx_message = 0;
        }
}


BeidouB2adPcpsAcquisitionTest_msg_rx::BeidouB2adPcpsAcquisitionTest_msg_rx() : gr::block("BeidouB2adPcpsAcquisitionTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&BeidouB2adPcpsAcquisitionTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


BeidouB2adPcpsAcquisitionTest_msg_rx::~BeidouB2adPcpsAcquisitionTest_msg_rx()
{
}


// ###########################################################

class BeidouB2adPcpsAcquisitionTest : public ::testing::Test
{
protected:
	BeidouB2adPcpsAcquisitionTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~BeidouB2adPcpsAcquisitionTest()
    {
    }

    void init();

    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
};


void BeidouB2adPcpsAcquisitionTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'C';
    std::string signal = "5C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 27;
    config->set_property("GNSS-SDR.internal_fs_sps", "25000000");
    config->set_property("Acquisition_5C.item_type", "gr_complex");
    config->set_property("Acquisition_5C.if", "0");
    config->set_property("Acquisition_5C.coherent_integration_time_ms", "5");
    config->set_property("Acquisition_5C.dump", "true");
    config->set_property("Acquisition_5C.dump_filename", "/media/sf_VMShare/VMShare/B2aUT_acquisition");
    config->set_property("Acquisition_5C.implementation", "Beidou_B2ad_PCPS_Acquisition");
    config->set_property("Acquisition_5C.pfa", "0.01");
    config->set_property("Acquisition_5C.doppler_max", "10000");
    config->set_property("Acquisition_5C.doppler_step", "50");
    config->set_property("Acquisition_5C.bit_transition_flag", "true");
    config->set_property("Acquisition_5C.repeat_satellite", "true");

}


TEST_F(BeidouB2adPcpsAcquisitionTest, Instantiate)
{
    init();
    boost::shared_ptr<BeidouB2adPcpsAcquisition> acquisition = boost::make_shared<BeidouB2adPcpsAcquisition>(config.get(), "Acquisition_1G", 1, 1);
}


TEST_F(BeidouB2adPcpsAcquisitionTest, ConnectAndRun)
{
    int fs_in = 25000000;
    int nsamples = 25000000*0.2;
    std::chrono::time_point<std::chrono::system_clock> begin, end;
    std::chrono::duration<double> elapsed_seconds(0);
    gr::msg_queue::sptr queue = gr::msg_queue::make(0);

    top_block = gr::make_top_block("Acquisition test");
    init();
    boost::shared_ptr<BeidouB2adPcpsAcquisition> acquisition = boost::make_shared<BeidouB2adPcpsAcquisition>(config.get(), "Acquisition_5C", 1, 1);
    boost::shared_ptr<BeidouB2adPcpsAcquisitionTest_msg_rx> msg_rx = BeidouB2adPcpsAcquisitionTest_msg_rx_make();

    ASSERT_NO_THROW({
        acquisition->connect(top_block);
        boost::shared_ptr<gr::analog::sig_source_c> source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        top_block->connect(source, 0, valve, 0);
        top_block->connect(valve, 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of acquisition test.";

    EXPECT_NO_THROW({
        begin = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - begin;
    }) << "Failure running the top_block.";

    std::cout << "Processed " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}


TEST_F(BeidouB2adPcpsAcquisitionTest, ValidationOfResults)
{
    std::chrono::time_point<std::chrono::system_clock> begin, end;
    std::chrono::duration<double> elapsed_seconds(0);
    top_block = gr::make_top_block("Acquisition test");

    double expected_delay_samples = 18719;//todo adjust
    double expected_doppler_hz = 400;//todo adjust
    init();
    std::shared_ptr<BeidouB2adPcpsAcquisition> acquisition = std::make_shared<BeidouB2adPcpsAcquisition>(config.get(), "Acquisition_5C", 1, 1);

    boost::shared_ptr<BeidouB2adPcpsAcquisitionTest_msg_rx> msg_rx = BeidouB2adPcpsAcquisitionTest_msg_rx_make();
    ASSERT_NO_THROW({
        acquisition->set_channel(1);
    }) << "Failure setting channel.";

    ASSERT_NO_THROW({
        acquisition->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro.";

    ASSERT_NO_THROW({
        acquisition->set_doppler_max(10000);
    }) << "Failure setting doppler_max.";

    ASSERT_NO_THROW({
        acquisition->set_doppler_step(50);
    }) << "Failure setting doppler_step.";

    ASSERT_NO_THROW({
        acquisition->connect(top_block);
    }) << "Failure connecting acquisition to the top_block.";

    acquisition->set_local_code();

    acquisition->set_state(1);  // Ensure that acquisition starts at the first sample

    acquisition->init();

    ASSERT_NO_THROW({
        std::string path = std::string(TEST_PATH);
        std::string file = path + "signal_samples/USRP_BDS_B2a_201805171115_fs_25e6_if0e3_ishort_200ms.bin";
        const char* file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
        top_block->connect(file_source, 0, acquisition->get_left_block(), 0);
        top_block->msg_connect(acquisition->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of acquisition test.";

    EXPECT_NO_THROW({
        begin = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - begin;
    }) << "Failure running the top_block.";
    unsigned long int nsamples = gnss_synchro.Acq_samplestamp_samples;
    std::cout << "Acquired " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;

    ASSERT_EQ(1, msg_rx->rx_message) << "Acquisition failure. Expected message: 1=ACQ SUCCESS.";

    double delay_error_samples = std::abs(expected_delay_samples - gnss_synchro.Acq_delay_samples);
    float delay_error_chips = static_cast<float>(delay_error_samples) * 10230 / 25000;
    double doppler_error_hz = std::abs(expected_doppler_hz - gnss_synchro.Acq_doppler_hz);

    EXPECT_LE(doppler_error_hz, 133.3) << "Doppler error exceeds the expected value: 133.3 Hz = 2/(3*integration period)";
    EXPECT_LT(delay_error_chips, 0.5) << "Delay error exceeds the expected value: 0.5 chips";
}
