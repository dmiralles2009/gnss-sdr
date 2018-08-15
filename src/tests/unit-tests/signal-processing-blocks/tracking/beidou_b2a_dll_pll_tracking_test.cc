/*!
 * \file beidou_b2a_dll_pll_tracking_test.cc
 * \brief  This class implements a tracking test for Beidou_B2a_DLL_PLL_Tracking
 *  implementation based on some input parameters.
 * \author Sara Hrbek, 2018 sara.hrbek(at)gmail.com. Code added as part of GSoC 2018 program
 *
 *
 * -------------------------------------------------------------------------
 *
 * Copyright (C) 2012-2018  (see AUTHORS file for a list of contributors)
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
#include <gnuradio/top_block.h>
#include <gnuradio/blocks/file_source.h>
#include <gnuradio/analog/sig_source_waveform.h>
#include <gnuradio/analog/sig_source_c.h>
#include <gnuradio/msg_queue.h>
#include <gnuradio/blocks/null_sink.h>
#include <gnuradio/blocks/skiphead.h>
#include <gtest/gtest.h>
#include "gnss_block_factory.h"
#include "gnss_block_interface.h"
#include "tracking_interface.h"
#include "in_memory_configuration.h"
#include "gnss_sdr_valve.h"
#include "gnss_synchro.h"
#include "dll_pll_veml_tracking.h"
#include "beidou_b2a_dll_pll_tracking.h"


// ######## GNURADIO BLOCK MESSAGE RECEVER #########
class BeidouB2aDllPllTrackingTest_msg_rx;

typedef boost::shared_ptr<BeidouB2aDllPllTrackingTest_msg_rx> BeidouB2aDllPllTrackingTest_msg_rx_sptr;

BeidouB2aDllPllTrackingTest_msg_rx_sptr BeidouB2aDllPllTrackingTest_msg_rx_make();

class BeidouB2aDllPllTrackingTest_msg_rx : public gr::block
{
private:
    friend BeidouB2aDllPllTrackingTest_msg_rx_sptr BeidouB2aDllPllTrackingTest_msg_rx_make();
    void msg_handler_events(pmt::pmt_t msg);
    BeidouB2aDllPllTrackingTest_msg_rx();

public:
    int rx_message;
    ~BeidouB2aDllPllTrackingTest_msg_rx();  //!< Default destructor
};


BeidouB2aDllPllTrackingTest_msg_rx_sptr BeidouB2aDllPllTrackingTest_msg_rx_make()
{
    return BeidouB2aDllPllTrackingTest_msg_rx_sptr(new BeidouB2aDllPllTrackingTest_msg_rx());
}


void BeidouB2aDllPllTrackingTest_msg_rx::msg_handler_events(pmt::pmt_t msg)
{
    try
        {
            long int message = pmt::to_long(msg);
            rx_message = message;
        }
    catch (boost::bad_any_cast& e)
        {
            LOG(WARNING) << "msg_handler_telemetry Bad any cast!";
            rx_message = 0;
        }
}


BeidouB2aDllPllTrackingTest_msg_rx::BeidouB2aDllPllTrackingTest_msg_rx() : gr::block("BeidouB2aDllPllTrackingTest_msg_rx", gr::io_signature::make(0, 0, 0), gr::io_signature::make(0, 0, 0))
{
    this->message_port_register_in(pmt::mp("events"));
    this->set_msg_handler(pmt::mp("events"), boost::bind(&BeidouB2aDllPllTrackingTest_msg_rx::msg_handler_events, this, _1));
    rx_message = 0;
}


BeidouB2aDllPllTrackingTest_msg_rx::~BeidouB2aDllPllTrackingTest_msg_rx()
{
}


// ###########################################################

class BeidouB2aDllPllTrackingTest : public ::testing::Test
{
protected:
    BeidouB2aDllPllTrackingTest()
    {
        factory = std::make_shared<GNSSBlockFactory>();
        config = std::make_shared<InMemoryConfiguration>();
        item_size = sizeof(gr_complex);
        gnss_synchro = Gnss_Synchro();
    }

    ~BeidouB2aDllPllTrackingTest()
    {
    }

    void init();

    gr::msg_queue::sptr queue;
    gr::top_block_sptr top_block;
    std::shared_ptr<GNSSBlockFactory> factory;
    std::shared_ptr<InMemoryConfiguration> config;
    Gnss_Synchro gnss_synchro;
    size_t item_size;
};


void BeidouB2aDllPllTrackingTest::init()
{
    gnss_synchro.Channel_ID = 0;
    gnss_synchro.System = 'C';
    std::string signal = "5C";
    signal.copy(gnss_synchro.Signal, 2, 0);
    gnss_synchro.PRN = 27;

    config->set_property("GNSS-SDR.internal_fs_sps", "25000000");
    config->set_property("Tracking_5C.item_type", "gr_complex");
    config->set_property("Tracking_5C.dump", "false");
    config->set_property("Tracking_5C.dump_filename", "/media/sf_VMShare/VMShare/B2a_UT_tracking_ch_");
    config->set_property("Tracking_5C.implementation", "Beidou_B2a_DLL_PLL_Tracking");
    config->set_property("Tracking_5C.early_late_space_chips", "0.5");
    config->set_property("Tracking_5C.order", "2");
    config->set_property("Tracking_5C.pll_bw_hz", "2");
    config->set_property("Tracking_5C.dll_bw_hz", "25");
}


TEST_F(BeidouB2aDllPllTrackingTest, ValidationOfResults)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds(0.0);
    int fs_in = 25000000;
    int nsamples = fs_in * 1;

    init();
    queue = gr::msg_queue::make(0);
    top_block = gr::make_top_block("Tracking test");
    std::shared_ptr<TrackingInterface> tracking = std::make_shared<BeidouB2aDllPllTracking>(config.get(), "Tracking_5C", 1, 1);
    boost::shared_ptr<BeidouB2aDllPllTrackingTest_msg_rx> msg_rx = BeidouB2aDllPllTrackingTest_msg_rx_make();
    std::cout << "We got to here 0 " << std::endl;
    gnss_synchro.Acq_delay_samples = 18719;
    gnss_synchro.Acq_doppler_hz = 400;
    gnss_synchro.Acq_samplestamp_samples = 0;//todo adjust.
    ASSERT_NO_THROW({
        tracking->set_channel(gnss_synchro.Channel_ID);
    }) << "Failure setting channel.";
    ASSERT_NO_THROW({
        tracking->set_gnss_synchro(&gnss_synchro);
    }) << "Failure setting gnss_synchro.";
    ASSERT_NO_THROW({
        tracking->connect(top_block);
    }) << "Failure connecting tracking to the top_block.";

    ASSERT_NO_THROW({
        //gr::analog::sig_source_c::sptr source = gr::analog::sig_source_c::make(fs_in, gr::analog::GR_SIN_WAVE, 1000, 1, gr_complex(0));
        std::string path = std::string(TEST_PATH);
        std::string file = path + "signal_samples/USRP_BDS_B2a_201805171115_fs_25e6_if0e3_ishort_200ms.bin";
        const char* file_name = file.c_str();
        gr::blocks::file_source::sptr file_source = gr::blocks::file_source::make(sizeof(gr_complex), file_name, false);
        boost::shared_ptr<gr::block> valve = gnss_sdr_make_valve(sizeof(gr_complex), nsamples, queue);
        gr::blocks::null_sink::sptr sink = gr::blocks::null_sink::make(sizeof(Gnss_Synchro));
        top_block->connect(file_source, 0, valve, 0);
        top_block->connect(valve, 0, tracking->get_left_block(), 0);
        top_block->connect(tracking->get_right_block(), 0, sink, 0);
        top_block->msg_connect(tracking->get_right_block(), pmt::mp("events"), msg_rx, pmt::mp("events"));
    }) << "Failure connecting the blocks of tracking test.";

    tracking->start_tracking();
   //todo this is where the test stops
    EXPECT_NO_THROW({
        start = std::chrono::system_clock::now();
        top_block->run();  // Start threads and wait
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
    }) << "Failure running the top_block.";


    // TODO: Verify tracking results
    std::cout << "Tracked " << nsamples << " samples in " << elapsed_seconds.count() * 1e6 << " microseconds" << std::endl;
}
