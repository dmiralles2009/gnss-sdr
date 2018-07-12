/*!
 * \file beidou_b2a_telemetry_decoder.h
 * \brief Interface of an adapter of a BEIDOU B2a CNAV2 data decoder block
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


#ifndef GNSS_SDR_BEIDOU_B2A_TELEMETRY_DECODER_H_
#define GNSS_SDR_BEIDOU_B2A_TELEMETRY_DECODER_H_

#include "telemetry_decoder_interface.h"
#include "beidou_b2a_telemetry_decoder_cc.h"
#include <string>

class ConfigurationInterface;

/*!
 * \brief This class implements a NAV data decoder for BEIDOU B2a
 */
class BeidouB2aTelemetryDecoder : public TelemetryDecoderInterface
{
public:
    BeidouB2aTelemetryDecoder(ConfigurationInterface* configuration,
        std::string role,
        unsigned int in_streams,
        unsigned int out_streams);

    virtual ~BeidouB2aTelemetryDecoder();
    std::string role() override
    {
        return role_;
    }

    //! Returns "BEIDOU_B2a_Telemetry_Decoder"
    std::string implementation() override
    {
        return "BEIDOU_B2a_Telemetry_Decoder";
    }
    void connect(gr::top_block_sptr top_block) override;
    void disconnect(gr::top_block_sptr top_block) override;
    gr::basic_block_sptr get_left_block() override;
    gr::basic_block_sptr get_right_block() override;
    void set_satellite(const Gnss_Satellite& satellite) override;
    void set_channel(int channel) override { telemetry_decoder_->set_channel(channel); }
    void reset() override
    {
        return;
    }
    size_t item_size() override
    {
        return 0;
    }

private:
    beidou_b2a_telemetry_decoder_cc_sptr telemetry_decoder_;
    Gnss_Satellite satellite_;
    int channel_;
    bool dump_;
    std::string dump_filename_;
    std::string role_;
    unsigned int in_streams_;
    unsigned int out_streams_;
};

#endif
