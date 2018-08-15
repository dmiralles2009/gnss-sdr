/*!
 * \file beidou_b2a_signal._processingcc
 * \brief This class implements signal generators for the BEIDOU B2a signals
 * \author Sara Hrbek, 2018. sara.hrbek(at)gmail.com. Code added as part of GSoC 2018 program
 *
 * Detailed description of the file here if needed.
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

#include "beidou_b2a_signal_processing.h"

#include <cinttypes>
#include <cmath>
#include <complex>
#include <deque>
#include "BEIDOU_B2A.h"
#include <iostream>
#include <fstream>



std::deque<bool> b2ad_g1_shift(std::deque<bool> g1)
{
	// Polynomial: G1 = 1 + x + x^5 + x^11 + x^13;
	std::deque<bool> out(g1.begin(), g1.end() - 1);
	out.push_front(g1[12] xor g1[10] xor g1[4] xor g1[0]);
	return out;

}


std::deque<bool> b2ap_g1_shift(std::deque<bool> g1)
{
	// Polynomial: G1 = 1 + x^3 + x^6 + x^7 + x^13;
	std::deque<bool> out(g1.begin(), g1.end() - 1);
	out.push_front(g1[12] xor g1[6] xor g1[5] xor g1[2]);
	return out;
}


std::deque<bool> b2ad_g2_shift(std::deque<bool> g2)
{
	// Polynomial: G2 = 1 + x^3 + x^5 + x^9 + x^11 +x^12 +x^13;
    std::deque<bool> out(g2.begin(), g2.end() - 1);
    out.push_front(g2[12] xor g2[11] xor g2[10] xor g2[8] xor g2[4] xor g2[2]);
    return out;
}


std::deque<bool> b2ap_g2_shift(std::deque<bool> g2)
{
	//Polynomial: G2 = 1 + x + x^5 + x^7 gsco+ x^8 +x^12 +x^13;
    std::deque<bool> out(g2.begin(), g2.end() - 1);
    out.push_front(g2[12] xor g2[11] xor g2[7] xor g2[6] xor g2[4] xor g2[0]);
    return out;
}

// Make the B2a data G1 sequence
std::deque<bool> make_b2ad_g1()
{
    std::deque<bool> g1 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    std::deque<bool> g1_seq(BEIDOU_B2ad_CODE_LENGTH_CHIPS, 0);

    for (int i = 0; i < BEIDOU_B2ad_CODE_LENGTH_CHIPS; i++)
        {
            g1_seq[i] = g1[12];
            g1 = b2ad_g1_shift(g1);
            // reset the g1 register
            if (i==8189)
            {
            	g1 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
            }
        }
    return g1_seq;
}

// Make the B2a data G2 sequence.
std::deque<bool> make_b2ad_g2(std::deque<bool> g2)
{
    std::deque<bool> g2_seq(BEIDOU_B2ad_CODE_LENGTH_CHIPS, 0);

    for (int i = 0; i < BEIDOU_B2ad_CODE_LENGTH_CHIPS; i++)
        {
            g2_seq[i] = g2[12];
            g2 = b2ad_g2_shift(g2);
        }
    return g2_seq;
}

// Make the B2a pilot G1 sequence
std::deque<bool> make_b2ap_g1()
{
    std::deque<bool> g1 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    std::deque<bool> g1_seq(BEIDOU_B2ap_CODE_LENGTH_CHIPS, 0);

    for (int i = 0; i < BEIDOU_B2ap_CODE_LENGTH_CHIPS; i++)
        {
            g1_seq[i] = g1[12];
            g1 = b2ap_g1_shift(g1);

			//reset the g1 register
			if (i==8189)
			{
				g1 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
			}
        }

    return g1_seq;
}

// Make the B2a pilot G2 sequence
std::deque<bool> make_b2ap_g2(std::deque<bool> g2)
{
    std::deque<bool> g2_seq(BEIDOU_B2ap_CODE_LENGTH_CHIPS, 0);

    for (int i = 0; i < BEIDOU_B2ap_CODE_LENGTH_CHIPS; i++)
        {
            g2_seq[i] = g2[12];
            g2 = b2ap_g2_shift(g2);
        }
    return g2_seq;
}

// Generate the B2a data PRN codes
void make_b2ad(int32_t* _dest, int prn)
{
	std::deque<bool> g2 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
	for (int i = 0;i < 13; i++)
		{
			g2[i] = BEIDOU_B2ad_INIT_REG[prn-1][i];

		}
    std::deque<bool> g1 = make_b2ad_g1();
    g2 = make_b2ad_g2(g2);

    std::deque<bool> out_code(BEIDOU_B2ad_CODE_LENGTH_CHIPS, 0);

    for (int n = 0; n < BEIDOU_B2ad_CODE_LENGTH_CHIPS; n++)
        {
            _dest[n] = g1[n] xor g2[n];
        }
}

// Generate a version of the B2a Data code with the secondary code included
void make_b2adSecondary(int32_t* _dest, int prn)
{
	// This is specific to the Beidou B2a data code
	bool Secondary1 = 0;
	bool Secondary2 = 1;
	std::deque<bool> g2 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
	for (int i = 0;i < 13; i++)
		{
			g2[i] = BEIDOU_B2ad_INIT_REG[prn-1][i];

		}
    std::deque<bool> g1 = make_b2ad_g1();
    g2 = make_b2ad_g2(g2);

    std::deque<bool> out_code(BEIDOU_B2ad_CODE_LENGTH_CHIPS*BEIDOU_B2ad_SECONDARY_CODE_LENGTH, 0);

	for(int m = 0;m <BEIDOU_B2ad_SECONDARY_CODE_LENGTH;m++){
		for (int n = 0; n < BEIDOU_B2ad_CODE_LENGTH_CHIPS; n++)
			{
			if (m==3){
				_dest[n+m*BEIDOU_B2ad_CODE_LENGTH_CHIPS] = (g1[n] xor g2[n]) xor Secondary2;
			}else{
				_dest[n+m*BEIDOU_B2ad_CODE_LENGTH_CHIPS] = (g1[n] xor g2[n]) xor Secondary1;
			}
			}
		}
}

// Generate the B2a pilot code
void make_b2ap(int32_t* _dest, int prn)
{

	std::deque<bool> g2 = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
	for (int i = 0;i < 13; i++)
		{
			g2[i] = BEIDOU_B2ap_INIT_REG[prn][i];
		}
	    std::deque<bool> g1 = make_b2ap_g1();
	    g2 = make_b2ap_g2(g2);

	    std::deque<bool> out_code(BEIDOU_B2ap_CODE_LENGTH_CHIPS, 0);
	    for (int n = 0; n < BEIDOU_B2ap_CODE_LENGTH_CHIPS; n++)
	        {
	            _dest[n] = g1[n] xor g2[n];
	        }
}


// Generate a complex version of the B2a data code with the Secondary code
void beidou_b2ad_code_gen_complexSecondary(std::complex<float>* _dest, unsigned int _prn)
{
    int32_t* _code = new int32_t[BEIDOU_B2ad_CODE_LENGTH_CHIPS*BEIDOU_B2ad_SECONDARY_CODE_LENGTH];

    if (_prn > 0 and _prn < 63)
        {
            make_b2adSecondary(_code, _prn );
        }

    for (signed int i = 0; i < BEIDOU_B2ad_CODE_LENGTH_CHIPS*BEIDOU_B2ad_SECONDARY_CODE_LENGTH; i++)
        {
            _dest[i] = std::complex<float>(1.0 - 2.0 * _code[i], 0.0);
        }

    delete[] _code;
}

// Generate a complex version of the B2a data code
void beidou_b2ad_code_gen_complex(std::complex<float>* _dest, unsigned int _prn)
{
    int32_t* _code = new int32_t[BEIDOU_B2ad_CODE_LENGTH_CHIPS];

    if (_prn > 0 and _prn < 63)
        {
            make_b2ad(_code, _prn );
        }

    for (signed int i = 0; i < BEIDOU_B2ad_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = std::complex<float>(1.0 - 2.0 * _code[i], 0.0);
        }

    delete[] _code;
}

// Generate a float version of the B2a data code
void beidou_b2ad_code_gen_float(float* _dest, unsigned int _prn)
{
    int32_t* _code = new int32_t[BEIDOU_B2ad_CODE_LENGTH_CHIPS];

    if (_prn > 0 and _prn < 63)
        {
            make_b2ad(_code, _prn );
        }

    for (signed int i = 0; i < BEIDOU_B2ad_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = 1.0 - 2.0 * static_cast<float>(_code[i]);
        }

    delete[] _code;
}

/*
 *  Generates complex BEIDOU B2a data code for the desired SV ID and sampled to specific sampling frequency
 */
void beidou_b2ad_code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, signed int _fs)
{
    int32_t* _code = new int32_t[BEIDOU_B2ad_CODE_LENGTH_CHIPS];
    if (_prn > 0 and _prn <= 63)
        {
            make_b2ad(_code, _prn);
        }

    signed int _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const signed int _codeLength = BEIDOU_B2ad_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B2ad_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                   // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B2ad_CODE_RATE_HZ);  // code chip period in sec

    for (signed int i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B2a code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B2ad code -----------------------
            if (i == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_codeLength - 1], 0);
                }
            else
                {
                    _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_codeValueIndex], 0);  //repeat the chip -> upsample
                }
        }
    delete[] _code;
}

/*
 *  Generates complex BEIDOU B2a data code for the desired SV ID and sampled to specific sampling frequency with the secondary code implemented
 */
void beidou_b2ad_code_gen_complex_sampledSecondary(std::complex<float>* _dest, unsigned int _prn, signed int _fs)
{
    int32_t* _code = new int32_t[BEIDOU_B2ad_CODE_LENGTH_CHIPS*BEIDOU_B2ad_SECONDARY_CODE_LENGTH];
    if (_prn > 0 and _prn <= 63)
        {
            make_b2adSecondary(_code, _prn);
        }

    signed int _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const signed int _codeLength = BEIDOU_B2ad_CODE_LENGTH_CHIPS*BEIDOU_B2ad_SECONDARY_CODE_LENGTH;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B2ad_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                   // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B2ad_CODE_RATE_HZ);  // code chip period in sec

    for (signed int i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B2a code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B2ad code -----------------------
            if (i == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_codeLength - 1], 0);
                }
            else
                {
                    _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_codeValueIndex], 0);  //repeat the chip -> upsample
                }

        }
    delete[] _code;
}

// Generates a complex version of the B2a pilot code
void beidou_b2ap_code_gen_complex(std::complex<float>* _dest, unsigned int _prn)
{
    int32_t* _code = new int32_t[BEIDOU_B2ap_CODE_LENGTH_CHIPS];

    if (_prn > 0 and _prn < 63)
        {
            make_b2ap(_code, _prn);
        }

    for (signed int i = 0; i < BEIDOU_B2ap_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = std::complex<float>(1.0 - 2.0 * _code[i], 0.0);
        }

    delete[] _code;
}

// Generates a float version of the B2a pilot code
void beidou_b2ap_code_gen_float(float* _dest, unsigned int _prn)
{
    int32_t* _code = new int32_t[BEIDOU_B2ap_CODE_LENGTH_CHIPS];

    if (_prn > 0 and _prn < 63)
        {
            make_b2ap(_code, _prn);
        }

    for (signed int i = 0; i < BEIDOU_B2ap_CODE_LENGTH_CHIPS; i++)
        {
            _dest[i] = 1.0 - 2.0 * static_cast<float>(_code[i]);
        }

    delete[] _code;
}
/*
 *  Generates complex BEIDOU B2a pilot code for the desired SV ID and sampled to specific sampling frequency
 */
void beidou_b2ap_code_gen_complex_sampled(std::complex<float>* _dest, unsigned int _prn, signed int _fs)
{
    int32_t* _code = new int32_t[BEIDOU_B2ap_CODE_LENGTH_CHIPS];
    if (_prn > 0 and _prn < 63)
        {
            make_b2ap(_code, _prn);
        }

    signed int _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const signed int _codeLength = BEIDOU_B2ap_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B2ap_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                   // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B2ap_CODE_RATE_HZ);  // C/A chip period in sec

    //float aux;
    for (signed int i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B2a pilot code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B2a code -----------------------
            if (i == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_codeLength - 1], 0);
                }
            else
                {
                    _dest[i] = std::complex<float>(1.0 - 2.0 * _code[_codeValueIndex], 0);  //repeat the chip -> upsample
                }
        }
    delete[] _code;
}
