/*!
 * \file beidou_b1c_signal._processing.cc
 * \brief This class implements signal generators for the BEIDOU B1c signals
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
 * along with GNSS-SDR. If not, see <https://www.gnu.org/licenses/>.
 *
 * -------------------------------------------------------------------------
 */


#include "beidou_b1c_signal_processing.h"
#include "Beidou_B1C.h"
#include <volk_gnsssdr/volk_gnsssdr.h>
#include <cinttypes>
#include <complex>
#include <fstream>
#include<iostream>
#include<cmath>
#include <vector>
#include <deque>
#include <algorithm>

// Make legendre sequence for Primary Data and Primary Pilot codes
bool make_leg_primary(int32_t k)
{
    bool squaremodp = false;
    int32_t z = 1;

    if (k == 0)
        {
            squaremodp = true;
        }
    else
        {
            while (z <= (BEIDOU_B1C_WEIL_N - 1) / 2)
                {
                    squaremodp = false;
                    if (((z * z) % BEIDOU_B1C_WEIL_N) == k)
                        {
                            // k is a square(mod p)
                            squaremodp = true;
                            break;
                        }
                    z = z + 1;
                }
        }

    return squaremodp;
}

// Make B1C Data Primary code
std::deque<bool> make_b1cd_primary_weil_seq(int32_t w, int32_t p)
{
    int32_t n, k = 0;
    int32_t N = BEIDOU_B1C_WEIL_N;
    std::deque<bool> trunc_weil_seq(BEIDOU_B1Cd_CODE_LENGTH_CHIPS, 0);

    // Generate only the truncated sequence
    for (n = 0; n < BEIDOU_B1Cd_CODE_LENGTH_CHIPS; n++)
        {
            k = (n + p - 1) % N;
            trunc_weil_seq[n] = make_leg_primary(k) xor make_leg_primary((k + w) % N);
        }

    return trunc_weil_seq;
}

void make_b1cd(gsl::span<int32_t> _dest, int32_t prn)
{
	int32_t phase_diff = BEIDOU_B1Cd_PHASE_DIFF[prn-1];
	int32_t truncation_point = BEIDOU_B1Cd_TRUNC_POINT[prn-1];

	// Generate Data Primary code
	std::deque<bool> b1cd_primary_code =  make_b1cd_primary_weil_seq(phase_diff, truncation_point);

	for (uint32_t n = 0; n < BEIDOU_B1Cd_CODE_LENGTH_CHIPS; n++)
	{
		_dest[n] = b1cd_primary_code[n];
	}
}



//! Generate a float version of the B1c Data Primary code
void beidou_b1cd_code_gen_float(gsl::span<float> _dest, uint32_t _prn)
{
	uint32_t _code_length = BEIDOU_B1Cd_CODE_LENGTH_CHIPS;
    	int32_t _code[_code_length];

	if (_prn > 0 and _prn < 63)
        {
            make_b1cd(gsl::span<int32_t>(_code, _code_length), _prn);
        }

	for (uint32_t ii = 0; ii < _code_length; ii++)
        {
            _dest[ii] = static_cast<float>(1.0 - 2.0 * static_cast<float>(_code[ii]));
        }
}

// Generate a complex version of the B1c Data Primary code
void beidou_b1cd_code_gen_complex(gsl::span<std::complex<float>> _dest, uint32_t _prn)
{
    	uint32_t _code_length = BEIDOU_B1Cd_CODE_LENGTH_CHIPS;
	int32_t _code[_code_length];

	if (_prn > 0 and _prn < 63)
        {
            make_b1cd(gsl::span<int32_t>(_code, _code_length), _prn);
        }

        for (uint32_t ii = 0; ii < _code_length; ii++)
        {
            _dest[ii] = std::complex<float>(static_cast<float>(1.0 - 2.0 * _code[ii]), 0.0);
        }
}

/*
 *  Generates complex BEIDOU B1c Data Primary code for the desired SV ID and sampled to specific sampling frequency
 */
void beidou_b1cd_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
    uint32_t _code_length = BEIDOU_B1Cd_CODE_LENGTH_CHIPS;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn <= 63)
        {
            make_b1cd(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength = BEIDOU_B1Cd_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1Cd_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B1Cd_CODE_RATE_HZ);  // code chip period in sec

    for (uint32_t i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B1C code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B1Cd code -----------------------
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
}

//=========================================PRIMARY CODE GENERATION OF PILOT COMPONENT=========================================

// Make B1C Pilot Primary code
std::deque<bool> make_b1cp_primary_weil_seq(int32_t w, int32_t p)
{
    int32_t n, k = 0;
    int32_t N = BEIDOU_B1C_WEIL_N;
    std::deque<bool> trunc_weil_seq(BEIDOU_B1Cp_CODE_LENGTH_CHIPS, 0);

    // Generate only the truncated sequence
    for (n = 0; n < BEIDOU_B1Cp_CODE_LENGTH_CHIPS; n++)
        {
            k = (n + p - 1) % N;
            trunc_weil_seq[n] = make_leg_primary(k) xor make_leg_primary((k + w) % N);
        }

    return trunc_weil_seq;
}

//! Generate the B1C Pilot Primary code
void make_b1cp(gsl::span<int32_t> _dest, int32_t prn)
{
	int32_t phase_diff = BEIDOU_B1Cp_PHASE_DIFF[prn-1];
	int32_t truncation_point = BEIDOU_B1Cp_TRUNC_POINT[prn-1];

	// Generate Data Primary code
	std::deque<bool> b1cp_primary_code =  make_b1cp_primary_weil_seq(phase_diff, truncation_point);

	for (uint32_t n = 0; n < 10230; n++)
	{
		_dest[n] = b1cp_primary_code[n];
	}
}


//! Generate a float version of the B1C Pilot Primary code
void beidou_b1cp_code_gen_float(gsl::span<float> _dest, uint32_t _prn)
{
	uint32_t _code_length = BEIDOU_B1Cp_CODE_LENGTH_CHIPS;
    	int32_t _code[_code_length];

	if (_prn > 0 and _prn < 63)
        {
            make_b1cp(gsl::span<int32_t>(_code, _code_length), _prn);
        }

	for (uint32_t ii = 0; ii < _code_length; ii++)
        {
             _dest[ii] = 1.0 - 2.0 * static_cast<float>(_code[ii]);
        }
}

//! Generate a complex version of the B1C Pilot Primary code
void beidou_b1cp_code_gen_complex(gsl::span<std::complex<float>> _dest, uint32_t _prn)
{
    	uint32_t _code_length = BEIDOU_B1Cp_CODE_LENGTH_CHIPS;
	int32_t _code[_code_length];

	if (_prn > 0 and _prn < 63)
        {
            make_b1cp(gsl::span<int32_t>(_code, _code_length), _prn);
        }

        for (uint32_t ii = 0; ii < _code_length; ii++)
        {
            _dest[ii] = std::complex<float>(static_cast<float>(0.0,1.0 - 2.0 * _code[ii]));
        }
}


/*
 *  Generates complex BEIDOU B1C Primary pilot code for the desired SV ID and sampled to specific sampling frequency
 */
void beidou_b1cp_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
    uint32_t _code_length = BEIDOU_B1Cp_CODE_LENGTH_CHIPS;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn < 63)
        {
            make_b1cp(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength = BEIDOU_B1Cp_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1Cp_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B1Cp_CODE_RATE_HZ);  // C/A chip period in sec

    //float aux;
    for (uint32_t ii = 0; ii < _samplesPerCode; ii++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B1C pilot code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(ii) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B1C code -----------------------
            if (ii == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[ii] = std::complex<float>(0, 1.0 - 2.0 * _code[_codeLength - 1]);
                }
            else
                {
                    _dest[ii] = std::complex<float>(0, 1.0 - 2.0 * _code[_codeValueIndex]);  //repeat the chip -> upsample
                }
        }
}

//=========================================SECONDARY CODE GENERATION OF PILOT COMPONENT=========================================

bool make_leg_secondary(int32_t k)
{
    bool squaremodp = false;
    int32_t z = 1;

    if (k == 0)
        {
            squaremodp = true;
        }
    else
        {
            while (z <= (BEIDOU_B1C_WEIL_N_SECONDARY - 1) / 2)
                {
                    squaremodp = false;
                    if (((z * z) % BEIDOU_B1C_WEIL_N_SECONDARY) == k)
                        {
                            // k is a square(mod p)
                            squaremodp = true;
                            break;
                        }
                    z = z + 1;
                }
        }

    return squaremodp;
}

// Make B1C Pilot Secondary code
std::deque<bool> make_b1cp_secondary_weil_seq(int32_t w, int32_t p)
{
    int32_t n, k = 0;
    int32_t N = BEIDOU_B1C_WEIL_N_SECONDARY;
    std::deque<bool> trunc_weil_seq(BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS, 0);

    // Generate only the truncated sequence
    for (n = 0; n < BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS; n++)
        {
            k = (n + p - 1) % N;
            trunc_weil_seq[n] = make_leg_secondary(k) xor make_leg_secondary((k + w) % N);
        }

    return trunc_weil_seq;
}

//! Generate a version of the B1C Pilot code with the secondary pilot code included
void make_b1cp_secondary(gsl::span<int32_t> _dest, int32_t prn)
{
    
    int32_t phase_diff_primary = BEIDOU_B1Cp_PHASE_DIFF[prn-1];
    int32_t truncation_point_primary = BEIDOU_B1Cp_TRUNC_POINT[prn-1];
    
    int32_t phase_diff_secondary = BEIDOU_B1Cp_SECONDARY_PHASE_DIFF[prn - 1];
    int32_t truncation_point_secondary = BEIDOU_B1Cp_SECONDARY_TRUNC_POINT[prn - 1];
    
    // Generate primary code
    std::deque<bool> b1cp_primary_code = make_b1cp_primary_weil_seq(phase_diff_primary, truncation_point_primary);
    
    // Generate secondary code
    std::deque<bool> b1cp_sec_code = make_b1cp_secondary_weil_seq(phase_diff_secondary, truncation_point_secondary);

    for (uint32_t m = 0; m < BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS; m++)
        {
            for (uint32_t n = 0; n < BEIDOU_B1Cp_CODE_LENGTH_CHIPS; n++)
                {
                    _dest[n + m * BEIDOU_B1Cp_CODE_LENGTH_CHIPS] = b1cp_primary_code[n] xor b1cp_sec_code[m];
                }
        }
}


// Generate a complex version of the B1C pilot code with the secondary pilot code
void beidou_b1cp_code_gen_complex_secondary(gsl::span<std::complex<float>> _dest, uint32_t _prn)
{
    uint32_t _code_length = BEIDOU_B1Cp_CODE_LENGTH_CHIPS * BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn < 63)
        {
            make_b1cp_secondary(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    for (uint32_t ii = 0; ii < _code_length; ii++)
        {
            _dest[ii] = std::complex<float>(0.0, static_cast<float>(1.0 - 2.0 * _code[ii]));
        }
}


/*
 * Generates a float version of the B1C pilot primary code
 */
void beidou_b1cp_code_gen_float_secondary(gsl::span<float> _dest, uint32_t _prn)
{
    uint32_t _code_length = BEIDOU_B1Cp_CODE_LENGTH_CHIPS * BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn < 63)
        {
            make_b1cp_secondary(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    for (uint32_t ii = 0; ii < _code_length; ii++)
        {
            _dest[ii] = static_cast<float>(1.0 - 2.0 * static_cast<float>(_code[ii]));
        }
}


/*
 *  Generates complex BEIDOU B1C pilot code for the desired SV ID and sampled to specific sampling frequency with the secondary code implemented
 */
void beidou_b1cp_code_gen_complex_sampled_secondary(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
    uint32_t _code_length = BEIDOU_B1Cp_CODE_LENGTH_CHIPS * BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS;
    int32_t _code[_code_length];

    if (_prn > 0 and _prn <= 63)
        {
            make_b1cp_secondary(gsl::span<int32_t>(_code, _code_length), _prn);
        }

    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength = BEIDOU_B1Cp_CODE_LENGTH_CHIPS * BEIDOU_B1Cp_SECONDARY_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1Cp_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B1Cp_CODE_RATE_HZ);  // code chip period in sec

    for (uint32_t ii = 0; ii < _samplesPerCode; ii++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B1C code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(ii) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B1Cp code -----------------------
            if (ii == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[ii] = std::complex<float>(0, 1.0 - 2.0 * _code[_codeLength - 1]);
                }
            else
                {
                    _dest[ii] = std::complex<float>(0, 1.0 - 2.0 * _code[_codeValueIndex]);  //repeat the chip -> upsample
                }
        }
}



/*
 *  Generates complex BEIDOU B1C data+pilot code for the desired SV ID and sampled to specific sampling frequency
 */
void beidou_b1c_code_gen_complex_sampled(gsl::span<std::complex<float>> _dest, uint32_t _prn, int32_t _fs)
{
    uint32_t _code_length_data = BEIDOU_B1Cd_CODE_LENGTH_CHIPS;
    uint32_t _code_length_pilot = BEIDOU_B1Cp_CODE_LENGTH_CHIPS;
    int32_t _code_pilot[_code_length_pilot];
    int32_t _code_data[_code_length_data];

    if (_prn > 0 and _prn < 63)
        {
            make_b1cp(gsl::span<int32_t>(_code_pilot, _code_length_pilot), _prn);
            make_b1cd(gsl::span<int32_t>(_code_data, _code_length_data), _prn);
        }

    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength = BEIDOU_B1Cp_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1Cp_CODE_RATE_HZ) / static_cast<double>(_codeLength)));

    //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B1Cp_CODE_RATE_HZ);  // C/A chip period in sec

    //float aux;
    for (uint32_t ii = 0; ii < _samplesPerCode; ii++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B1C pilot code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(ii) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B1C code -----------------------
            if (ii == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[ii] = std::complex<float>(1.0 - 2.0 * _code_data[_codeLength - 1], 1.0 - 2.0 * _code_pilot[_codeLength - 1]);
                }
            else
                {
                    _dest[ii] = std::complex<float>(1.0 - 2.0 * _code_data[_codeValueIndex], 1.0 - 2.0 * _code_pilot[_codeValueIndex]);  //repeat the chip -> upsample
                }
        }
}


//================================================BDS_B1C_BOC_Generation===================================================


void beidou_b1c_data_sinboc_11_gen_int(gsl::span<int> _dest, gsl::span<const int> _prn)
{
    const uint32_t _length_in = BEIDOU_B1Cd_CODE_LENGTH_CHIPS;
    auto _period = static_cast<uint32_t>(_dest.size() / _length_in);
    for (uint32_t i = 0; i < _length_in; i++)
        {
            for (uint32_t j = 0; j < (_period / 2); j++)
                {
                    _dest[i * _period + j] = _prn[i];
                }
            for (uint32_t j = (_period / 2); j < _period; j++)
                {
                    _dest[i * _period + j] = -_prn[i];
                }
        }
}


void beidou_b1c_pilot_sinboc_11_gen_int(gsl::span<int> _dest, gsl::span<const int> _prn)
{
    const uint32_t _length_in = BEIDOU_B1Cp_CODE_LENGTH_CHIPS;
    auto _period = static_cast<uint32_t>(_dest.size() / _length_in);
    for (uint32_t i = 0; i < _length_in; i++)
        {
            for (uint32_t j = 0; j < (_period / 2); j++)
                {
                    _dest[i * _period + j] = _prn[i];
                }
            for (uint32_t j = (_period / 2); j < _period; j++)
                {
                    _dest[i * _period + j] = -_prn[i];
                }
        }
}


void beidou_b1c_pilot_sinboc_61_gen_int(gsl::span<int> _dest, gsl::span<const int> _prn)
{
    const uint32_t _length_in = BEIDOU_B1Cp_CODE_LENGTH_CHIPS;
    auto _period = static_cast<uint32_t>(_dest.size() / _length_in);

    for (uint32_t i = 0; i < _length_in; i++)
        {
            for (uint32_t j = 0; j < _period; j += 2)
                {
                    _dest[i * _period + j] = _prn[i];
                }
            for (uint32_t j = 1; j < _period; j += 2)
                {
                    _dest[i * _period + j] = -_prn[i];
                }
        }
}


//! Generates float version of sine BOC(1,1) modulated Data Code 
void beidou_b1cd_gen_float_11(gsl::span<float> _dest, uint32_t _prn)
{
	   /*
	    *  PROCEDURE:
	    *  1. Generate Beidou B1C Data Code
	    *  2. Apply Sine BOC(1,1) on generated Beidou B1C Data Code
	    *  3. Multiply the output of Sine BOC(1,1) with the Power Ratio
	    */
	
	
   	 // This function is based on the GNU software GPS for MATLAB in Kay Borre's book
    	
    	
    	auto* b1c_data_primary_code_chips = static_cast<int32_t*>(volk_gnsssdr_malloc(static_cast<uint32_t>(BEIDOU_B1Cd_CODE_LENGTH_CHIPS) * sizeof(int32_t), volk_gnsssdr_get_alignment()));
    	const int32_t _samplesPerChip =  12;

    	// 1. Generate Beidou B1C Data Code
    	make_b1cd(gsl::span<int32_t>(b1c_data_primary_code_chips, static_cast<uint32_t>(BEIDOU_B1Cd_CODE_LENGTH_CHIPS)), _prn); //generate Beidou B1C code, 1 sample per chip
	

	
	
	//In BOC(n,m),n= subcarrier frequency (subchip frequency) and m= code chipping rate
	//M = number of subchips per chip is calculated as, M=2*n/m
	//Here, 12 is the result of the sub chips after the BOC is applied
	const uint32_t _codeLength1 = 12 * BEIDOU_B1Cd_CODE_LENGTH_CHIPS;
	
	//Value of alpha is according to equation 4-11 in ICD which is data component and in Real part of the equation, 
	//SB1C(t)=(1/2*DB1C_data(t))*(CB1C_data(t))*(sign(sin(2πfsc_B1C_at)))+(sqrt(1.0 / 11.0)*(CB1C_pilot(t))*(sign(sin(2πfsc_B1C_bt)))+j(sqrt(29.0 / 44.0)*(CB1C_pilot(t)*(sign(sin(2πfsc_B1C_t)))

	const float alpha = (1.0 / 2.0);  
	
	int32_t sinboc_11[12 * BEIDOU_B1Cd_CODE_LENGTH_CHIPS] = {0}; //  _codeLength not accepted by Clang
	
	gsl::span<int32_t> sinboc_11_(sinboc_11, _codeLength1);
	
	// 2. Apply Sine BOC(1,1) on generated Beidou B1C Data Code
	beidou_b1c_data_sinboc_11_gen_int(sinboc_11_, gsl::span<int>(b1c_data_primary_code_chips, static_cast<uint32_t>(BEIDOU_B1Cd_CODE_LENGTH_CHIPS))); //generate sinboc(1,1) 12 samples per chip
	
	// 3. Multiply the output of Sine BOC(1,1) with the Power Ratio
	for (uint32_t i = 0; i < _codeLength1; i++)
        {
        	_dest[i] = alpha * static_cast<float>(sinboc_11[i]);
                               
	}
}

//! Generates complex version of sine BOC(1,1) modulated Data Code 
void beidou_b1cd_code_gen_complex_sampled_boc_11(gsl::span<std::complex<float>> _dest,uint32_t _prn, int32_t _fs)
{
    
    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    //In BOC(n,m),n= subcarrier frequency (subchip frequency) and m= code chipping rate
    //M = number of subchips per chip is calculated as, M=2*n/m
    //Here, 12 is the result of the sub chips after the BOC is applied
    const int32_t _codeLength =12 * BEIDOU_B1Cd_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1Cd_CODE_RATE_HZ) / static_cast<double>(_codeLength)));
    
    auto* real_code = static_cast<float*>(volk_gnsssdr_malloc(_samplesPerCode * sizeof(float), volk_gnsssdr_get_alignment()));
    gsl::span<float> real_code_span(real_code, _samplesPerCode);
    beidou_b1cd_gen_float_11(real_code_span, _prn);
    
     //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B1Cd_CODE_RATE_HZ);  // code chip period in sec
    
        
    for (uint32_t i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B1C code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B1Cd code -----------------------
            if (i == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[i] = std::complex<float>(1.0 - 2.0 * real_code_span[_codeLength - 1], 0);
                }
            else
                {
                    _dest[i] = std::complex<float>(1.0 - 2.0 * real_code_span[_codeValueIndex], 0);  //repeat the chip -> upsample
                }
        }           
    volk_gnsssdr_free(real_code);
}


//--------------------------------------------------BOC_FOR_PILOT_COMPONENT----------------------------------------------------

//! Generate BOC for first Pilot component which is in Real part
void beidou_b1cp_gen_float_61(gsl::span<float> _dest, uint32_t _prn)
{
	   /*
	    *  PROCEDURE:
	    *  1. Generate Beidou B1C Pilot Code
	    *  2. Apply Sine BOC(6,1) on generated Beidou B1C Pilot Code
	    *  3. Multiply the output of Sine BOC(6,1) with the Power Ratio
	    */	
	
    	// This function is based on the GNU software GPS for MATLAB in Kay Borre's book
    	auto* b1c_pilot_primary_code_chips = static_cast<int32_t*>(volk_gnsssdr_malloc(static_cast<uint32_t>(BEIDOU_B1Cp_CODE_LENGTH_CHIPS) * sizeof(int32_t), volk_gnsssdr_get_alignment()));
    	const int32_t _samplesPerChip =  12;	
	
	
	// 1. Generate Beidou B1C Pilot Code
	make_b1cp(gsl::span<int32_t>(b1c_pilot_primary_code_chips, static_cast<uint32_t>(BEIDOU_B1Cp_CODE_LENGTH_CHIPS)), _prn); //generate Beidou B1C pilot code, 1 sample per chip
	
	//In BOC(n,m),n= subcarrier frequency (subchip frequency) and m= code chipping rate
	//M = number of subchips per chip is calculated as, M=2*n/m
	//Here, 12 is the result of the sub chips after the BOC is applied
	const uint32_t _codeLength = 12 * BEIDOU_B1Cp_CODE_LENGTH_CHIPS;
	
	//Value of alpha is according to equation 4-11 in ICD which is pilot component and in Real part of the equation, 
	//SB1C(t)=(1/2*DB1C_data(t))*(CB1C_data(t))*(sign(sin(2πfsc_B1C_at)))+(sqrt(1.0 / 11.0)*(CB1C_pilot(t))*(sign(sin(2πfsc_B1C_bt)))+j(sqrt(29.0 / 44.0)*(CB1C_pilot(t)*(sign(sin(2πfsc_B1C_t)))
	const float alpha = sqrt(1.0 / 11.0);				// Power Ratio
	
	int32_t sinboc_61[12 * BEIDOU_B1Cp_CODE_LENGTH_CHIPS] = {0};    //  _codeLength not accepted by Clang
	gsl::span<int32_t> sinboc_61_(sinboc_61, _codeLength);
    	   	
   	// 2. Apply Sine BOC(6,1) on generated Beidou B1C Pilot Code
   	beidou_b1c_pilot_sinboc_61_gen_int(sinboc_61_,gsl::span<int>(b1c_pilot_primary_code_chips, static_cast<uint32_t>(BEIDOU_B1Cp_CODE_LENGTH_CHIPS)));  //generate sinboc(6,1) 12 samples per chip
    		
	// 3. Multiply the output of Sine BOC(6,1) with the Power Ratio
	for (uint32_t i = 0; i < _codeLength; i++)
        {
        	_dest[i] = alpha * static_cast<float>(sinboc_61[i]);
                               
	}
}


//! Generate BOC for second Pilot component which is in Imaginary part
void beidou_b1cp_gen_float_11(gsl::span<float> _dest, uint32_t _prn)
{
	   /*
	    *  PROCEDURE:
	    *  1. Generate Beidou B1C Pilot Code
	    *  2. Apply Sine BOC(1,1) on generated Beidou B1C Data Code(This Component is in Imaginary part of signal eqation)
	    *  3. Multiply the output of Sine BOC(6,1) with the Power Ratio
	    */		
        
        
        // This function is based on the GNU software GPS for MATLAB in Kay Borre's book
    	//auto _codeLength = static_cast<uint32_t>(BEIDOU_B1Cp_CODE_LENGTH_CHIPS);
    	auto* b1c_pilot_primary_code_chips = static_cast<int32_t*>(volk_gnsssdr_malloc(static_cast<uint32_t>(BEIDOU_B1Cp_CODE_LENGTH_CHIPS) * sizeof(int32_t), volk_gnsssdr_get_alignment()));
    	const int32_t _samplesPerChip =  12;
    	
    	
    	// 1. Generate Beidou B1C Pilot Code
    	make_b1cp(gsl::span<int32_t>(b1c_pilot_primary_code_chips, static_cast<uint32_t>(BEIDOU_B1Cp_CODE_LENGTH_CHIPS)), _prn); //generate Beidou B1C pilot code, 1 sample per chip

	//In BOC(n,m),n= subcarrier frequency (subchip frequency) and m= code chipping rate
	//M = number of subchips per chip is calculated as, M=2*n/m
	//Here, 12 is the result of the sub chips after the BOC is applied
	const uint32_t _codeLength = 12 * BEIDOU_B1Cp_CODE_LENGTH_CHIPS;
	
    	//Value of alpha is according to equation 4-11 in ICD which is pilot component and in Imaginary part of equation, 
	//SB1C(t)=(1/2*DB1C_data(t))*(CB1C_data(t))*(sign(sin(2πfsc_B1C_at)))+(sqrt(1.0 / 11.0)*(CB1C_pilot(t))*(sign(sin(2πfsc_B1C_bt)))+j(sqrt(29.0 / 44.0)*(CB1C_pilot(t)*(sign(sin(2πfsc_B1C_t)))

	const float beta = sqrt(29.0 / 44.0);
	
	int32_t sinboc_11[12 * BEIDOU_B1Cp_CODE_LENGTH_CHIPS] = {0};     //  _codeLength not accepted by Clang
    	gsl::span<int32_t> sinboc_11_(sinboc_11, _codeLength);
    	
    	// 2. Apply Sine BOC(1,1) on generated Beidou B1C Pilot Code
    	beidou_b1c_pilot_sinboc_11_gen_int(sinboc_11_, gsl::span<int>(b1c_pilot_primary_code_chips, static_cast<uint32_t>(BEIDOU_B1Cp_CODE_LENGTH_CHIPS)));  //generate sinboc(1,1) 12 samples per chip
    	
	// 3. Multiply the output of Sine BOC(1,1) with the Power Ratio
	for (uint32_t i = 0; i < _codeLength; i++)
        {
        	_dest[i] = beta * static_cast<float>(sinboc_11[i]);
                               
	}
}


//! Generates complex version of both pilot components having sine BOC(6,1) which is in Real part and sine BOC(1,1) which is in Imaginary part
void beidou_b1cp_code_gen_complex_sampled_boc_61_11(gsl::span<std::complex<float>> _dest,uint32_t _prn, int32_t _fs)
{
    
    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength =12 * BEIDOU_B1Cp_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1Cp_CODE_RATE_HZ) / static_cast<double>(_codeLength)));
    
    auto* real_code = static_cast<float*>(volk_gnsssdr_malloc(_samplesPerCode * sizeof(float), volk_gnsssdr_get_alignment()));
    gsl::span<float> real_code_span(real_code, _samplesPerCode);
    beidou_b1cp_gen_float_61(real_code_span, _prn);
    
    
    auto* imaginary_code = static_cast<float*>(volk_gnsssdr_malloc(_samplesPerCode * sizeof(float), volk_gnsssdr_get_alignment()));
    gsl::span<float> imaginary_code_span(imaginary_code, _samplesPerCode);
    beidou_b1cp_gen_float_11(imaginary_code_span, _prn);
    
     //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B1Cp_CODE_RATE_HZ);  // code chip period in sec
    
        
    for (uint32_t i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B1C code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B1Cd code -----------------------
            if (i == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[i] = std::complex<float>(1.0 - 2.0 * real_code_span[_codeLength - 1], 1.0 - 2.0 * imaginary_code_span[_codeLength - 1]);
                }
            else
                {
                    _dest[i] = std::complex<float>(1.0 - 2.0 * real_code_span[_codeValueIndex], 1.0 - 2.0 * imaginary_code_span[_codeValueIndex]);  //repeat the chip -> upsample
                }
        }           
    volk_gnsssdr_free(real_code);
    volk_gnsssdr_free(imaginary_code);
}


/*
* Generates complex version of data+pilot components as follows
* Data component(in ICD Equation 4-11) in Sine BOC(1,1) and is in Real part  
* First Pilot component(in ICD Equation 4-11) in Sine BOC(6,1) and is in Real part  
* Second Pilot component(in ICD Equation 4-11) in Sine BOC(1,1) and is in Imaginary part 
*/
void beidou_b1c_code_gen_complex_sampled_boc(gsl::span<std::complex<float>> _dest,uint32_t _prn, int32_t _fs)
{
    //uint32_t _code_length =12 * BEIDOU_B1Cd_CODE_LENGTH_CHIPS;
    //int32_t _code[_code_length];
    
    int32_t _samplesPerCode, _codeValueIndex;
    float _ts;
    float _tc;
    const int32_t _codeLength =12 * BEIDOU_B1Cd_CODE_LENGTH_CHIPS;

    //--- Find number of samples per spreading code ----------------------------
    _samplesPerCode = static_cast<int>(static_cast<double>(_fs) / (static_cast<double>(BEIDOU_B1Cd_CODE_RATE_HZ) / static_cast<double>(_codeLength)));
    
    //Generating Data Component which is in Real Part
    auto* real_code_data = static_cast<float*>(volk_gnsssdr_malloc(_samplesPerCode * sizeof(float), volk_gnsssdr_get_alignment()));
    gsl::span<float> real_code_span_data(real_code_data, _samplesPerCode);
    beidou_b1cd_gen_float_11(real_code_span_data, _prn);


    //Generating Pilot Component which is in Real Part
    auto* real_code_pilot = static_cast<float*>(volk_gnsssdr_malloc(_samplesPerCode * sizeof(float), volk_gnsssdr_get_alignment()));
    gsl::span<float> real_code_span_pilot(real_code_pilot, _samplesPerCode);
    beidou_b1cp_gen_float_61(real_code_span_pilot, _prn);
        
    
    //Generating Pilot Component which is in Imaginary Part
    auto* imaginary_code_pilot = static_cast<float*>(volk_gnsssdr_malloc(_samplesPerCode * sizeof(float), volk_gnsssdr_get_alignment()));
    gsl::span<float> imaginary_code_span_pilot(imaginary_code_pilot, _samplesPerCode);
    beidou_b1cp_gen_float_11(imaginary_code_span_pilot, _prn);
    
    
    //XORing the output of two Real parts to put in one Real Part 
    auto* real_code = static_cast<float*>(volk_gnsssdr_malloc(_samplesPerCode * sizeof(float), volk_gnsssdr_get_alignment()));
    gsl::span<float> real_code_span(real_code, _samplesPerCode);
    
    for (uint32_t m = 0; m < 12 * BEIDOU_B1Cd_CODE_LENGTH_CHIPS; m++)
        {
            for (uint32_t n = 0; n < 12 * BEIDOU_B1Cd_CODE_LENGTH_CHIPS; n++)
                {
		    real_code_span[n + m * 12 * BEIDOU_B1Cd_CODE_LENGTH_CHIPS] = real_code_span_data[n] + real_code_span_pilot[m];
                }
        }
    
    
     //--- Find time constants --------------------------------------------------
    _ts = 1.0 / static_cast<float>(_fs);                       // Sampling period in sec
    _tc = 1.0 / static_cast<float>(BEIDOU_B1Cd_CODE_RATE_HZ);  // code chip period in sec
    
        
    for (uint32_t i = 0; i < _samplesPerCode; i++)
        {
            //=== Digitizing =======================================================

            //--- Make index array to read B1C code values -------------------------
            _codeValueIndex = ceil((_ts * (static_cast<float>(i) + 1)) / _tc) - 1;

            //--- Make the digitized version of the B1Cd code -----------------------
            if (i == _samplesPerCode - 1)
                {
                    //--- Correct the last index (due to number rounding issues) -----------
                    _dest[i] = std::complex<float>(1.0 - 2.0 * real_code_span[_codeLength - 1], 1.0 - 2.0 * imaginary_code_span_pilot[_codeLength - 1]);
                }
            else
                {
                    _dest[i] = std::complex<float>(1.0 - 2.0 * real_code_span[_codeValueIndex], 1.0 - 2.0 * imaginary_code_span_pilot[_codeValueIndex]);  //repeat the chip -> upsample
                }
        }           
    volk_gnsssdr_free(real_code_data);
    volk_gnsssdr_free(real_code_pilot);
    volk_gnsssdr_free(imaginary_code_pilot);
    volk_gnsssdr_free(real_code);
}


//! Generates Data code required at the time of tracking(followed approach like Galileo E1)
void beidou_b1cd_code_gen_sinboc11_float(gsl::span<float> _dest, uint32_t _prn)
{

    const auto _codeLength = static_cast<uint32_t>(BEIDOU_B1Cd_CODE_LENGTH_CHIPS);
    std::array<int32_t, BEIDOU_B1Cd_CODE_LENGTH_CHIPS> primary_code_b1c_chips{};                                               // _codeLength not accepted by Clang
    make_b1cd(gsl::span<int32_t>(primary_code_b1c_chips.data(), BEIDOU_B1Cd_CODE_LENGTH_CHIPS), _prn);  //generate beidou B1C code, 1 sample per chip
    for (uint32_t i = 0; i < _codeLength; i++)
        {
            _dest[2 * i] = static_cast<float>(primary_code_b1c_chips[i]);
            _dest[2 * i + 1] = -_dest[2 * i];
        }
}


//! Generates pilot code required at the time of tracking(followed approach like Galileo E1)
void beidou_b1cp_code_gen_sinboc11_float(gsl::span<float> _dest, uint32_t _prn)
{

    const auto _codeLength = static_cast<uint32_t>(BEIDOU_B1Cp_CODE_LENGTH_CHIPS);
    std::array<int32_t, BEIDOU_B1Cp_CODE_LENGTH_CHIPS> primary_code_b1c_chips{};                                               // _codeLength not accepted by Clang
    make_b1cp(gsl::span<int32_t>(primary_code_b1c_chips.data(), BEIDOU_B1Cp_CODE_LENGTH_CHIPS), _prn);  //generate beidou B1C code, 1 sample per chip
    for (uint32_t i = 0; i < _codeLength; i++)
        {
            _dest[2 * i] = static_cast<float>(primary_code_b1c_chips[i]);
            _dest[2 * i + 1] = -_dest[2 * i];
        }
}
