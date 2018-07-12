/*!
 * \file beidou_cnav2_ephemeris.h
 * \brief  Interface of a BEIDOU EPHEMERIS storage
 * \note Code added as part of GSoC 2018 program
 * \author Dong Kyeong Lee, 2018. dole7890(at)colorado.edu
 * \see <a href="http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf">beidou ICD</a>
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


#ifndef GNSS_SDR_BEIDOU_CNAV2_EPHEMERIS_H_
#define GNSS_SDR_BEIDOU_CNAV2_EPHEMERIS_H_


#include <boost/serialization/nvp.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


/*!
 * \brief This class is a storage and orbital model functions for the BEIDOU SV ephemeris data as described in beidou ICD (Edition 5.1)
 * \note Code added as part of GSoC 2018 program
 * \see <a href="http://m.beidou.gov.cn/xt/gfxz/201712/P020171226742357364174.pdf">BEIDOU ICD</a>
 */
class Beidou_Cnav2_Ephemeris
{

public:
	// Other values
	unsigned int PRN;	//Pseudo-Random Noise, Satellite ID
	double SOW;			//Seconds of week [s]
	double WN;			//Week number [week]
	double Rev;			//Unspecified in ICD. Probably Reserved

	// Satellite Health Satus
	double HS;			//0:Satellite is healthy/provides services, 1:Satellite is unhealthy or in test/does not provide services, 2:reserved/reserved, 3:reserved/reserved

	// Issue of Data, Ephemeris
	double IODE;		//Issue of Data, Ephemeris

	// Issue of Data, Clock
	double IODC;		//Issue of Data, Clock

	// Ephemeris Perimeters
	double t_oe;		//Ephemeris reference time [s]
	double SatType;		//Satellite orbit type [dimensionless]
	double dA;			//Semi-major axis difference at reference time [m]
	double A_dot;		//Change rate in semi-major axis [m/s]
	double dn_0;		//Mean motion difference from computed value at reference time [pi/s]
	double dn_0_dot;	//Rate of mean motion difference from computed value at reference time [pi/s^2]
	double M_0;			//Mean anomaly at reference time [pi]
	double e;			//Eccentricity [dimensionless]
	double omega;		//Argument of perigee [pi]
	double Omega_0;		//Longitude of ascending node of orbital plane at weekly epoch [pi]
	double i_0;			//Inclination angle at reference time [pi]
	double Omega_dot;	//Rate of right ascension [pi/s]
	double i_0_dot;		//Rate of inclination angle [pi/s]
	double C_IS;		//Amplitude of sine harmonic correction term to the angle of inclination [rad]
	double C_IC;		//Amplitude of cosine harmonic correction term to the angle of inclination [rad]
	double C_RS;		//Amplitude of sine harmonic correction term to the orbit radius [m]
	double C_RC;		//Amplitude of cosine harmonic correction term to the orbit radius [m]
	double C_US;		//Amplitude of sine harmonic correction ot the argument of latitude [rad]
	double C_UC;		//Amplitude of cosine harmonic correction to the argument of latitude [rad]

	// Earth Orientation Parameters
	double t_EOP;		//EOP data reference time [s]
	double PM_X;		//X Axis polar motion value at reference time [arc s]
	double PM_X_dot;	//X Axis polar motion drift at reference time [arc s/day]
	double PM_Y;		//Y Axis polar motion value at reference time [arc s]
	double PM_Y_dot;	//Y Axis polar motion drift at reference time [arc s/day]
	double dUT1;		//UT1-UTC difference at reference time [s]
	double dUT1_dot;	//Rate of UT1-UTC difference at reference time [s/day]

	//Satellite Integrity Status Flag
	double DIF;			//Data Integrity Flag, 0:The error of message parameters broadcasted in this signal does not exceed the predictive accuracy, 1:The error of message parameters broadcasted in this signal exceeds the predictive accuracy
	double SIF;			//Signal Integrity Flag, 0:This signal is normal, 1:This signal is abnormal
	double AIF;			//Accuracy Integrity Flag, 0:SISMAI value of this signal is valud, 1:SISMAI value of this signal is invalid
	double DIF_B1C;		//Data Integrity FLAG B1C
	double SIF_B1C;		//Signal Integrity FLAG B1C
	double AIF_B1C;		//Accuracy Integrity FLAG B1C
	double SISMAI;		//Signal in space monitoring accuracy index

	// Signal In Space Accuracy Index
	double SISAI_OE;	//Satellite orbit along-track and cross-track accuracy index
	double t_op;		//Time of week for data prediction
	double SISAI_ocb;	//Satellite orbit radius and fixed satellite clock bias accuracy index
	double SISAI_oc1;	//Satellite clock bias accuracy index
	double SISAI_oc2;	//Satellite clock drift accuracy index

	// Ionospheric Delay Correction Model Parameters
	double alpha_1;		//[TECu]
	double alpha_2;		//[TECu]
	double alpha_3;		//[TECu]
	double alpha_4;		//[TECu]
	double alpha_5;		//[TECu]
	double alpha_6;		//[TECu]
	double alpha_7;		//[TECu]
	double alpha_8;		//[TECu]
	double alpha_9;		//[TECu]

	// Group Delay Differential Parameters
	double T_GDB1Cp;	//Group delay differential of the B1C pilot component [s]
	double T_GDB2ap;	//Group delay differential of the B2a pilot component [s]
	double ISC_B2ad;	//Group delay differential between the B2a data and pilot components [s]

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization. Here is used to save the ephemeris data on disk file.
     */
    void serialize(Archive& archive, const unsigned int version)
    {
        using boost::serialization::make_nvp;
        if (version)
            {
            };
    	// Others values
    	archive& make_nvp("PRN", PRN);
    	archive& make_nvp("SOW", SOW);
    	archive& make_nvp("WN", WN);

    	// Satellite Heath Status
    	archive& make_nvp("HS", HS);	//0:Satellite is healthy/provides services, 1:Satellite is unhealthy or in test/does not provide services, 2:reserved/reserved, 3:reserved/reserved

    	// Issue of Data, Ephemeris
    	archive& make_nvp("IODE", IODE);

    	// Issue of Data, Clock
    	archive& make_nvp("IODC", IODC);

        // Ephemeris
        archive& make_nvp("t_oe", t_oe);  //!< SV PRN frequency channel number
        archive& make_nvp("SatType", SatType);
        archive& make_nvp("dA", dA);
        archive& make_nvp("A_dot", A_dot);
        archive& make_nvp("dn_0", dn_0);
        archive& make_nvp("dn_0_dot", dn_0_dot);
        archive& make_nvp("M_0", M_0);
        archive& make_nvp("e", e);
        archive& make_nvp("omega", omega);
        archive& make_nvp("Omega_0", Omega_0);
        archive& make_nvp("i_0", i_0);
        archive& make_nvp("Omega_dot", Omega_dot);
        archive& make_nvp("i_0_dot", i_0_dot);
        archive& make_nvp("C_IS", C_IS);
        archive& make_nvp("C_IC", C_IC);
        archive& make_nvp("C_RS", C_RS);
        archive& make_nvp("C_RC", C_RC);
        archive& make_nvp("C_US", C_US);
        archive& make_nvp("C_UC", C_UC);

    	// Earth Orientation Parameters
    	archive& make_nvp("t_EOP", t_EOP);			//EOP data reference time [s]
    	archive& make_nvp("PM_X", PM_X);			//X Axis polar motion value at reference time [arc s]
    	archive& make_nvp("PM_X_dot", PM_X_dot);	//X Axis polar motion drift at reference time [arc s/day]
    	archive& make_nvp("PM_Y", PM_Y);			//Y Axis polar motion value at reference time [arc s]
    	archive& make_nvp("PM_Y_dot", PM_Y_dot);	//Y Axis polar motion drift at reference time [arc s/day]
    	archive& make_nvp("dUT1", dUT1);			//UT1-UTC difference at reference time [s]
    	archive& make_nvp("dUT1_dot", dUT1_dot);	//Rate of UT1-UTC difference at reference time [s/day]

    	// Satellite Integrity Flag
    	archive& make_nvp("DIF", DIF);
    	archive& make_nvp("SIF", AIF);
    	archive& make_nvp("AIF", AIF);
    	archive& make_nvp("DIF_B1C", DIF_B1C);
    	archive& make_nvp("SIF_B1C", SIF_B1C);
    	archive& make_nvp("AIF_B1C", AIF_B1C);
    	archive& make_nvp("SISMAI", SISMAI);

    	// Signal In Space Accuracy Index
    	archive& make_nvp("SISAI_OE", SISAI_OE);	//Satellite orbit along-track and cross-track accuracy index
		archive& make_nvp("t_op", t_op);			//Time of week for data prediction
		archive& make_nvp("SISAI_ocb", SISAI_ocb);	//Satellite orbit radius and fixed satellite clock bias accuracy index
		archive& make_nvp("SISAI_oc1", SISAI_oc1);	//Satellite clock bias accuracy index
		archive& make_nvp("SISAI_oc2", SISAI_oc2);	//Satellite clock drift accuracy index

		// Ionospheric Delay Correction Model Parameters
		archive& make_nvp("alpha_1", alpha_1);
		archive& make_nvp("alpha_2", alpha_2);
		archive& make_nvp("alpha_3", alpha_3);
		archive& make_nvp("alpha_4", alpha_4);
		archive& make_nvp("alpha_5", alpha_5);
		archive& make_nvp("alpha_6", alpha_6);
		archive& make_nvp("alpha_7", alpha_7);
		archive& make_nvp("alpha_8", alpha_8);
		archive& make_nvp("alpha_9", alpha_9);

		// Group Delay Differential Parameters
		archive& make_nvp("T_GDB1Cp", T_GDB1Cp);	//Group delay differential of the B1C pilot component [s]
		archive& make_nvp("T_GDB2ap", T_GDB2ap);	//Group delay differential of the B2a pilot component [s]
		archive& make_nvp("ISC_B2ad", ISC_B2ad);	//Group delay differential between the B2a data and pilot components [s]
    }

    /*!
     * \brief Sets (\a d_satClkDrift)and returns the clock drift in seconds according to the User Algorithm for SV Clock Correction
     *  (IS-GPS-200E,  20.3.3.3.3.1)
     */
    double sv_clock_drift(double transmitTime, double timeCorrUTC);

    /*!
     *  \brief Computes the BEIDOU System Time and returns a boost::posix_time::ptime object
     * \ param offset_time Is the start of day offset to compute the time
     */
    boost::posix_time::ptime compute_BEIDOU_time(const double offset_time) const;

    /*!
     * \brief Converts from BEIDOUT to UTC
     * \details The function simply adjust for the 6 hrs offset between BEIDOUT and UTC
     * \param[in] offset_time Is the start of day offset
     * \param[in] beidt2utc_corr Correction from BEIDOUT to UTC
     * \returns UTC time as a boost::posix_time::ptime object
     */
    boost::posix_time::ptime beidt_to_utc(const double offset_time, const double glot2utc_corr) const;

    /*!
     * \brief Converts from BEIDOUT to GPST
     * \details Converts from BEIDOUT to GPST in time of week (TOW) and week number (WN) format
     * \param[in] tod_offset Is the start of day offset
     * \param[in] beidt2utc_corr Correction from BEIDOUT to UTC
     * \param[in] beidt2gpst_corr Correction from BEIDOUT to GPST
     * \param[out] WN Week Number, not in mod(1024) format
     * \param[out] TOW Time of Week in seconds of week
     */
    void beidt_to_gpst(double tod_offset, double beidt2utc_corr, double beidt2gpst_corr, double* WN, double* TOW) const;

    /*!
     * Default constructor
     */
    Beidou_Cnav2_Ephemeris();
};

#endif
