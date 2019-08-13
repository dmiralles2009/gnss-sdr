/*!
 * \file beidou_cnav1_iono.h
 * \brief  Interface of a BeiDou CNAV1 Ionospheric Model storage
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


#ifndef GNSS_SDR_BEIDOU_CNAV1_IONO_H_
#define GNSS_SDR_BEIDOU_CNAV1_IONO_H_

#include <boost/serialization/nvp.hpp>

/*!
 * \brief This class is a storage for the BeiDou CNAV1 IONOSPHERIC data as described in Beidou B1C ICD
 */
class Beidou_Cnav1_Iono
{
public:
    // Ionospheric correction
    bool valid;  //!< Valid flag

    // BeiDou Global Ionospheric delay correction Model (BDGIM) parameters
    double alpha1;  //!< Coefficient 1 of the BDGIM model [TECu]
    double alpha2;  //!< Coefficient 2 of the BDGIM model [TECu]
    double alpha3;  //!< Coefficient 3 of the BDGIM model [TECu]
    double alpha4;  //!< Coefficient 3 of the BDGIM model [TECu]
    double alpha5;  //!< Coefficient 3 of the BDGIM model [TECu]
    double alpha6;  //!< Coefficient 3 of the BDGIM model [TECu]
    double alpha7;  //!< Coefficient 3 of the BDGIM model [TECu]
    double alpha8;  //!< Coefficient 3 of the BDGIM model [TECu]
    double alpha9;  //!< Coefficient 3 of the BDGIM model [TECu]

    // from message type 1 of Subframe 3, get the iono correction parameters
    double d_TOW_1;  //!< BDT data reference Time of Week [s]
    double d_WN_1;   //!< BDT data reference Week number [week]

    /*!
     * Default constructor
     */
    Beidou_Cnav1_Iono();

    template <class Archive>

    /*!
     * \brief Serialize is a boost standard method to be called by the boost XML serialization.
     Here is used to save the iono data on disk file.
     */
    inline void serialize(Archive& archive, const unsigned int version)
    {
        using boost::serialization::make_nvp;
        if (version)
            {
            };
        archive& make_nvp("d_alpha1", alpha1);
        archive& make_nvp("d_alpha2", alpha2);
        archive& make_nvp("d_alpha3", alpha3);
        archive& make_nvp("d_alpha4", alpha4);
        archive& make_nvp("d_alpha5", alpha5);
        archive& make_nvp("d_alpha6", alpha6);
        archive& make_nvp("d_alpha7", alpha7);
        archive& make_nvp("d_alpha8", alpha8);
        archive& make_nvp("d_alpha9", alpha9);
        archive& make_nvp("TOW_1", d_TOW_1);
        archive& make_nvp("WN_1", d_WN_1);
    }
};

#endif
