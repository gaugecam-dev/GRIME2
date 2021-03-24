/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   Copyright 2021 Kenneth W. Chapman

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/** \file metadata.h
 * @brief A file for a class to add/retrieve metadata to/from images
 *
 * \author Kenneth W. Chapman
 * \copyright Copyright (C) 2010-2020, Kenneth W. Chapman <coffeesig@gmail.com>, all rights reserved.\n
 * This project is released under the Apache License, Version 2.0.
 * \bug No known bugs.
 */

#ifndef METADATA_H
#define METADATA_H

#include "gc_types.h"
#include <boost/property_tree/ptree.hpp>
#include "featuredata.h"

namespace gc
{

// TODO: Write doxygen comments
//class ExifFeatures
//{
//public:
//    ExifFeatures() :
//        imageDims( -1, -1 ),
//        captureTime( "" ),
//        exposureTime( -1.0 ),
//        fNumber( -1.0 ),
//        isoSpeedRating( -1 ),
//        shutterSpeed( -1.0 )
//    {}

//    void clear()
//    {
//        imageDims = cv::Size( -1, -1 );
//        captureTime.clear();
//        exposureTime = -1.0;
//        fNumber = -1.0;
//        isoSpeedRating = -1;
//        shutterSpeed = -1.0;
//    }
//    cv::Size imageDims;
//    std::string captureTime;
//    double exposureTime;
//    double fNumber;
//    int isoSpeedRating;
//    double shutterSpeed;
//};

class MetaData
{
public:
    /**
     * @brief Constructor
     */
    MetaData() {}

    /**
     * @brief Destructor
     */
    ~MetaData() {}

    // TODO Write doxygen comments
    static const std::string Version() { return "0.0.0.1"; }
    static void GetExifToolVersion();
    GC_STATUS GetImageData( const std::string filepath, std::string &data );
    GC_STATUS GetImageData( const std::string filepath, ExifFeatures &exifFeat );
    GC_STATUS GetExifData( const std::string filepath, const std::string tag, std::string &data );

private:
    std::string ConvertToLocalTimestamp( const std::string exifTimestamp );
};

} // namespace gc

#endif // METADATA_H
