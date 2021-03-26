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
 * \copyright Copyright (C) 2010-2021, Kenneth W. Chapman <coffeesig@gmail.com>, all rights reserved.\n
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

/**
 * @brief Class to read metadata from images
 */
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

    /**
     * @brief Retreive the current software version of MetaData class
     * @return String holding the version
     */
    static const std::string Version() { return "0.0.0.1"; }

    /**
     * @brief Print to stdout the version of the ExifTool program used to retrieve metadaa
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    static void GetExifToolVersion();

    /**
     * @brief Retrieve the metadata into a human readable string
     * @param filepath The filepath of the image file from which to retrieve the metadata
     * @param data String to hold the retrieved metadata
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS GetImageData( const std::string filepath, std::string &data );

    /**
     * @brief Retrieve the metadata into an instance of the ExifFeatures data class
     * @param filepath The filepath of the image file from which to retrieve the metadata
     * @param exifFeat Instance of the ExifFeatures data class to hold the retrieved metadata
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS GetImageData( const std::string filepath, ExifFeatures &exifFeat );

    /**
     * @brief Retrieve the metadata for a specific tag from an image file
     * @param filepath The filepath of the image file from which to retrieve the metadata
     * @param tag Name of tag to be retrieved
     * @param data String to hold the retrieved metadata
     * @return GC_OK=Success, GC_FAIL=Failure, GC_EXCEPT=Exception thrown
     */
    GC_STATUS GetExifData( const std::string filepath, const std::string tag, std::string &data );

private:
    std::string ConvertToLocalTimestamp( const std::string exifTimestamp );
};

} // namespace gc

#endif // METADATA_H
