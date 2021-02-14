#ifndef FEATURES_H
#define FEATURES_H

#include "gc_types.h"
#include <string>
#include <fstream>
#include <opencv2/core.hpp>
#include "featuredata.h"

namespace gc
{

class Features
{
public:
    Features();

    GC_STATUS clear();
    GC_STATUS Add( const FeatureSet &featureSet );
    GC_STATUS WriteToJson( const std::string filepath );
    GC_STATUS ReadFromJson( const std::string filepath );
    GC_STATUS WriteToCSV( const std::string filepath );
    GC_STATUS AddToCSV( const std::string filepath, const std::vector< FeatureSet > &featSets );
    GC_STATUS AddToCSV( const std::string filepath, const FeatureSet &featSet );
    GC_STATUS NewCSV( const std::string filepath );

    GC_STATUS FindDuplicates( std::vector< std::pair< size_t, std::vector< size_t > > > duplicatePairs );
    GC_STATUS RemoveRow( size_t row );

private:
    std::vector< FeatureSet > m_features;

    GC_STATUS CreateFoldersForFile( const std::string filepath );
    GC_STATUS WriteFeatureSetRow( std::ofstream &outStream, const FeatureSet &featSet );
};

} // namespace gc

#endif // FEATURES_H
