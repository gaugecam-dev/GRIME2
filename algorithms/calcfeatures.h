#ifndef CALCFEATURES_H
#define CALCFEATURES_H

#include "exifmetadata.h"
#include "areafeatures.h"
#include "entropymap.h"

namespace gc
{

class CalcFeatures
{
public:
    CalcFeatures();

    GC_STATUS Calculate( const std::string filepath, FeatureSet &featSet, const std::string saveResultFolder = "" );
    GC_STATUS CreateCSVFileAndHeader( const std::string filepath, const FeatureSet &featSet );
    GC_STATUS WriteFeatSetToCSV( const std::string filepath, const FeatureSet &featSet );
    GC_STATUS ReadCSV( const std::string filepath, std::vector< FeatureSet > &featSets );
    GC_STATUS MergeSensorData( const std::string featFilepath, const std::string sensorDataFilepath,
                               const std::string mergeFilepath,  const bool isSensorData, const std::string noSensorDataFilepath = "" );
    GC_STATUS SplitTestTrainSets( const std::string allCSV, const double percentTrain,
                                     const std::string trainCSV, const std::string testCSV );
    GC_STATUS SplitTestTrainSets( const std::string allCSV, const std::string setFolder, const int beforeCount,
                                     const int afterCount, const size_t timeStampCol, const std::string testStartTimeStamp,
                                     const std::string testEndTimeStamp );

private:

    ExifMetadata m_exif;
    AreaFeatures m_imgFeats;
    EntropyMap m_entropy;

    GC_STATUS ParseRow( const std::vector< std::string > data, FeatureSet &feat );

};

} // namespace water

#endif // CALCFEATURES_H
