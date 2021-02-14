#include "../utility/log.h"
#include "calcfeatures.h"
#include <mutex>
#include <random>
#include <numeric>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include "csvreader.h"

#ifdef DEBUG_CALCULATE_FEATS
#undef DEBUG_CALCULATE_FEATS
static const string DEBUG_FOLDER = "/var/tmp/water/";
#endif

using namespace cv;
using namespace std;
using namespace boost;

namespace fs = filesystem;

static mutex g_image_read_mutex;


static bool is_number(const string& s)
{
    return !s.empty() && find_if(s.begin(),
        s.end(), [](char c) { return !(isdigit(c)||c=='.'||c=='-'); }) == s.end();
}
double DISTANCE( const Point2d a, const Point2d b ) { return static_cast< double >( sqrt( ( b.x - a.x ) * ( b.x - a.x ) +
                                                                                          ( b.y - a.y ) * ( b.y - a.y ) ) ); }

namespace gc
{

GC_STATUS PickNRandom( const int total, const int numToPick, vector< int > &picks );

CalcFeatures::CalcFeatures()
{

}
GC_STATUS CalcFeatures::WriteFeatSetToCSV( const string filepath, const FeatureSet &featSet )
{
    GC_STATUS retVal = CreateCSVFileAndHeader( filepath, featSet );
    if ( GC_OK == retVal )
    {
        try
        {
            ofstream outFile( filepath, ios_base::out | ios_base::app );
            if ( !outFile.is_open() )
            {
                FILE_LOG( logERROR ) << "[CalcFeatures::WriteFeatSetToCSV] Could not open CSV: " << filepath;
                retVal = GC_ERR;
            }
            else
            {
                outFile << std::fixed;
                outFile << featSet.sensor.timeStamp << ",";
                outFile << featSet.exif.captureTime << ",";
                outFile << featSet.imageFilename << ",";
                outFile << featSet.sensor.agency << ",";
                outFile << featSet.sensor.siteNumber << ",";
                outFile << featSet.sensor.timeZone << ",";
                outFile << featSet.sensor.stage << ",";
                outFile << featSet.sensor.discharge << ",";
                outFile << featSet.calcTimestamp << ",";
                outFile << featSet.wholeImage.imageSize.width << ",";
                outFile << featSet.wholeImage.imageSize.height << ",";
                outFile << featSet.exif.exposureTime << ",";
                outFile << featSet.exif.fNumber << ",";
                outFile << featSet.exif.isoSpeedRating << ",";
                outFile << featSet.exif.shutterSpeed << ",";
                outFile << featSet.wholeImage.grayStats.average << ",";
                outFile << featSet.wholeImage.grayStats.sigma << ",";
                outFile << featSet.wholeImage.entropyStats.average << ",";
                outFile << featSet.wholeImage.entropyStats.sigma << ",";
                if ( 3 != featSet.wholeImage.hsvStats.size() )
                {
                    outFile << "N/A,N/A,N/A,N/A,N/A,N/A," << endl;
                }
                else
                {
                    outFile << featSet.wholeImage.hsvStats[ 0 ].average << ",";
                    outFile << featSet.wholeImage.hsvStats[ 0 ].sigma << ",";
                    outFile << featSet.wholeImage.hsvStats[ 1 ].average << ",";
                    outFile << featSet.wholeImage.hsvStats[ 1 ].sigma << ",";
                    outFile << featSet.wholeImage.hsvStats[ 2 ].average << ",";
                    outFile << featSet.wholeImage.hsvStats[ 2 ].sigma << ",";
                }
                outFile << featSet.areaFeats.size() << ",";
                for ( size_t i = 0; i < featSet.areaFeats.size(); ++i )
                {
                    outFile << featSet.areaFeats[ i ].grayStats.average << ",";
                    outFile << featSet.areaFeats[ i ].grayStats.sigma << ",";
                    outFile << featSet.areaFeats[ i ].entropyStats.average << ",";
                    outFile << featSet.areaFeats[ i ].entropyStats.sigma;
                    if ( 3 != featSet.areaFeats[ i ].hsvStats.size() )
                    {
                        outFile << "N/A,N/A,N/A,N/A,N/A,N/A" << endl;
                    }
                    else
                    {
                        outFile << featSet.areaFeats[ i ].hsvStats[ 0 ].average << ",";
                        outFile << featSet.areaFeats[ i ].hsvStats[ 0 ].sigma << ",";
                        outFile << featSet.areaFeats[ i ].hsvStats[ 1 ].average << ",";
                        outFile << featSet.areaFeats[ i ].hsvStats[ 1 ].sigma << ",";
                        outFile << featSet.areaFeats[ i ].hsvStats[ 2 ].average << ",";
                        outFile << featSet.areaFeats[ i ].hsvStats[ 2 ].sigma;
                    }
                    if ( featSet.areaFeats.size() - 1 > i )
                        outFile << ",";
                }
            }
            outFile.close();
        }
        catch( const boost::exception &e )
        {
            FILE_LOG( logERROR ) << "[CalcFeatures::WriteFeatSetToCSV] " << diagnostic_information( e );
            retVal = GC_EXCEPT;
        }
    }
    return retVal;
}
GC_STATUS CalcFeatures::CreateCSVFileAndHeader( const string filepath, const FeatureSet &featSet )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        ofstream outFile;
        if ( !fs::exists( filepath ) )
        {
            if ( !fs::exists( fs::path( filepath ).parent_path() ) )
            {
                 bool bRet = fs::create_directories( fs::path( filepath ).parent_path() );
                 if ( !bRet )
                 {
                     FILE_LOG( logERROR ) << "[CalcFeatures::CreateCSVFileAndHeader] Could not create folder for CSV file: " << filepath;
                     retVal = GC_ERR;
                 }
            }
            if ( GC_OK == retVal )
            {
                outFile.open( filepath, ios_base::out );
                if ( !outFile.is_open() )
                {
                    FILE_LOG( logERROR ) << "[CalcFeatures::CreateCSVFileAndHeader] Could not create CSV: " << filepath;
                    retVal = GC_ERR;
                }
                else
                {
                    outFile << "SensorTime, CaptureTime, Filename, Agency, SiteNumber, TimeZone, Stage, Discharge, ";
                    outFile << "CalcTimestamp, width, height, exposure, fNumber, isoSpeed, shutterSpeed, grayMean, ";
                    outFile << "graySigma, entropyMean, entropySigma, hMean, hSigma, sMean, sSigma, vMean, vSigma, ";
                    outFile << "areaFeatCount";
                    if ( 0 == featSet.areaFeats.size() )
                    {
                        outFile << endl;
                    }
                    else
                    {
                        outFile << ", ";
                        for ( size_t i = 0; i < featSet.areaFeats.size(); ++i )
                        {
                            outFile << "grayMean " << i << ", graySigma " << i << ", ";
                            outFile << "entropyMean " << i << ", entropySigma " << i << ", ";
                            outFile << "hMean " << i << ", hSigma " << i << ", ";
                            outFile << "sMean " << i << ", sSigma " << i << ", ";
                            outFile << "vMean " << i << ", vSigma " << i << ", ";
                        }
                    }
                    outFile << "WeirAngle, WeirPt1X, WeirPt1Y, WeirPt2X, WeirPt2Y, WwRawLineMin, WwRawLineMax, WwRawLineMean, ";
                    outFile << "WwRawLineSigma, WwCurveLineMin, WwCurveLineMax, WwCurveLineMean, WwCurveLineSigma" << endl;
                }
                outFile.close();
            }
        }
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalcFeatures::CreateCSVFileAndHeader] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS CalcFeatures::ParseRow( const vector< string > data, FeatureSet &feat )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        int j = 0;
        PixelStats pixStats;

        feat.clear();
        feat.sensor.timeStamp = data[ j++ ];
        feat.exif.captureTime = data[ j++ ];
        feat.imageFilename = data[ j++ ];
        feat.sensor.agency = data[ j++ ];
        feat.sensor.siteNumber = data[ j++ ];
        feat.sensor.timeZone = data[ j++ ];
        feat.sensor.stage = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
        feat.sensor.discharge = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;

        feat.calcTimestamp = data[ j++ ];
        feat.wholeImage.imageSize.width = is_number( data[ j ] ) ? stoi( data[ j ] ) : -9999999; ++j;
        feat.wholeImage.imageSize.height = is_number( data[ j ] ) ? stoi( data[ j ] ) : -9999999; ++j;
        feat.exif.exposureTime = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
        feat.exif.fNumber = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
        feat.exif.isoSpeedRating = is_number( data[ j ] ) ? stoi( data[ j ] ) : -9999999; ++j;
        feat.exif.shutterSpeed = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;

        feat.wholeImage.grayStats.average = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
        feat.wholeImage.grayStats.sigma = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;

        feat.wholeImage.entropyStats.average = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
        feat.wholeImage.entropyStats.sigma = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;

        pixStats.average = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
        pixStats.sigma = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
        feat.wholeImage.hsvStats.push_back( pixStats );

        pixStats.average = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
        pixStats.sigma = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
        feat.wholeImage.hsvStats.push_back( pixStats );

        pixStats.average = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
        pixStats.sigma = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
        feat.wholeImage.hsvStats.push_back( pixStats );

        ImageAreaFeatures areaFeat;
        int areaFeatCount  = is_number( data[ j ] ) ? stoi( data[ j ] ) : 0; ++j;
        for ( size_t i = 0; i < static_cast< size_t >( areaFeatCount ); ++i )
        {
            areaFeat.clear();

            areaFeat.grayStats.average = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
            areaFeat.grayStats.sigma = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;

            areaFeat.entropyStats.average = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
            areaFeat.entropyStats.sigma = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;

            pixStats.average = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
            pixStats.sigma = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
            areaFeat.hsvStats.push_back( pixStats );

            pixStats.average = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
            pixStats.sigma = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
            areaFeat.hsvStats.push_back( pixStats );

            pixStats.average = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
            pixStats.sigma = is_number( data[ j ] ) ? stod( data[ j ] ) : -9999999; ++j;
            areaFeat.hsvStats.push_back( pixStats );

            feat.areaFeats.push_back( areaFeat );
        }
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalcFeatures::ReadCSV] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS CalcFeatures::ReadCSV( const string filepath, vector< FeatureSet > &featSets )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        CSVReader reader( filepath );
        vector< vector< string > > data = reader.getData();
        if ( data.empty() )
        {
            FILE_LOG( logERROR ) << "[CalcFeatures::ReadCSV] No data in file " << filepath;
            retVal = GC_ERR;
        }
        else
        {
            FeatureSet feat;
            size_t count = data[ 0 ].size();
            for ( size_t i = 1; i < data.size(); ++i )
            {
                cout << "Parse features: " << i << " of " << data.size() << "\r";
                if ( count <= data[ i ].size() )
                {
                    retVal = ParseRow( data[ i ], feat );
                    if ( GC_OK == retVal )
                    {
                        featSets.push_back( feat );
                    }
                }
            }
        }
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalcFeatures::ReadCSV] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
// TODO: Fix this later -- KWC
/*
GC_STATUS CalcFeatures::MergeSensorData( const std::string featFilepath, const std::string sensorDataFilepath,
                                         const std::string mergeFilepath, const bool isSensorData, const std::string noSensorDataFilepath )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        SensorData sensor;
        retVal = isSensorData ? sensor.Load( sensorDataFilepath, site ) :
                                sensor.LoadManualData( sensorDataFilepath, site );
        if ( GC_OK == retVal )
        {
            CSVReader reader( featFilepath );
            vector< vector< string > > data = reader.getData();
            if ( data.empty() )
            {
                FILE_LOG( logERROR ) << "[CalcFeatures::MergeSensorData] No data in file " << featFilepath;
                retVal = GC_ERR;
            }
            else
            {
                FeatureSet feat;
                size_t startIdx = 0;

                size_t count = data[ 0 ].size();
                double elapsed_seconds = 0.0;
                auto start = std::chrono::system_clock::now();
                fs::remove( fs::path( mergeFilepath ) );

                for ( size_t i = 1; i < data.size(); ++i )
                {
                    cout << "Parse features: " << i << " of " << data.size() << "\r";
                    if ( data[ i ].size() != count )
                    {
                        FILE_LOG( logERROR ) << "[CalcFeatures::MergeSensorData] row data columns not equal to header columns for row " << i;
                        retVal = GC_ERR;
                        break;
                    }
                    else
                    {
                        retVal = ParseRow( data[ i ], feat );
                        if ( GC_OK != retVal )
                            break;

                        retVal = sensor.GetNearestSensorData( feat, startIdx, site );
                        if ( GC_OK != retVal )
                        {
                            startIdx = 0;
                            retVal = sensor.GetNearestSensorData( feat, startIdx, site );
                            if ( GC_OK == retVal )
                            {
                                retVal = sensor.GetNearestSensorData( feat, startIdx, site );
                            }
                        }
                        if ( GC_OK == retVal )
                        {
                            retVal = WriteFeatSetToCSV( mergeFilepath, feat );
                            if ( GC_OK != retVal )
                                break;

                            elapsed_seconds = std::chrono::duration_cast< std::chrono::seconds >( std::chrono::system_clock::now() - start ).count();
                            FILE_LOG( logINFO ) << "[" << i << " of " << data.size() << " Secs=" << elapsed_seconds << "] " << feat.imageFilename << ": " << ( GC_OK == retVal ? "SUCCESS" : "FAILURE" );
                        }
                        else if ( GC_WARN == retVal )
                        {
                            retVal = WriteFeatSetToCSV( noSensorDataFilepath, feat );
                            if ( GC_OK != retVal )
                                break;
                        }
                    }
                }
            }
        }
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalcFeatures::ReadCSV] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
*/
GC_STATUS CalcFeatures::Calculate( const string filepath, FeatureSet &featSet, const std::string saveResultFolder )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( !fs::exists( filepath ) )
        {
            FILE_LOG( logERROR ) << "[CalcFeatures::Calculate] File does not exist: " << filepath;
            retVal = GC_ERR;
        }
        else if( !fs::is_regular_file( filepath ) )
        {
            FILE_LOG( logERROR ) << "[CalcFeatures::Calculate] File is not a regular file: " << filepath;
            retVal = GC_ERR;
        }
        else if ( ".jpg" != filepath.substr( filepath.size() - 4 ) &&
                  ".JPG" != filepath.substr( filepath.size() - 4 ) )
        {
            FILE_LOG( logERROR ) << "[CalcFeatures::Calculate] File is not an image file: " << filepath;
            retVal = GC_ERR;
        }
        else
        {
            Mat src, raw, scratch, imgColor;
            string data;

            {
                lock_guard< mutex > guard( g_image_read_mutex );
                retVal = m_exif.Retrieve( filepath, data, featSet.exif );
                if ( GC_OK == retVal )
                {
                    raw = imread( filepath, IMREAD_COLOR );
                    if ( raw.empty() )
                    {
                        FILE_LOG( logERROR ) << "[CalcFeatures::Calculate] Could not read image : " << filepath;
                        retVal = GC_ERR;
                    }
                    else
                    {
                        imgColor = raw.clone();
                        cvtColor( imgColor, src, COLOR_BGR2GRAY );
                    }
                }
            }
#if 0
            if ( GC_OK == retVal )
            {
#ifdef DEBUG_CALCULATE_FEATS
                imwrite( DEBUG_FOLDER + "weir_original.png", imgColor );
#endif
                featSet.imageFilename = fs::path( filepath ).filename().string();
                Rect rectDarkness = ( 0 == settings.darknessROI.width || 0 == settings.darknessROI.height ) ?
                            Rect( 0, src.rows >> 1, src.cols, ( src.rows >> 1 ) - 1 ) : settings.darknessROI;
                double darknessMean = mean( src( rectDarkness ) ).val[ 0 ];
                Canny( src, scratch, 35, 70, 3 );
                double edgesMean = mean( scratch ).val[ 0 ] / 255.0;
//                FILE_LOG( logINFO ) << "**********************";
//                FILE_LOG( logINFO ) << "Edges=" << edgesMean;
//                FILE_LOG( logINFO ) << "**********************";

                if ( m_settings.darknessMin > darknessMean || m_settings.darknessMax < darknessMean )
                {
                    featSet.status = FIND_BAD_TOO_DARK;
                    putText( imgColor, "FAIL: Image too dark to process", Point( 100, 100 ), FONT_HERSHEY_PLAIN, 2.0, Scalar( 0, 0, 255 ), 3 );
                    FILE_LOG( logERROR ) << "[CalcFeatures::Calculate] Image too dark to process for " << fs::path( filepath ).filename().string();
                    retVal = GC_ERR;
                }
                else if ( m_settings.edgesMin > edgesMean )
                {
                    featSet.status = FIND_LOW_EDGE_COUNT;
                    putText( imgColor, "FAIL: Image too low edge count to process", Point( 100, 100 ), FONT_HERSHEY_PLAIN, 2.0, Scalar( 0, 0, 255 ), 3 );
                    FILE_LOG( logERROR ) << "[CalcFeatures::Calculate] Image too dark to process for " << fs::path( filepath ).filename().string();
                    retVal = GC_ERR;
                }
                else
                {
                    if ( WHOLE_IMAGE_FEATURE_TYPE & featureTypes )
                    {
                        Mat mask;
                        featSet.wholeImage.imageSize = src.size();
                        retVal = m_imgFeats.CalcImageFeatures( imgColor, featSet.wholeImage, mask );
                        if ( GC_OK != retVal )
                        {
                            putText( imgColor, "FAIL: Whole image feature calculation", Point( 100, 100 ), FONT_HERSHEY_PLAIN, 2.0, Scalar( 0, 0, 255 ), 3 );
                            FILE_LOG( logERROR ) << "[CalcFeatures::Calculate] Could not calculate whole image features for " << filepath;
                        }
                    }

                    bool foundAreaFeature = false;
                    if ( GC_OK == retVal && ( AREA_FEATURE_TYPE & featureTypes ) )
                    {
                        retVal = m_imgFeats.CalcMaskedFeatures( imgColor, settings.areaFeatRois, settings.areaFeatShapes, featSet.areaFeats );
                        if ( GC_OK == retVal )
                        {
                            foundAreaFeature = true;
                        }
                        else
                        {
                            putText( imgColor, "FAIL: Area features", Point( 100, 200 ), FONT_HERSHEY_PLAIN, 2.0, Scalar( 0, 0, 255 ), 3 );
                            FILE_LOG( logERROR ) << "[CalcFeatures::Calculate] Could not calculate whole image features for " << filepath;
                        }
                    }

                    bool weirFound = false;
                    if ( GC_OK == retVal && ( WEIR_FEATURE_TYPE & featureTypes ) )
                    {
                        Mat belowWeirMask;
                        retVal = m_weir.Calculate( src, featSet.weir, settings.weirSettings, belowWeirMask );
                        if ( GC_OK == retVal )
                        {
                            weirFound = true;
                            if ( !saveResultFolder.empty() )
                            {
                                line( imgColor, featSet.weir.weirLine.start, featSet.weir.weirLine.end, Scalar( 255, 0, 0 ), 3 );
                                for ( size_t k = 0; k < featSet.weir.whitewater.lineFeats.size(); ++k )
                                {
                                    line( imgColor, featSet.weir.whitewater.lineFeats[ k ].ends.start,
                                          featSet.weir.whitewater.lineFeats[ k ].ends.end, Scalar( 0, 255, 255 ), 1 );
                                }
                                vector< Point > wwOutline;
                                for ( size_t k = 0; k < featSet.weir.whitewater.lines.size(); ++k )
                                {
                                    line( imgColor, featSet.weir.whitewater.lines[ k ].start,
                                          featSet.weir.whitewater.lines[ k ].end, Scalar( 0, 255, 0 ), 1 );
                                    wwOutline.push_back( featSet.weir.whitewater.lines[ k ].end );
                                }
                                polylines( imgColor, wwOutline, true, Scalar( 0, 0, 255 ), 3 );
                            }

                            // FILE_LOG( logINFO ) << "Right bank point x=" << featSet.weir.rightBankPt.x << " y=" << featSet.weir.rightBankPt.y;
                            if ( m_settings.weirSettings.weirAngleMin > featSet.weir.angle ||
                                 m_settings.weirSettings.weirAngleMax < featSet.weir.angle )
                            {
                                featSet.status = FIND_BAD_WEIR_ANGLE;
                                putText( imgColor, "FAIL: Bad weir angle found", Point( 100, 100 ), FONT_HERSHEY_PLAIN, 2.0, Scalar( 0, 0, 255 ), 3 );
                                FILE_LOG( logERROR ) << "[CalcFeatures::Calculate] Bad weir angle=" << featSet.weir.angle << " found for " << fs::path( filepath ).filename().string();
                                retVal = GC_ERR;
                            }
                        }
                        if ( GC_OK != retVal )
                        {
                            featSet.weir.clear();
                            retVal = GC_OK;
                        }
                        featSet.status = FIND_OK;
                        if ( !saveResultFolder.empty() )
                        {
                            if ( foundAreaFeature )
                            {
                                for ( size_t i = 0; i < settings.areaFeatRois.size(); ++i )
                                    polylines( imgColor, settings.areaFeatRois[ i ], true,
                                               i % 2 ? Scalar( 255, 0, 0 ) :  Scalar( 0, 255, 255 ), 3 );
                            }

                            string fileResult = saveResultFolder;
                            if ( '/' != fileResult[ fileResult.size() - 1 ] )
                                fileResult += '/';
#if 0
                            fileResult += string( GC_OK == retVal ? "OK_" : "FAIL_" ) + featSet.imageFilename;
                            imwrite( fileResult, imgColor );
#else

                            Mat resizeScratch;
                            Mat scratch( Size( 1072, 1444 ), CV_8UC3 );
                            scratch = 255;
                            Mat top = scratch( Rect( 0, 0, 1072, 712 ) );
                            Mat bot = scratch( Rect( 0, 732, 1072, 712 ) );
                            resize( raw, resizeScratch, Size( 1072, 712 ) );
                            resizeScratch.copyTo( top );
                            if ( featSet.status == FIND_BAD_TOO_DARK )
                            {
                                circle( bot, Point( bot.cols >> 1, bot.rows >> 1 ), 250, Scalar( 0, 0, 255 ), 50 );
                                line( bot, Point( ( bot.cols >> 1 ) - 175, ( bot.rows >> 1 ) - 175 ), Point( ( bot.cols >> 1 ) + 175, ( bot.rows >> 1 ) + 175 ), Scalar( 0, 0, 255 ), 50 );
                                putText( bot, "Could not process", Point( 100, 100 ), FONT_HERSHEY_PLAIN, 2.4, Scalar( 0, 255, 255 ), 5 );
                            }
                            else
                            {
                                resize( imgColor, resizeScratch, Size( 1072, 712 ) );
                                resizeScratch.copyTo( bot );
                                if ( !weirFound )
                                {
                                    putText( bot, "Weir not found", Point( 100, 100 ), FONT_HERSHEY_PLAIN, 2.0, Scalar( 0, 255, 255 ), 3 );
                                }
                            }
                            fileResult += featSet.imageFilename;
                            imwrite( fileResult, scratch );
#endif
                        }
                    }
                }
            }
#endif
        }
    }
    catch( const Exception &e )
    {
        FILE_LOG( logERROR ) << "[CalcFeatures::Calculate] " << e.what();
        retVal = GC_EXCEPT;
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalcFeatures::Calculate] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}

GC_STATUS CalcFeatures::SplitTestTrainSets( const string allCSV, const string setFolder, const int beforeCount,
                                               const int afterCount, const size_t timeStampCol,
                                               const string testStartTimeStamp, const string testEndTimeStamp )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        CSVReader reader( allCSV );
        vector< vector< string > > data = reader.getData();

        if ( data.empty() )
        {
            FILE_LOG( logERROR ) << "[CalcFeatures::SplitTestTrainSets] Could not load data from " << allCSV;
            retVal = GC_ERR;
        }
        else
        {
            int endTest = -1;
            int startTest = -1;
            for ( size_t i = 0; i < data.size(); ++i )
            {
                if ( testStartTimeStamp == data[ i ][ timeStampCol ] )
                    startTest = static_cast< int >( i );
                if ( testEndTimeStamp == data[ i ][ timeStampCol ] )
                    endTest = static_cast< int >( i );
                if ( -1 != startTest && -1 != endTest )
                    break;
            }
            if ( -1 == startTest || -1 == endTest )
            {
                FILE_LOG( logERROR ) << "[CalcFeatures::SplitTestTrainSets] Could not find start and/or end dates: " << testStartTimeStamp << " to " << testEndTimeStamp;
                retVal = GC_ERR;
            }
            else
            {
                int beforeStart = startTest - beforeCount;
                int beforeEnd = startTest;
                int afterStart = endTest + 1;
                int afterEnd = endTest + afterCount + 1;

                string resultFolder = setFolder;
                if ( '/' != resultFolder[ resultFolder.size() - 1 ] )
                    resultFolder += '/';

                if ( !fs::exists( resultFolder ) )
                    fs::create_directories( resultFolder );

                string trainCSV = resultFolder + "train_" + to_string( beforeCount ) + "_" +
                        to_string( afterCount ) + "_" + fs::path( allCSV ).filename().string();
                string testCSV = resultFolder + "test_" + to_string( beforeCount ) + "_" +
                        to_string( afterCount ) + "_" + fs::path( allCSV ).filename().string();

                ofstream outFile( trainCSV );
                if ( !outFile.is_open() )
                {
                    FILE_LOG( logERROR ) << "[CalcFeatures::SplitTestTrainSets] Could not open train set file for writing: " << trainCSV;
                    retVal = GC_ERR;
                }
                else
                {
                    for ( size_t i = 0; i < data[ 0 ].size(); ++i )
                    {
                        outFile << data[ 0 ][ i ];
                        if ( data[ 0 ].size() - 1 == i )
                            outFile << endl;
                        else
                            outFile << ",";
                    }
                    for ( int j = beforeStart; j < beforeEnd; ++j )
                    {
                        for ( size_t i = 0; i < data[ j ].size(); ++i )
                        {
                            outFile << data[ j ][ i ];
                            if ( data[ j ].size() - 1 == i )
                                outFile << endl;
                            else
                                outFile << ",";
                        }
                    }
                    for ( int j = afterStart; j < afterEnd; ++j )
                    {
                        for ( size_t i = 0; i < data[ j ].size(); ++i )
                        {
                            outFile << data[ j ][ i ];
                            if ( data[ j ].size() - 1 == i )
                                outFile << endl;
                            else
                                outFile << ",";
                        }
                    }
                    outFile.close();
                    ofstream outFile( testCSV );
                    if ( !outFile.is_open() )
                    {
                        FILE_LOG( logERROR ) << "[CalcFeatures::SplitTestTrainSets] Could not open test set file for writing: " << testCSV;
                        retVal = GC_ERR;
                    }
                    else
                    {
                        for ( size_t i = 0; i < data[ 0 ].size(); ++i )
                        {
                            outFile << data[ 0 ][ i ];
                            if ( data[ 0 ].size() - 1 == i )
                                outFile << endl;
                            else
                                outFile << ",";
                        }
                        for ( int j = startTest; j < endTest; ++j )
                        {
                            for ( size_t i = 0; i < data[ j ].size(); ++i )
                            {
                                outFile << data[ j ][ i ];
                                if ( data[ j ].size() - 1 == i )
                                    outFile << endl;
                                else
                                    outFile << ",";
                            }
                        }
                        outFile.close();
                    }
                }
            }
        }
    }
    catch( boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalcFeatures::SplitTestTrainSets] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }

    return retVal;
}

GC_STATUS CalcFeatures::SplitTestTrainSets( const string allCSV, const double percentTrain,
                                               const string trainCSV, const string testCSV )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        ifstream inFile( allCSV );
        if ( !inFile.is_open() )
        {
            FILE_LOG( logERROR ) << "[CalcFeatures::SplitTestTrainSets] Could not open for reading " << allCSV;
            retVal = GC_ERR;
        }
        else
        {
            string line;
            vector< string > rows;
            while( getline( inFile, line ) )
                rows.push_back( line );
            inFile.close();

            if ( rows.empty() )
            {
                FILE_LOG( logERROR ) << "[CalcFeatures::SplitTestTrainSets] No data inspecified source file " << allCSV;
                retVal = GC_ERR;
            }
            else
            {
                vector< int > trainRows;
                string header = rows[ 0 ];
                retVal = PickNRandom( static_cast< int >( rows.size() ), cvRound( percentTrain * rows.size() ), trainRows );
                if ( GC_OK == retVal )
                {
                    vector< int > allRows;
                    for( size_t i = 1; i < rows.size(); ++i )
                    {
                        allRows.push_back( static_cast< int >( i ) );
                    }

                    for( size_t i = 1; i < trainRows.size(); ++i )
                    {
                        allRows[ trainRows[ i ] ] *= -1;
                    }

                   vector< string > rowSet;
                    for ( size_t i = 0; i < allRows.size(); ++i )
                    {
                        if ( 0 > allRows[ i ] )
                            rowSet.push_back( rows[ -allRows[ i ] ] );
                    }

                    ofstream outFile( trainCSV, ios_base::out );
                    if ( !outFile.is_open() )
                    {
                        FILE_LOG( logERROR ) << "[CalcFeatures::SplitTestTrainSets] Could not open CSV: " << trainCSV;
                        retVal = GC_ERR;
                    }
                    else
                    {
                        outFile << rows[ 0 ] << endl;
                        for ( size_t i = 0; i < rowSet.size(); ++i )
                        {
                            outFile << rowSet[ i ] << endl;
                        }
                        outFile.close();
                    }
                    FILE_LOG( logINFO ) << "Training set=" << rowSet.size();

                    rowSet.clear();
                    for ( size_t i = 0; i < allRows.size(); ++i )
                    {
                        if ( 0 < allRows[ i ] )
                            rowSet.push_back( rows[ allRows[ i ] ] );
                    }

                    outFile.open( testCSV, ios_base::out );
                    if ( !outFile.is_open() )
                    {
                        FILE_LOG( logERROR ) << "[CalcFeatures::SplitTestTrainSets] Could not open CSV: " << testCSV;
                        retVal = GC_ERR;
                    }
                    else
                    {
                        outFile << rows[ 0 ] << endl;
                        for ( size_t i = 0; i < rowSet.size(); ++i )
                        {
                            outFile << rowSet[ i ] << endl;
                        }
                        outFile.close();
                    }
                    FILE_LOG( logINFO ) << "Test set=" << rowSet.size();
                }
            }
        }
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[CalcFeatures::SplitTestTrainSets] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS PickNRandom( const int total, const int numToPick, vector< int > &picks )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        random_device rd;
        mt19937 rng( rd() );                       // seed rng using rd
        vector< int > data( total );               // create a 3-entry vector
        iota( data.begin(), data.end(), 0 );       // fill with sequence
        shuffle( data.begin(), data.end(), rng );  // mix entries using rng

        for ( size_t i = 0; i < static_cast< size_t >( numToPick ); ++i )
            picks.push_back( data[ i ] );
    }
    catch( const std::exception &e )
    {
        FILE_LOG( logERROR ) << "[PickNRandom] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}


} // namespace gc
