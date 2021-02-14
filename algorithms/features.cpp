#include "log.h"
#include "features.h"
#include <boost/filesystem.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using namespace std;
using namespace boost;

namespace fs = filesystem;
namespace pt = property_tree;

namespace gc
{

Features::Features()
{
}
GC_STATUS Features::clear()
{
    GC_STATUS retVal = GC_OK;

    try
    {
        m_features.clear();
    }
    catch( const std::exception &e )
    {
        FILE_LOG( logERROR ) << "[Features::clear][" << __func__ << "] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS Features::WriteFeatureSetRow( ofstream &outStream, const FeatureSet &featSet )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        if ( !outStream.is_open() )
        {
            FILE_LOG( logERROR ) << "[Features::WriteFeatureSetRow][" << __func__ << "] File stream not open";
            retVal = GC_ERR;
        }
        else
        {
            outStream << featSet.imageFilename << ",";
            outStream << featSet.exif.fNumber << ",";
            outStream << featSet.exif.imageDims.width << ",";
            outStream << featSet.exif.imageDims.height << ",";
            outStream << featSet.exif.captureTime << ",";
            outStream << featSet.exif.exposureTime << ",";
            outStream << featSet.exif.shutterSpeed << ",";
            outStream << featSet.exif.isoSpeedRating << ",";
            outStream << featSet.wholeImage.imageSize.width << ",";
            outStream << featSet.wholeImage.imageSize.height << ",";
            outStream << featSet.wholeImage.grayStats.average << ",";
            outStream << featSet.wholeImage.grayStats.sigma << ",";
            outStream << featSet.wholeImage.entropyStats.average << ",";
            outStream << featSet.wholeImage.entropyStats.sigma << ",";
            outStream << featSet.wholeImage.hsvStats[ 0 ].average << ",";
            outStream << featSet.wholeImage.hsvStats[ 0 ].sigma << ",";
            outStream << featSet.wholeImage.hsvStats[ 1 ].average << ",";
            outStream << featSet.wholeImage.hsvStats[ 1 ].sigma << ",";
            outStream << featSet.wholeImage.hsvStats[ 2 ].average << ",";
            outStream << featSet.wholeImage.hsvStats[ 2 ].sigma << endl;
            outStream.close();
        }
    }
    catch( const std::exception &e )
    {
        FILE_LOG( logERROR ) << "[Features::WriteFeatureSetRow][" << __func__ << "] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS Features::AddToCSV( const std::string filepath, const vector< FeatureSet > &featSets )
{
    GC_STATUS retVal = CreateFoldersForFile( filepath );

    if ( GC_OK == retVal )
    {
        try
        {
            if ( !fs::exists( filepath ) )
            {
                retVal = NewCSV( filepath );
                if ( GC_OK == retVal )
                {
                    FILE_LOG( logINFO ) << "File " << fs::path( filepath ).filename().string() <<
                                           " did not exist for AddToCSV, so it was created";
                }
                else
                {
                    FILE_LOG( logINFO ) << "File " << fs::path( filepath ).filename().string() <<
                                           " did not exist and could not be created during AddToCSV";
                    retVal = GC_ERR;
                }
            }
            bool allRowsWritten = true;
            if ( GC_OK == retVal )
            {
                ofstream outStream( filepath, ios::app );
                for ( size_t i = 0; i < featSets.size(); ++i )
                {
                    retVal = WriteFeatureSetRow( outStream, featSets[ i ] );
                    if ( GC_OK != retVal )
                    {
                        allRowsWritten = false;
                        FILE_LOG( logWARNING ) << "Could not write features to CSV for " << featSets[ i ].imageFilename;
                    }
                }
            }
            if ( !allRowsWritten )
            {
                FILE_LOG( logWARNING ) << "Not all feature set rows written properly";
                retVal = GC_WARN;
            }
        }
        catch( const std::exception &e )
        {
            FILE_LOG( logERROR ) << "[Features::AddToCSV][" << __func__ << "] " << e.what();
            retVal = GC_EXCEPT;
        }
    }
    return retVal;
}
GC_STATUS Features::AddToCSV( const std::string filepath, const FeatureSet &featSet )
{
    GC_STATUS retVal = CreateFoldersForFile( filepath );

    if ( GC_OK == retVal )
    {
        try
        {
            if ( !fs::exists( filepath ) )
            {
                retVal = NewCSV( filepath );
                if ( GC_OK == retVal )
                {
                    FILE_LOG( logINFO ) << "File " << fs::path( filepath ).filename().string() <<
                                           " did not exist for AddToCSV, so it was created";
                }
                else
                {
                    FILE_LOG( logINFO ) << "File " << fs::path( filepath ).filename().string() <<
                                           " did not exist and could not be created during AddToCSV";
                    retVal = GC_ERR;
                }
            }

            if ( GC_OK == retVal )
            {
                ofstream outStream( filepath, ios::app );
                retVal = WriteFeatureSetRow( outStream, featSet );
                if ( GC_OK != retVal )
                {
                    FILE_LOG( logWARNING ) << "Could not write features to CSV for " << featSet.imageFilename;
                }
            }
        }
        catch( const std::exception &e )
        {
            FILE_LOG( logERROR ) << "[Features::AddToCSV][" << __func__ << "] " << e.what();
            retVal = GC_EXCEPT;
        }
    }
    return retVal;
}
GC_STATUS Features::NewCSV( const std::string filepath )
{
    GC_STATUS retVal = CreateFoldersForFile( filepath );

    if ( GC_OK == retVal )
    {
        try
        {
            ofstream outStream( filepath );
            if ( !outStream.is_open() )
            {
                FILE_LOG( logERROR ) << "[Features::NewCSV][" << __func__ << "] File does not exist: " << filepath;
                retVal = GC_ERR;
            }
            else
            {
                outStream << "Image,"
                             "Timestamp (calc),"
                             "fNumber,"
                             "Exif width,"
                             "Exif height,"
                             "Timestamp (capture),"
                             "Exposure time,"
                             "Shutter speed,"
                             "ISO speed rating,"
                             "Actual width,"
                             "Actual height,"
                             "Gray mean,"
                             "Gray sigma,"
                             "Entropy mean,"
                             "Entropy sigma,"
                             "Hue mean,"
                             "Hue sigma,"
                             "Saturation mean,"
                             "Saturation sigma,"
                             "Value mean,"
                             "Value sigma" << endl;
                outStream.close();
            }
        }
        catch( const std::exception &e )
        {
            FILE_LOG( logERROR ) << "[Features::NewCSV][" << __func__ << "] " << e.what();
            retVal = GC_EXCEPT;
        }
    }
    return retVal;
}
GC_STATUS Features::Add( const FeatureSet &featureSet )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        m_features.push_back( featureSet );
    }
    catch( const std::exception &e )
    {
        FILE_LOG( logERROR ) << "[Features::Add][" << __func__ << "] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS Features::FindDuplicates( vector< pair< size_t, vector< size_t > > > duplicatePairs )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        duplicatePairs.clear();
        vector< size_t > duplicates;
        for ( size_t i = 0; i < m_features.size() - 1; ++i )
        {
            duplicates.clear();
            for ( size_t j = i + 1; j < m_features.size(); ++j )
            {
                if ( m_features[ i ].imageFilename == m_features[ j ].imageFilename )
                {
                    duplicates.push_back( j );
                }
            }
            if ( !duplicates.empty() )
            {
                duplicatePairs.push_back( pair< size_t, vector< size_t > >( i, duplicates ) );
            }
        }
    }
    catch( const std::exception &e )
    {
        FILE_LOG( logERROR ) << "[Features::FindDuplicates][" << __func__ << "] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS Features::RemoveRow( size_t row )
{
    GC_STATUS retVal = GC_OK;

    try
    {
    }
    catch( const std::exception &e )
    {
        FILE_LOG( logERROR ) << "[Features::RemoveDuplicates][" << __func__ << "] " << e.what();
        retVal = GC_EXCEPT;
    }
    return retVal;
}
GC_STATUS Features::WriteToJson( const string filepath )
{
    GC_STATUS retVal = CreateFoldersForFile( filepath );

    if ( GC_OK == retVal )
    {
        try
        {
            pt::ptree root, children;
            for ( size_t i = 0; i < m_features.size(); ++i )
            {
                pt::ptree child;
                child.put( "EXIF.fNumber", m_features[ i ].exif.fNumber );
                child.put( "EXIF.image.width", m_features[ i ].exif.imageDims.width );
                child.put( "EXIF.image.height", m_features[ i ].exif.imageDims.height );
                child.put( "EXIF.CaptureTime", m_features[ i ].exif.captureTime );
                child.put( "EXIF.ExposureTime", m_features[ i ].exif.exposureTime );
                child.put( "EXIF.ShutterSpeed", m_features[ i ].exif.shutterSpeed );
                child.put( "EXIF.ISOSpeedRating", m_features[ i ].exif.isoSpeedRating );
                children.push_back( make_pair( m_features[ i ].imageFilename, child ) );
            }
            root.add_child( "ImageArray", children );
            pt::write_json( filepath, root );
        }
        catch( const boost::exception &e )
        {
            FILE_LOG( logERROR ) << "[Features::WriteToJson][" << __func__ << "] " << diagnostic_information( e );
            retVal = GC_EXCEPT;
        }
        catch( const std::exception &e )
        {
            FILE_LOG( logERROR ) << "[Features::WriteToJson][" << __func__ << "] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
void print(boost::property_tree::ptree const& pt)
{
    using boost::property_tree::ptree;
    ptree::const_iterator end = pt.end();
    for (ptree::const_iterator it = pt.begin(); it != end; ++it)
    {
        FILE_LOG( logINFO ) << it->first << ": " << it->second.get_value<string>();
        print( it->second );
    }
}
GC_STATUS Features::ReadFromJson( const string filepath )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        if ( !fs::exists( filepath ) )
        {
            FILE_LOG( logERROR ) << "[Features::ReadFromJson][" << __func__ << "] Filepath does not exist: " << filepath;
            retVal = GC_ERR;
        }
        else if ( string::npos == filepath.substr( filepath.size() - 5 ).find( ".json" ) )
        {
            FILE_LOG( logERROR ) << "[Features::ReadFromJson][" << __func__ << "] Filepath must have .json extension: " << filepath;
            retVal = GC_ERR;
        }
        else
        {
            pt::ptree root;
            pt::read_json( filepath, root );

            m_features.clear();
            FeatureSet featSet;

            pt::ptree child;
            pt::ptree images = root.get_child( "ImageArray" );
            pt::ptree::const_iterator end = images.end();
            for ( pt::ptree::const_iterator image = images.begin(); image != end; ++image )
            {
                featSet.clear();
                featSet.imageFilename = image->first;
                featSet.exif.fNumber = stod( image->second.get< string >( "EXIF.fNumber" ) );
                featSet.exif.imageDims.width = stoi( image->second.get< string >( "EXIF.image.width" ) );
                featSet.exif.imageDims.height = stoi( image->second.get< string >( "EXIF.image.height" ) );
                featSet.exif.captureTime = image->second.get< string >( "EXIF.CaptureTime" );
                featSet.exif.exposureTime = stod( image->second.get< string >( "EXIF.ExposureTime" ) );
                featSet.exif.shutterSpeed = stod( image->second.get< string >( "EXIF.ShutterSpeed" ) );
                featSet.exif.isoSpeedRating = stoi( image->second.get< string >( "EXIF.ISOSpeedRating" ) );
                m_features.push_back( featSet );
            }
        }
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[Features::ReadFromJson][" << __func__ << "] " << diagnostic_information( e );
        retVal = GC_EXCEPT;
    }
    catch( const std::exception &e )
    {
        FILE_LOG( logERROR ) << "[Features::ReadFromJson][" << __func__ << "] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS Features::WriteToCSV( const string filepath )
{
    GC_STATUS retVal = CreateFoldersForFile( filepath );

    if ( GC_OK == retVal )
    {
        try
        {
            ofstream outStream( filepath );
            if ( !outStream.is_open() )
            {
                FILE_LOG( logERROR ) << "[Features::ReadFromJson][" << __func__ << "] File does not exist: " << filepath;
                retVal = GC_ERR;
            }
            else
            {
                outStream << "Filename, EXIF.fnumber, EXIF.image.width, EXIF.image.height, ";
                outStream << "EXIF.CaptureTime, EXIF.ExposureTime, EXIF.ShutterSpeed, EXIF.ISOSpeedRating " << endl;

                for ( size_t i = 0; i < m_features.size(); ++i )
                {
                    outStream << m_features[ i ].imageFilename << ",";
                    outStream << m_features[ i ].exif.fNumber << ",";
                    outStream << m_features[ i ].exif.imageDims.width << ",";
                    outStream << m_features[ i ].exif.imageDims.height << ",";
                    outStream << m_features[ i ].exif.captureTime << ",";
                    outStream << m_features[ i ].exif.exposureTime << ",";
                    outStream << m_features[ i ].exif.shutterSpeed << ",";
                    outStream << m_features[ i ].exif.isoSpeedRating << endl;
                }
                outStream.close();
            }
        }
        catch( const boost::exception &e )
        {
            FILE_LOG( logERROR ) << "[Features::WriteToCSV][" << __func__ << "] " << diagnostic_information( e );
            retVal = GC_EXCEPT;
        }
        catch( const std::exception &e )
        {
            FILE_LOG( logERROR ) << "[Features::WriteToCSV][" << __func__ << "] " << e.what();
            retVal = GC_EXCEPT;
        }
    }

    return retVal;
}
GC_STATUS Features::CreateFoldersForFile( const string filepath )
{
    GC_STATUS retVal = GC_OK;

    try
    {
        fs::path p( filepath );
        if ( !fs::exists( p.parent_path() ) )
        {
            bool isOK = fs::create_directories( p.parent_path() );
            if ( !isOK )
            {
                FILE_LOG( logERROR ) << "[Features::CreateFoldersForFile][" << __func__ << "] Could not create specified folders for " << filepath;
                retVal = GC_ERR;
            }
        }
    }
    catch( const std::exception &e )
    {
        FILE_LOG( logERROR ) << "[Features::CreateFoldersForFile][" << __func__ << "] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}

} // namespace gc
