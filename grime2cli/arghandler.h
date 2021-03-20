#ifndef ARGHANDLER_H
#define ARGHANDLER_H
#include "../algorithms/log.h"
#include <boost/filesystem.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <string>
#include <iostream>

using namespace std;
using namespace boost;
namespace fs = boost::filesystem;
static FILE *g_logFile = nullptr;

bool IsExistingImagePath( const string imgPath );

typedef enum GRIME2_CLI_OPERATIONS
{
    CALIBRATE,
    FIND_LINE,
    RUN_FOLDER,
    SHOW_METADATA,
    SHOW_VERSION,
    SHOW_HELP
} GRIME2_CLI_OP;

class Grime2CLIParams
{
public:
    Grime2CLIParams() :
        verbose( false ),
        opToPerform( SHOW_HELP ),
        timestamp_startPos( -1 ),
        timeStamp_length( -1 )
    {}
    void clear()
    {
        verbose = false;
        opToPerform = SHOW_HELP;
        src_imagePath.clear();
        csvPath.clear();
        calib_jsonPath.clear();
        result_imagePath.clear();
        timestamp_format.clear();
        timestamp_type.clear();
        timestamp_startPos = -1;
        timeStamp_length = -1;
    }
    bool verbose;
    GRIME2_CLI_OP opToPerform;
    string src_imagePath;
    string csvPath;
    string calib_jsonPath;
    string result_imagePath;
    string timestamp_format;
    string timestamp_type;
    int timestamp_startPos;
    int timeStamp_length;
};
int GetArgs( int argc, char *argv[], Grime2CLIParams &params )
{
    int retVal = 0;
    try
    {
        params.clear();
        for ( int i = 1; i < argc; ++i )
        {
            // cout << string( argv[ i ] ) << endl;
            if ( 0 != retVal )
                break;
            if ( 0 == string( argv[ i ] ).find( "--" ) )
            {
                if ( "help" == string( argv[ i ] ).substr( 2 ) )
                {
                    params.opToPerform = SHOW_HELP;
                    return 0;
                }
                else if ( "version" == string( argv[ i ] ).substr( 2 ) )
                {
                    params.opToPerform = SHOW_VERSION;
                    return 0;
                }
                else if ( "verbose" == string( argv[ i ] ).substr( 2 ) )
                {
                    params.verbose = true;
                }
                else if ( "logFile" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 1 >= argc )
                    {
                        FILE_LOG( logERROR ) << "[" << __func__ << "][GetArgs()] no log filename specified on --logFile request";
                        retVal = -1;
                    }
                    else
                    {
                        g_logFile = fopen( argv[ ++i ], "w" );
                        if ( nullptr == g_logFile )
                        {
                            FILE_LOG( logERROR ) << "[" << __func__ << "][GetArgs()] could not open requested log file: " << argv[ i ];
                            retVal = -1;
                            break;
                        }
                        else
                        {
                            Output2FILE::Stream() = g_logFile;
                        }
                    }
                }
                else if ( "calibrate" == string( argv[ i ] ).substr( 2 ) )
                {
                    params.opToPerform = CALIBRATE;
                }
                else if ( "find_line" == string( argv[ i ] ).substr( 2 ) )
                {
                    params.opToPerform = FIND_LINE;
                }
                else if ( "run_folder" == string( argv[ i ] ).substr( 2 ) )
                {
                    params.opToPerform = RUN_FOLDER;
                }
                else if ( "show_metadata" == string( argv[ i ] ).substr( 2 ) )
                {
                    params.opToPerform = SHOW_METADATA;
                }
                else if ( "version" == string( argv[ i ] ).substr( 2 ) )
                {
                    params.opToPerform = SHOW_VERSION;
                }
                else if ( "help" == string( argv[ i ] ).substr( 2 ) )
                {
                    params.opToPerform = SHOW_HELP;
                }
                else if ( "timestamp_from_exif" == string( argv[ i ] ).substr( 2 ) )
                {
                    params.timestamp_type = "from_exif";
                }
                else if ( "timestamp_from_filename" == string( argv[ i ] ).substr( 2 ) )
                {
                    params.timestamp_type = "from_filename";
                }
                else if ( "timestamp_start_pos" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 1 < argc )
                    {
                        params.timestamp_startPos = stoi( argv[ ++i ] );
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "No value supplied on --timestamp_start_pos request";
                        retVal = -1;
                        break;
                    }
                }
                else if ( "timestamp_length" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 1 < argc )
                    {
                        params.timeStamp_length = stoi( argv[ ++i ] );
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "No value supplied on --timestamp_start_pos request";
                        retVal = -1;
                        break;
                    }
                }
                else if ( "timestamp_format" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 1 < argc )
                    {
                        params.timestamp_format = string( argv[ ++i ] );
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "No value supplied on --timestamp_start_pos request";
                        retVal = -1;
                        break;
                    }
                }
                else if ( "csv_file" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 1 < argc )
                    {
                        params.csvPath = string( argv[ ++i ] );
                        if ( string::npos == params.csvPath.find( ".csv" ) )
                        {
                            FILE_LOG( logERROR ) << "CSV file " << params.csvPath << " extension not recognized";
                            retVal = -1;
                        }
                        else if ( !fs::exists( params.csvPath ) )
                        {
                            FILE_LOG( logERROR ) << "CSV file does not exist: " << params.csvPath;
                            retVal = -1;
                        }
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "No value supplied on --csv_file request";
                        retVal = -1;
                        break;
                    }
                }
                else if ( "calib_json" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 1 < argc )
                    {
                        params.calib_jsonPath = string( argv[ ++i ] );
                        if ( string::npos == params.calib_jsonPath.find( ".json" ) )
                        {
                            FILE_LOG( logERROR ) << "JSON file " << params.calib_jsonPath << " extension not recognized";
                            retVal = -1;
                            break;
                        }
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "No value supplied on --result_image request";
                        retVal = -1;
                        break;
                    }
                }
                else if ( "result_image" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 1 < argc )
                    {
                        params.result_imagePath = string( argv[ ++i ] );
                        if ( string::npos == params.result_imagePath.find( ".png" ) &&
                             string::npos == params.result_imagePath.find( ".jpg" ) )
                        {
                            FILE_LOG( logERROR ) << "Image file " << params.result_imagePath << " extension not recognized";
                            retVal = -1;
                        }
                        else if ( !fs::exists( params.result_imagePath ) )
                        {
                            bool isOk = fs::create_directories( fs::path( params.result_imagePath ).parent_path() );
                            if ( !isOk )
                            {
                                FILE_LOG( logERROR ) << "Could not create result image folder: " << params.result_imagePath;
                                retVal = -1;
                            }
                        }
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "No value supplied on --result_image request";
                        retVal = -1;
                        break;
                    }
                }
                else if ( "result_folder" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 1 < argc )
                    {
                        params.result_imagePath = string( argv[ ++i ] );
                        if ( !fs::is_directory( params.result_imagePath ) )
                        {
                            FILE_LOG( logERROR ) << "Result path " << params.result_imagePath << " is not a folder";
                            retVal = -1;
                        }
                        else if ( !fs::exists( params.result_imagePath ) )
                        {
                            bool isOk = fs::create_directories( params.result_imagePath );
                            if ( !isOk )
                            {
                                FILE_LOG( logERROR ) << "Could not create result folder: " << params.result_imagePath;
                                retVal = -1;
                            }
                        }
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "No value supplied on --result_folder request";
                        retVal = -1;
                        break;
                    }
                }
                else
                {
                    FILE_LOG( logERROR ) << "[arghandler] Invalid command line item " << argv[ i ];
                    retVal = -1;
                }
            }
            else if ( params.src_imagePath.empty() )
            {
                params.src_imagePath = argv[ i ];
                if ( CALIBRATE == params.opToPerform ||
                     FIND_LINE == params.opToPerform ||
                     SHOW_METADATA == params.opToPerform )
                {
                    bool isOk = IsExistingImagePath( params.src_imagePath );
                    if ( !isOk )
                    {
                        retVal = -1;
                    }
                }
                else if ( RUN_FOLDER == params.opToPerform )
                {
                    if ( !fs::is_directory( params.src_imagePath ) )
                    {
                        FILE_LOG( logERROR ) << "Source path is not a folder: " << params.src_imagePath;
                        retVal = -1;
                    }
                }
                else
                {
                    FILE_LOG( logERROR ) << "There is no associated operation for the first path";
                    retVal = -1;
                }
            }
            else
            {
                FILE_LOG( logWARNING ) << "Extraneous command line item " << argv[ i ];
            }
            if ( 0 != retVal )
                break;
        }
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[" << __func__ << "][GetArgs()] " << diagnostic_information( e );
        retVal = -1;
    }
    return retVal;
}
bool IsExistingImagePath( const string imgPath )
{
    bool isGood = true;
    if ( string::npos == imgPath.find( ".png" ) &&
         string::npos == imgPath.find( ".jpg" ) )
    {
        FILE_LOG( logERROR ) << "Image file " << imgPath << " extension not recognized";
        isGood = false;
    }
    else if ( !fs::exists( imgPath ) )
    {
        FILE_LOG( logERROR ) << "Image file " << imgPath << " does not exist";
        isGood = false;
    }
    else if ( !fs::is_regular_file( imgPath ) )
    {
        FILE_LOG( logERROR ) << "Image file " << imgPath << " is not a regular file";
        isGood = false;
    }
    return isGood;
}
void PrintHelp()
{
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // --well_series_to_folder [top level folder path] [destination folder path] --stack_image_index [index OPTIONAL default=2] --well_id [well id]
    // Creates folders for each tile position for the specified well and places
    // the images for each scan time and tile positions from the specified stack
    // index into the created folder to which it pertains
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    cout << "FORMAT: StackCLI --align_image_series [Folder with images] " << endl <<
            "                 [--start_index [start index] OPTIONAL default=0]" << endl <<
            "        Loads all the image in the specified folder, sorts them by filename," << endl <<
            "        aligns them to each other, creates a folder named \"aligned\" in the" << endl <<
            "        specified folder, and stores the aligned images there with the filename" << endl <<
            "        [previous filename]_to_[next filename].png" << endl;
    cout << "FORMAT: StackCLI --align_images [Image path one] [Image path two] --result_image [result image path]" << endl <<
            "        Aligns the second image to the first image and saves the image to the" << endl <<
            "        specified result image path" << endl;
    cout << "--verbose" << endl;
    cout << "--logFile [filepath]" << endl;
    cout << "     Logs message to specified file rather than stderr" << endl;
    cout << "--help" << endl;
    cout << "     Shows this help message" << endl;
    cout << "--version" << endl;
    cout << "     Shows the ScanAlignCLI version" << endl;
}
#endif // ARGHANDLER_H
