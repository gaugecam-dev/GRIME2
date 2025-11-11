#ifndef ARGHANDLER_H
#define ARGHANDLER_H
#include "../algorithms/log.h"
#include "../algorithms/calibexecutive.h"
#include <opencv2/core.hpp>
#include <filesystem>
#include <boost/exception/diagnostic_information.hpp>
#include <string>
#include <iostream>

using namespace std;
using namespace boost;
namespace fs = std::filesystem;
static FILE *g_logFile = nullptr;

bool IsExistingImagePath( const string imgPath );

typedef enum GRIME2_CLI_OPERATIONS
{
    CALIBRATE,
    CREATE_CALIB,
    FIND_LINE,
    RUN_FOLDER,
    MAKE_GIF,
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
        timeStamp_length( -1 ),
        delay_ms( 250 ),
        scale( 0.2 ),
        calib_roi(cv::Rect(-1,-1,-1,-1)),
        waterline_region(gc::LineSearchRoi(cv::Point(-1,-1),
                                           cv::Point(-1,-1),
                                           cv::Point(-1,-1),
                                           cv::Point(-1,-1))),
        facet_length(-1.0),
        zero_offset(-1.0),
        noCalibSave(false),
        cache_result(false)
    {}
    void clear()
    {
        verbose = false;
        opToPerform = SHOW_HELP;
        src_imagePath.clear();
        csvPath.clear();
        line_roi_folder.clear();
        calib_jsonPath.clear();
        calib_type.clear();
        result_imagePath.clear();
        timestamp_format.clear();
        timestamp_type.clear();
        timestamp_startPos = -1;
        timeStamp_length = -1;
        delay_ms = 250;
        scale = 0.2;
        calib_roi = cv::Rect(-1, -1, -1, -1);
        waterline_region.clear();
        facet_length = -1.0;
        zero_offset = -1.0;
        noCalibSave = false;
        cache_result = false;
    }
    bool verbose;
    GRIME2_CLI_OP opToPerform;
    string src_imagePath;
    string csvPath;
    string line_roi_folder;
    string calib_jsonPath;
    string calib_type;
    string result_imagePath;
    string timestamp_format;
    string timestamp_type;
    int timestamp_startPos;
    int timeStamp_length;
    int delay_ms;
    double scale;
    cv::Rect calib_roi;
    gc::LineSearchRoi waterline_region;
    double facet_length;
    double zero_offset;
    bool noCalibSave;
    bool cache_result;

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
                }
                else if ( "version" == string( argv[ i ] ).substr( 2 ) )
                {
                    params.opToPerform = SHOW_VERSION;
                }
                else if ( "verbose" == string( argv[ i ] ).substr( 2 ) )
                {
                    params.verbose = true;
                }
                else if ( "logFile" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 1 >= argc )
                    {
                        FILE_LOG( logERROR ) << "[ArgHandler] no log filename specified on --logFile request";
                        retVal = -1;
                        break;
                    }
                    else
                    {
#if WIN32
                        fopen_s( &g_logFile, argv[ ++i ], "w" );
#else
                        g_logFile = fopen( argv[ ++i ], "w" );
#endif
                        if ( nullptr == g_logFile )
                        {
                            FILE_LOG( logERROR ) << "[ArgHandler]  could not open requested log file: " << argv[ i ];
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
                else if ( "no_calib_save" == string( argv[ i ] ).substr( 2 ) )
                {
                    params.noCalibSave = true;
                }
                else if ( "cache_result" == string( argv[ i ] ).substr( 2 ) )
                {
                    params.cache_result = true;
                }
                else if ( "create_calib" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 1 < argc )
                    {
                        params.opToPerform = CREATE_CALIB;
                        string calType = string( argv[ ++i ] );
                        params.calib_type = "Octagon";
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "[ArgHandler] No calibration type supplied on --fps request";
                        retVal = -1;
                        break;
                    }
                }
                else if ( "find_line" == string( argv[ i ] ).substr( 2 ) )
                {
                    params.opToPerform = FIND_LINE;
                }
                else if ( "run_folder" == string( argv[ i ] ).substr( 2 ) )
                {
                    params.opToPerform = RUN_FOLDER;
                }
                else if ( "make_gif" == string( argv[ i ] ).substr( 2 ) )
                {
                    params.opToPerform = MAKE_GIF;
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
                else if ( "delay_ms" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 1 < argc )
                    {
                        params.delay_ms = stoi( argv[ ++i ] );
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "[ArgHandler] No value supplied on --fps request";
                        retVal = -1;
                        break;
                    }
                }
                else if ( "scale" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 1 < argc )
                    {
                        params.scale = stod( argv[ ++i ] );
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "[ArgHandler] No value supplied on --scale request";
                        retVal = -1;
                        break;
                    }
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
                        FILE_LOG( logERROR ) << "[ArgHandler] No value supplied on --timestamp_start_pos request";
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
                        FILE_LOG( logERROR ) << "[ArgHandler] No value supplied on --timestamp_start_pos request";
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
                            FILE_LOG( logERROR ) << "[ArgHandler] CSV file " << params.csvPath << " extension not recognized";
                            retVal = -1;
                            break;
                        }
                        else if ( !fs::exists( fs::path( params.csvPath ).parent_path() ) )
                        {
                            bool isOk = fs::create_directories( fs::path( params.result_imagePath ).parent_path() );
                            if ( !isOk )
                            {
                                FILE_LOG( logERROR ) << "[ArgHandler] Could not create result image folder: " << fs::path( params.result_imagePath ).parent_path().string();
                                retVal = -1;
                                break;
                            }
                        }
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "[ArgHandler] No value supplied on --csv_file request";
                        retVal = -1;
                        break;
                    }
                }
                else if ( "line_roi_folder" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 1 < argc )
                    {
                        params.line_roi_folder = string( argv[ ++i ] );
                        if ( !fs::exists( fs::path( params.line_roi_folder ).parent_path() ) )
                        {
                            bool isOk = fs::create_directories( params.line_roi_folder );
                            if ( !isOk )
                            {
                                FILE_LOG( logERROR ) << "[ArgHandler] Could not create line roi folder: " << params.line_roi_folder;
                                retVal = -1;
                                break;
                            }
                        }
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "[ArgHandler] No value supplied on --line_roi_folder request";
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
                            FILE_LOG( logERROR ) << "[ArgHandler] JSON file " << params.calib_jsonPath << " extension not recognized";
                            retVal = -1;
                            break;
                        }
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "[ArgHandler] No value supplied on --calib_json request";
                        retVal = -1;
                        break;
                    }
                }
                else if ( "result_image" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 1 < argc )
                    {
                        params.result_imagePath = string( argv[ ++i ] );
                        if ( !params.result_imagePath.empty() )
                        {
                            if ( ( MAKE_GIF != params.opToPerform &&
                                 string::npos == params.result_imagePath.find( ".png" ) &&
                                 string::npos == params.result_imagePath.find( ".jpg" ) ) ||
                                ( MAKE_GIF == params.opToPerform &&
                                 string::npos == params.result_imagePath.find( ".gif" ) ) )
                            {
                                FILE_LOG( logERROR ) << "[ArgHandler] Image file " << params.result_imagePath << " extension not recognized";
                                retVal = -1;
                                break;
                            }
                            else
                            {
                                string result_folder = fs::path( params.result_imagePath ).parent_path().string();
                                if ( !fs::exists( result_folder ) )
                                {
                                    bool isOk = fs::create_directories( result_folder );
                                    if ( !isOk )
                                    {
                                        FILE_LOG( logERROR ) << "[ArgHandler] Could not create result image folder: " << result_folder;
                                        retVal = -1;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "[ArgHandler] No value supplied on --result_image request";
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
                            FILE_LOG( logERROR ) << "[ArgHandler] Result path " << params.result_imagePath << " is not a folder";
                            retVal = -1;
                            break;
                        }
                        else if ( !fs::exists( params.result_imagePath ) )
                        {
                            bool isOk = fs::create_directories( params.result_imagePath );
                            if ( !isOk )
                            {
                                FILE_LOG( logERROR ) << "[ArgHandler] Could not create result folder: " << params.result_imagePath;
                                retVal = -1;
                                break;
                            }
                        }
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "[ArgHandler] No value supplied on --result_folder request";
                        retVal = -1;
                        break;
                    }
                }
                else if ( "source" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 1 < argc )
                    {
                        params.src_imagePath = argv[ ++i ];
                        if ( CALIBRATE == params.opToPerform ||
                            CREATE_CALIB == params.opToPerform ||
                            FIND_LINE == params.opToPerform ||
                            SHOW_METADATA == params.opToPerform )
                        {
                            bool isOk = IsExistingImagePath( params.src_imagePath );
                            if ( !isOk )
                            {
                                FILE_LOG( logERROR ) << "Source image does not exist: " << params.src_imagePath;
                                retVal = -1;
                                break;
                            }
                        }
                        else if ( MAKE_GIF == params.opToPerform ||
                                 RUN_FOLDER == params.opToPerform )
                        {
                            if ( !fs::is_directory( params.src_imagePath ) )
                            {
                                FILE_LOG( logERROR ) << "Source path is not a folder: " << params.src_imagePath;
                                retVal = -1;
                                break;
                            }
                        }
                        else
                        {
                            FILE_LOG( logERROR ) << "[ArgHandler] There is no associated operation for the first path";
                            retVal = -1;
                            break;
                        }
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "[ArgHandler] No value supplied on --source request";
                        retVal = -1;
                        break;
                    }
                }
                else if ( "facet_length" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 1 < argc )
                    {
                        params.facet_length = stod( string( argv[ ++i ] ) );
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "[ArgHandler] No value supplied on --facet_length request";
                        retVal = -1;
                    }
                }
                else if ( "zero_offset" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 1 < argc )
                    {
                        params.zero_offset = stod( string( argv[ ++i ] ) );
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "[ArgHandler] No value supplied on --zero_offset request";
                        retVal = -1;
                    }
                }
                else if ( "calib_roi" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 4 < argc )
                    {
                        params.calib_roi.x = stoi( string( argv[ ++i ] ) );
                        params.calib_roi.y = stoi( string( argv[ ++i ] ) );
                        params.calib_roi.width = stoi( string( argv[ ++i ] ) );
                        params.calib_roi.height = stoi( string( argv[ ++i ] ) );
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "[GetArgs()] Not enough paramters for calib roi (need 4)";
                        retVal = -1;
                    }
                }
                else if ( "waterline_roi" == string( argv[ i ] ).substr( 2 ) )
                {
                    if ( i + 8 < argc )
                    {
                        int tl_x = stoi( string( argv[ ++i ] ) );
                        int tl_y = stoi( string( argv[ ++i ] ) );
                        int tr_x = stoi( string( argv[ ++i ] ) );
                        int tr_y = stoi( string( argv[ ++i ] ) );
                        int bl_x = stoi( string( argv[ ++i ] ) );
                        int bl_y = stoi( string( argv[ ++i ] ) );
                        int br_x = stoi( string( argv[ ++i ] ) );
                        int br_y = stoi( string( argv[ ++i ] ) );
                        params.waterline_region = gc::LineSearchRoi( cv::Point( tl_x, tl_y ),
                                                                    cv::Point( tr_x, tr_y ),
                                                                    cv::Point( bl_x, bl_y ),
                                                                    cv::Point( br_x, br_y ) );
                    }
                    else
                    {
                        FILE_LOG( logERROR ) << "[GetArgs()] Not enough paramters for waterline region (need 8)";
                        retVal = -1;
                    }
                }
                else
                {
                    FILE_LOG( logERROR ) << "[ArgHandler]  Invalid command line item " << argv[ i ];
                    retVal = -1;
                    break;
                }
            }
            else
            {
                FILE_LOG( logWARNING ) << "[ArgHandler] Extraneous command line item " << argv[ i ];
            }
            if ( 0 != retVal )
                break;
        }
    }
    catch( const boost::exception &e )
    {
        FILE_LOG( logERROR ) << "[ArgHandler] " << diagnostic_information( e );
        retVal = -1;
    }
    return retVal;
}
bool IsExistingImagePath( const string imgPath )
{
    bool isGood = true;
    if ( string::npos == imgPath.find( ".png" ) &&
        string::npos == imgPath.find( ".jpg" ) &&
        string::npos == imgPath.find( ".PNG" ) &&
        string::npos == imgPath.find( ".JPG" ) )
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
    // --well_series_to_folder <top level folder path> <destination folder path> --stack_image_index [index OPTIONAL default=2] --well_id <well id>
    // Creates folders for each tile position for the specified well and places
    // the images for each scan time and tile positions from the specified stack
    // index into the created folder to which it pertains
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    cout << "FORMAT: grime2cli --calibrate <Target image> " << endl <<
        "                  --csv_file <CSV file with bow tie target xy positions (if needed)>" << endl <<
        "                  --calib_json <json filepath for created json file>" << endl <<
        "                 [--result_image <Result overlay image> OPTIONAL]" << endl <<
        "        Loads image with calibration target. Loads an existing calibration," << endl <<
        "        performs a new calibration if a source image is supplied," << endl <<
        "        then stores the calibration to the specified json file. An optional" << endl <<
        "        result image with the calibration result can be created." << endl;
    cout << "FORMAT: grime2cli --create_calibrate <type (must be bowtie or octagon> " << endl <<
        "                  --source <Target image>" << endl <<
        "                  --csv_file <CSV file with bow tie target xy positions>" << endl <<
        "                  --calib_json <json filepath for file to be created>" << endl <<
        "                  --waterline_roi <tl_x> <tl_y> <tr_x> <tr_y> <bl_x> <bl_y> <br_x> btr_y>" << endl <<
        "                          top-left, top-right, bottom-left, bottom-right points of waterline search region"
        "                 [--calib_roi <left> <top> <width> <height> OPTIONAL if not used, whole image is searched]" << endl <<
        "                 [--facet_length <length of facet in world units>]" << endl <<
        "                 [--zero_offset <distance from octo to zero water level in world units>]" << endl <<
        "                 [--result_image <Result overlay image> OPTIONAL]" << endl <<
        "        For octagon calibration json file creation only." << endl <<
        "        Performs a calibration if a source image is supplied," << endl <<
        "        then stores the calibration to the specified json file. An optional" << endl <<
        "        result image with the calibration result can be created." << endl;
    cout << "FORMAT: grime2cli --find_line --timestamp_from_filename or --timestamp_from_exif " << endl <<
        "                  --timestamp_start_pos <position of the first timestamp char of source string>" << endl <<
        "                  --timestamp_format <y-m-d H:M format string for timestamp, e.g., yyyy-mm-ddTMM:HH>" << endl <<
        "                  <Image path to be analyzed> --calib_json <Calibration json file path>" << endl <<
        "                  [--csv_file <Path of csv file to create or append with find line result> OPTIONAL]" << endl <<
        "                  [--result_image <Path of result overlay image> OPTIONAL]" << endl <<
        "                  [--line_roi_folder <Path of line roi image folder> OPTIONAL]" << endl <<
        "        Loads the specified image and calibration file, extracts the image using the specified" << endl <<
        "        timestamp parameters, calculates the line position, returns a json string with the find line" << endl <<
        "        results to stdout, and creates the optional overlay result image if specified" << endl;
    cout << "FORMAT: grime2cli --run_folder --timestamp_from_filename or --timestamp_from_exif " << endl <<
        "                   --timestamp_start_pos <position of the first timestamp char of source string>" << endl <<
        "                   --timestamp_format <y-m-d H:M format string for timestamp, e.g., yyyy-mm-ddTMM:HH>" << endl <<
        "                   <Folder path of images to be analyzed> --calib_json <Calibration json file path>" << endl <<
        "                   [--csv_file <Path of csv file to create or append with find line results> OPTIONAL]" << endl <<
        "                   [--result_folder <Path of folder to hold result overlay images> OPTIONAL]" << endl <<
        "                   [--line_roi_folder <Path of line roi image folder> OPTIONAL]" << endl <<
        "        Loads the specified images and calibration file, extracts the timestamps using the specified" << endl <<
        "        timestamp parameters, calculates the line positions,  and creates the optional overlay result" << endl <<
        "        image if specified" << endl;
    cout << "FORMAT: grime2cli --make_gif <Folder path of images> --result_image <File path of GIF to create>" << endl <<
        "                   [--delay_ms <Animation frames per second> OPTIONAL default=250]" << endl <<
        "                   [--scale <Animation image scale from original> OPTIONAL default=0.2]" << endl <<
        "        Creates a gif animation with the images in the specifed folder at the specified scale and" << endl <<
        "        frame rate" << endl;
    cout << "FORMAT: grime2cli --show_metadata <Image filepath>" << endl;
    cout << "     Returns metadata extracted from the image to stdout" << endl;
    cout << "--verbose" << endl;
    cout << "     Currently has no effect" << endl;
    cout << "--logFile <filepath>" << endl;
    cout << "     Logs message to specified file rather than stderr" << endl;
    cout << "--help" << endl;
    cout << "     Shows this help message" << endl;
    cout << "--version" << endl;
    cout << "     Shows the grime2cli version" << endl;
}
#endif // ARGHANDLER_H
