#include "log.h"
#include "metadataex.h"
#include <iostream>
#include <fstream>
#include <cstring>
#include <png.h>
#include <filesystem>
#include <libexif/exif-data.h>

using namespace std;
namespace fs = std::filesystem;

namespace gc
{

MetadataEx::MetadataEx()
{
}

GC_STATUS MetadataEx::ReadExifDescription( const std::string filepath, std::string &desc )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        std::string ext = fs::path( filepath ).extension();
        std::transform( ext.begin(), ext.end(), ext.begin(), []( unsigned char c ){ return std::tolower( c ); } );
        if ( ".jpg" == ext || ".jpeg" == ext )
        {
            retVal = ReadJpgDescription( filepath, desc );
        }
        else if ( ".png" == ext )
        {
            retVal = ReadPngDescription( filepath, desc );
        }
        else
        {
            FILE_LOG( logERROR ) << "[MetadataEx::ReadExifDescription] Invalid image type. Must be PNG or JPG";
            retVal = GC_ERR;
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[MetadataEx::ReadExifDescription] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS MetadataEx::ReadPngDescription( const std::string filepath, std::string &desc )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        FILE *fp = fopen( filepath.c_str(), "rb" );
        if ( nullptr == fp )
        {
            FILE_LOG( logERROR ) << "[MetadataEx::ReadPngDescription] Could not open file " << filepath;
            retVal = GC_ERR;
        }
        else
        {
            png_structp png = png_create_read_struct( PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr );
            if ( nullptr == png )
            {
                fclose( fp );
                FILE_LOG( logERROR ) << "[MetadataEx::ReadPngDescription] Could not create read structure";
                retVal = GC_ERR;
            }
            else
            {
                png_infop info = png_create_info_struct( png );
                if (nullptr == info )
                {
                    png_destroy_read_struct( &png, nullptr, nullptr );
                    fclose(fp);
                    FILE_LOG( logERROR ) << "[MetadataEx::ReadPngDescription] Could create info structure";
                    retVal = GC_ERR;
                }
                else
                {
                    int ret = setjmp( png_jmpbuf( png ) );
                    if ( 0 != ret )
                    {
                        png_destroy_read_struct( &png, &info, nullptr );
                        fclose(fp);
                        FILE_LOG( logERROR ) << "[MetadataEx::ReadPngDescription] Could not set PNG error handling";
                        retVal = GC_ERR;
                    }
                    else
                    {
                        png_init_io( png, fp );
                        png_read_info( png, info );

                        png_charp description = nullptr;
                        png_textp text_ptr;
                        int num_text;
                        png_get_text( png, info, &text_ptr, &num_text );
                        for ( int i = 0; i < num_text; ++i )
                        {
                            if ( std::string( text_ptr[ i ].key ) == "Description" )
                            {
                                description = text_ptr[ i ].text;
                                break;
                            }
                        }

                        desc = nullptr == description ? "" : std::string( description );
                        png_destroy_read_struct( &png, &info, nullptr );
                        fclose(fp);
                    }
                }
            }
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[MetadataEx::ExtractExifDescription] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS MetadataEx::ReadJpgDescription( const std::string filepath, std::string &desc )
{
    GC_STATUS retVal = GC_OK;
    try
    {
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[MetadataEx::ReadJpgDescription] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS MetadataEx::WriteExifDescription( const std::string filepath, const std::string desc )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        std::string ext = fs::path( filepath ).extension();
        std::transform( ext.begin(), ext.end(), ext.begin(), []( unsigned char c ){ return std::tolower( c ); } );
        if ( ".jpg" == ext || ".jpeg" == ext )
        {
            retVal = WriteJpgDescription( filepath, desc );
        }
        else if ( ".png" == ext )
        {
            retVal = WritePngDescription( filepath, desc );
        }
        else
        {
            FILE_LOG( logERROR ) << "[MetadataEx::WriteExifDescription] Invalid image type. Must be PNG or JPG";
            retVal = GC_ERR;
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[MetadataEx::WriteExifDescription] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS MetadataEx::WritePngDescription( const std::string filepath, const std::string desc )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        ExifData *data = exif_data_new();
        if (!data) {
            std::cerr << "Failed to create ExifData object" << std::endl;
            return;
        }

        // Load existing EXIF data from the file
        std::ifstream file(filename, std::ios::binary);
        if (file) {
            file.seekg(0, std::ios::end);
            std::streamsize size = file.tellg();
            file.seekg(0, std::ios::beg);

            std::vector<char> buffer(size);
            if (file.read(buffer.data(), size)) {
                exif_data_load_data(data, reinterpret_cast<const unsigned char*>(buffer.data()), size);
            }
        } else {
            std::cerr << "Failed to open file: " << filename << std::endl;
            exif_data_unref(data);
            return;
        }

        // Add or update the description tag
        ExifEntry *entry = exif_data_get_entry(data, EXIF_TAG_IMAGE_DESCRIPTION);
        if (!entry) {
            entry = exif_entry_new();
            if (!entry) {
                std::cerr << "Failed to create ExifEntry object" << std::endl;
                exif_data_unref(data);
                return;
            }
            exif_entry_initialize(entry, EXIF_TAG_IMAGE_DESCRIPTION);
            exif_data_set_entry(data, entry);
        }

        exif_entry_set_value(entry, description);

        // Save the updated EXIF data back to the file
        std::ofstream outFile(filename, std::ios::binary | std::ios::app);
        if (!outFile) {
            std::cerr << "Failed to open file for writing: " << filename << std::endl;
            exif_data_unref(data);
            return;
        }

        std::vector<unsigned char> exifDataBuffer;
        unsigned int exifSize = exif_data_save_data(data, &exifDataBuffer);
        outFile.write(reinterpret_cast<const char*>(exifDataBuffer.data()), exifSize);

        outFile.close();
        exif_data_unref(data);

        std::cout << "Description added successfully!" << std::endl;
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[MetadataEx::WritePngDescription] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}
GC_STATUS MetadataEx::WriteJpgDescription( const std::string filepath, const std::string desc )

{
    GC_STATUS retVal = GC_OK;
    try
    {
        ExifData *ed = exif_data_new();
        if (!ed) {
            std::cerr << "Error initializing libexif" << std::endl;
            return 1;
        }

        // Set the image description
        exif_data_set_option(ed, EXIF_TAG_IMAGE_DESCRIPTION, description);

        // Save the modified EXIF data back to the image
        if (!exif_data_save_file(ed, inputFilePath)) {
            std::cerr << "Error saving EXIF data to " << inputFilePath << std::endl;
            exif_data_unref(ed);
            return 1;
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[MetadataEx::WritePngDescription] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}


} // namespace gc
