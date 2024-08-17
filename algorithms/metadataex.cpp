#include "log.h"
#include "metadataex.h"
#include <iostream>
#include <fstream>
#include <cstring>
#include <libexif/exif-data.h>
#include <png.h>

namespace gc
{

MetadataEx::MetadataEx()
{
}

// Function to extract EXIF description from a JPEG file
GC_STATUS MetadataEx::ExtractExifDescription( const std::string filepath, std::string &desc )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        ExifData *exif_data = exif_data_new_from_file( filepath.c_str() );
        if ( nullptr == exif_data )
        {
            FILE_LOG( logERROR ) << "[MetadataEx::ExtractExifDescription] Could not load EXIF data from image file";
            retVal = GC_ERR;
        }
        else
        {
            ExifEntry *entry = exif_data_get_entry(exif_data, EXIF_TAG_IMAGE_DESCRIPTION);
            desc.clear();
            if ( nullptr == entry )
            {
                FILE_LOG( logERROR ) << "[MetadataEx::ExtractExifDescription] Could not extract description from EXIF data";
                retVal = GC_ERR;
            }
            else
            {
                desc = std::string( reinterpret_cast< char * >( entry->data ), entry->size );
                exif_data_unref( exif_data );
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

// Function to write metadata to a PNG file
GC_STATUS MetadataEx::WritePngWithDescription( const std::string inputFilepath, const std::string outputFilepath, const std::string &description )
{
    GC_STATUS retVal = GC_OK;
    try
    {
        FILE *fp = fopen( inputFilepath.c_str(), "rb" );
        if ( nullptr == fp )
        {
            FILE_LOG( logERROR ) << "[MetadataEx::ExtractExifDescription] Could not open PNG file " << inputFilepath;
            retVal = GC_ERR;
        }
        else
        {
            png_structp png = png_create_read_struct( PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr );
            if ( nullptr == png )
            {
                fclose( fp );
                FILE_LOG( logERROR ) << "[MetadataEx::ExtractExifDescription] Could not create PNG read structure";
                retVal = GC_ERR;
            }
            else
            {
                png_infop info = png_create_info_struct( png );
                if ( nullptr == info )
                {
                    png_destroy_read_struct( &png, nullptr, nullptr );
                    fclose( fp );
                    FILE_LOG( logERROR ) << "[MetadataEx::ExtractExifDescription] Could not create PNG read info structure";
                    retVal = GC_ERR;
                }
                else
                {
                    int ret = setjmp( png_jmpbuf( png ) );
                    if ( 0 != ret )
                    {
                        png_destroy_read_struct( &png, &info, nullptr );
                        fclose( fp );
                        FILE_LOG( logERROR ) << "[MetadataEx::ExtractExifDescription] Could jump to the PNG buffer";
                        retVal = GC_ERR;
                    }
                    else
                    {
                        png_init_io( png, fp );
                        png_read_png( png, info, PNG_TRANSFORM_IDENTITY, nullptr );

                        // Create text chunk for the description
                        png_text text_chunk;
                        text_chunk.key = ( char * )"Description";
                        text_chunk.text = ( char * )description.c_str();
                        text_chunk.compression = PNG_TEXT_COMPRESSION_NONE;
                        text_chunk.lang = nullptr;
                        text_chunk.lang_key = nullptr;

                        png_set_text( png, info, &text_chunk, 1 );

                        // Open the output PNG file
                        FILE* out_fp = fopen( outputFilepath.c_str(), "wb" );
                        if ( nullptr == out_fp )
                        {
                            png_destroy_read_struct( &png, &info, nullptr );
                            fclose(fp);
                            FILE_LOG( logERROR ) << "[MetadataEx::ExtractExifDescription] Could not open PNG file " << outputFilepath;
                            retVal = GC_ERR;
                        }
                        else
                        {
                            png_structp png_write = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
                            if ( nullptr == png_write )
                            {
                                fclose(out_fp);
                                png_destroy_read_struct( &png, &info, nullptr );
                                fclose(fp);
                                FILE_LOG( logERROR ) << "[MetadataEx::ExtractExifDescription] Could not create PNG write structure";
                                retVal = GC_ERR;
                            }
                            else
                            {
                                png_infop info_write = png_create_info_struct(png_write);
                                if ( nullptr != info_write )
                                {
                                    png_destroy_write_struct( &png_write, nullptr );
                                    fclose(out_fp);
                                    png_destroy_read_struct( &png, &info, nullptr );
                                    fclose(fp);
                                    FILE_LOG( logERROR ) << "[MetadataEx::ExtractExifDescription] Could not creating PNG write info structure";
                                    retVal = GC_ERR;
                                }
                                else
                                {
                                    ret = setjmp( png_jmpbuf( png_write ) );
                                    if ( 0 != ret )
                                    {
                                        png_destroy_write_struct( &png_write, &info_write );
                                        fclose(out_fp);
                                        png_destroy_read_struct( &png, &info, nullptr );
                                        fclose(fp);
                                        std::cerr << "Error during PNG writing.\n";
                                        FILE_LOG( logERROR ) << "[MetadataEx::ExtractExifDescription] Could not write PNG file";
                                        retVal = GC_ERR;
                                    }
                                    else
                                    {
                                        png_init_io( png_write, out_fp );
                                        png_write_png( png_write, info_write, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT );

                                        fclose( out_fp );
                                        png_destroy_write_struct( &png_write, &info_write );
                                        png_destroy_read_struct( &png, &info, nullptr );
                                        fclose( fp );
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    catch( std::exception &e )
    {
        FILE_LOG( logERROR ) << "[MetadataEx::WritePngWithDescription] " << e.what();
        retVal = GC_EXCEPT;
    }

    return retVal;
}

// int main() {
//     const char* jpeg_filename = "input.jpg";
//     const char* png_filename = "output.png";
//     const char* output_png = "output_with_description.png";

//     // Extract EXIF description from JPEG
//     std::string description = extract_exif_description(jpeg_filename);

//     // Write the description to the PNG file
//     write_png_with_description(png_filename, output_png, description);

//     return 0;
// }

} // namespace gc
