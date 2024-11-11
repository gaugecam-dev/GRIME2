#ifndef METADATAEX_H
#define METADATAEX_H

#include "gc_types.h"

namespace gc
{

class MetadataEx
{
public:
    MetadataEx();

    GC_STATUS ReadExifDescription( const std::string filepath, std::string &desc );
    GC_STATUS ReadPngDescription( const std::string filepath, std::string &desc );
    GC_STATUS ReadJpgDescription( const std::string filepath, std::string &desc );

    GC_STATUS WriteExifDescription( const std::string filepath, const std::string desc );
    GC_STATUS WritePngDescription( const std::string filepath, const std::string desc );
    GC_STATUS WriteJpgDescription( const std::string filepath, const std::string desc );
};

} // namespace gc

#endif // METADATAEX_H
