#ifndef METADATAEX_H
#define METADATAEX_H

#include "gc_types.h"

namespace gc
{

class MetadataEx
{
public:
    MetadataEx();

    GC_STATUS ExtractExifDescription( const std::string filepath, std::string &desc );
    GC_STATUS WritePngWithDescription( const std::string inputFilepath, const std::string outputFilepath, const std::string &description );
};

} // namespace gc

#endif // METADATAEX_H
