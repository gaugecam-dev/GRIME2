#ifndef RANSACSTREAMFLOW_H
#define RANSACSTREAMFLOW_H

#include "gc_types.h"

namespace gc
{

class RansacStreamflow
{
public:
    RansacStreamflow();

    GC_STATUS CreateRandomStreamflowModel( const std::string filepathCSV, const std::string filepathResult,
                                           const std::string timestampFormat, const int timestampCol,
                                           const int valueCol /*, const int chances */ );
};

} // namespace gc

#endif // RANSACSTREAMFLOW_H
