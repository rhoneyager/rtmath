#include "Stdafx-ddscat.h"
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <tuple>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/weak_ptr.hpp>

#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/macros.h>
#include <Ryan_Debug/hash.h>
#include <Ryan_Debug/splitSet.h>
#include <Ryan_Debug/registry.h>
#include <Ryan_Debug/Serialization.h>
#include <Ryan_Debug/error.h>
#include "../rtmath/error/debug.h"
#include "../rtmath/Voronoi/Voronoi.h"
#include "../rtmath/ddscat/shapefile.h"
#include "shapestats_private.h"
namespace rtmath {
	    namespace ddscat {
		            namespace shapefile {

				                size_t decimateDielCount(const convolutionCellInfo& info,
								                const boost::shared_ptr<const shapefile>)
							            { return info.numFilled; }

						            size_t decimateThreshold(const convolutionCellInfo& info,
									                    const boost::shared_ptr<const shapefile>,
											                    size_t threshold)
								                {
											                if (info.numFilled >= threshold) return info.initDiel;
													                return 0;
															            }
							            }
			        }
}

