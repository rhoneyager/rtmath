#include "../rtmath/Stdafx.h"
#include <Eigen/Core>
//#include "../rtmath/matrixop.h"
#include "../rtmath/ddscat/shapefile.h"

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/string.hpp>

#include "../rtmath/Serialization/serialization_macros.h"
#include "../rtmath/Serialization/eigen_serialization.h"


namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void serialize(Archive &ar, rtmath::ddscat::shapefile &g, const unsigned int version)
		{
				ar & make_nvp("Filename", g.filename);
				ar & make_nvp("Description", g.desc);
				ar & make_nvp("N", g.numPoints);
				ar & make_nvp("Dielectrics", g.Dielectrics);
				ar & make_nvp("a1", g.a1);
				ar & make_nvp("a2", g.a2);
				ar & make_nvp("a3", g.a3);
				ar & make_nvp("d", g.d);
				ar & make_nvp("x0", g.x0);
				ar & make_nvp("xd", g.xd);
				if (version)
				{
					// Write out the points. Eigen's serialization routines should work well here.
					ar & make_nvp("latticePts", g.latticePts);
					ar & make_nvp("latticePtsStd", g.latticePtsStd);
					ar & make_nvp("latticePtsNorm", g.latticePtsNorm);
					ar & make_nvp("latticePtsRi", g.latticePtsRi);
					
					ar & make_nvp("mins", g.mins);
					ar & make_nvp("maxs", g.maxs);
					ar & make_nvp("means", g.means);
				}
		}

		EXPORT(serialize,rtmath::ddscat::shapefile);
	}
}

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::shapefile);
BOOST_CLASS_VERSION(rtmath::ddscat::shapefile, 1);
