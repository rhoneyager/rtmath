#pragma once

#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <complex>

namespace rtmath
{
	namespace ddscat
	{
		class shapeFileStats;
		class ddOutputSingle;

		/** \brief Class that matches shapefiles to run results.
		*
		* \todo Retool this to match shape files, shape stats and ddscat results
		*
		* \note holly shape files are like 1mm14shape.txt or 1mm1_..._shape.txt
		* liu shape files are like avg_5000.shp
		* stat results end in .xml (+.bz2,+...)
		* holly avg files follow 1mm14_t_f.avg
		* liu avg files follow avg_3000[.*]
		* 
		* \note avg file provides freq, dielectric info, qsca, qbk, qext, aeff, d
		**/
		class dataset
		{
		public:
			dataset();
			dataset(const std::string &prefix);
			/// For holly stuff, the shape id is before the first underscore.
			/// Other stuff may be appended at the end (lev2), but she nicely splits the folders!
			std::string id;

			/// all avg results (.avg)
			std::vector<boost::filesystem::path> ddres;
			/// all loaded ddscat results
			std::vector<boost::shared_ptr<rtmath::ddscat::ddOutputSingle> > ddloaded;

			/// shape stats (.xml)
			boost::filesystem::path shapestatsfile;
			/// the shape file (.shp, _shape.txt)
			boost::filesystem::path shapefile;
			/// The actual shape stats.
			boost::shared_ptr<rtmath::ddscat::shapeFileStats> stats;

			/** \brief Load shape statistics
			*
			* If a shape stats file is found, open it directly.
			* If no shape stats file, then calculate the stats from a 
			* provided shape file. Stats will then be written if requested, to 
			* the desired directory.
			* If no shape file, throws an error.
			**/
			void prepStats(bool writeStats = false, const std::string& statsdir = "");


			static std::string getPrefix(const boost::filesystem::path &filename);
			static bool isValid(const boost::filesystem::path &filename);
			static bool isAvg(const boost::filesystem::path &filename);
			static bool isShape(const boost::filesystem::path &filename);
			static bool isShapeStats(const boost::filesystem::path &filename);
		};

	}
}
