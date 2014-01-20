#include "WritePoints.h"

#define DB_USE_MODERN_DTPTR
#include <silo.h>

namespace rtmath {
	namespace plugins {
		namespace silo {

			void WritePoints(DBfile *db, const char* name,
				const std::array<std::string, 3> &axislabels,
				const std::array<std::string, 3> &axisunits, 
				const Eigen::Matrix<float, Eigen::Dynamic, 3> &pts,
				std::vector<std::tuple<std::string, std::string, 
				const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > > &vals)
			{
				// Write the point mesh
				DBoptlist *optlist = DBMakeOptlist(6);
				DBAddOption(optlist, DBOPT_XLABEL, (void*)axislabels[0].c_str());
				DBAddOption(optlist, DBOPT_YLABEL, (void*)axislabels[1].c_str());
				DBAddOption(optlist, DBOPT_ZLABEL, (void*)axislabels[2].c_str());
				DBAddOption(optlist, DBOPT_XUNITS, (void *)axisunits[0].c_str());
				DBAddOption(optlist, DBOPT_YUNITS, (void *)axisunits[0].c_str());
				DBAddOption(optlist, DBOPT_ZUNITS, (void *)axisunits[0].c_str());
				const int numPoints = (int)pts.rows();

				// Convert the points, by coordinate, into arrays
				const float *pcoords[3] = { pts.col(0).data(), 
					pts.col(1).data(), pts.col(2).data() };
				DBPutPointmesh(db, name, 3, pcoords, numPoints, DB_FLOAT, optlist);
				DBFreeOptlist(optlist);
				
				for (const auto &a : vals)
				{
					// Write the point var
					optlist = DBMakeOptlist(1);
					DBAddOption(optlist, DBOPT_UNITS, (void *)std::get<1>(a).c_str());
					
					const float **vals = new const float*[std::get<2>(a).cols()];
					for (size_t i = 0; i < (size_t) std::get<2>(a).cols(); ++i)
						vals[i] = std::get<2>(a).col(i).data();
					//float *vals[1] = { a.second.col(0).data() };
					DBPutPointvar(db, std::get<0>(a).c_str(), name, 
						(int) std::get<2>(a).cols(), 
						vals, 
						numPoints, 
						DB_FLOAT, 
						optlist);
					DBFreeOptlist(optlist);
					delete[] vals;
				}
			}

		}
	}
}

