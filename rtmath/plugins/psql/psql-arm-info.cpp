#include "plugin-psql.h"

/// \brief Provides psql database access
#define _SCL_SECURE_NO_WARNINGS

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <string>
#include <iostream>
#include <sstream>
#include <boost/algorithm/string/trim.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/data/arm_info.h"
#include "../../rtmath/rtmath/data/arm_scanning_radar_sacr.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-psql.h"

namespace rtmath
{
	namespace plugins
	{
		using namespace rtmath::data::arm;
		using std::ostringstream;
		namespace psql
		{
			void search(const arm_info_registry::arm_info_index &index, 
				arm_info_registry::arm_info_index::collection res)
			{
				ostringstream sq;
				sq << "SELECT * FROM ARM_FILES ";
				bool whereflag = false;


				// Select sites, subsites, data_levels

				// Select time ranges and discrete times

			}

			void update(const arm_info_registry::arm_info_index::collection c, 
				arm_info_registry::updateType t)
			{
				// First, look for any matching filenames.
				arm_info_registry::arm_info_index::collection 
					toUpdate = arm_info::makeCollection(),
					toInsert = arm_info::makeCollection();

				std::vector<const char*> filenames;
				filenames.reserve(c->size());
				for (const auto &i : *c)
					filenames.push_back(i->filename.c_str());

				//auto res = 
				
			}
		}
	}
}
