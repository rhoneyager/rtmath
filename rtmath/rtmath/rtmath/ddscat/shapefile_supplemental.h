#pragma once
#include "../defs.h"
#include <array>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <set>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <Ryan_Debug/hash.h>
#include <Ryan_Debug/splitSet.h>
#include <Ryan_Debug/registry.h>
#include <Ryan_Debug/io.h>

namespace rtmath {
	namespace ddscat {
		namespace shapefile {
			struct convolutionCellInfo;

				/// \brief Decimation dielectric function that assigns a dielectric
				/// that corresponds to the number of filled dipoles.
				size_t decimateDielCount(const convolutionCellInfo&,
					const boost::shared_ptr<const shapefile>);

				/// \brief Decimation dielectric function that fills a dielectric 
				/// based on a threshold value (high-pass, inclusive).
				size_t decimateThreshold(const convolutionCellInfo&,
					const boost::shared_ptr<const shapefile>,
					size_t threshold);

				/** \brief Get filled cells within a certain distance
				*
				* \param rsq is the radius squared for the search
				* \param out is the output vector that holds the cell indices
				* \param x,y,s are the coordinates of the search cell
				**/
				//void getNeighbors(float x, float y, float z, float rsq, std::vector<size_t>& out) const;

				/** \brief Get filled cells within a certain distance
				*
				* \param rsq is the radius squared for the search
				* \param out is the output vector that holds the cell indices
				* \param index is the cell lattice point index
				**/
				//void getNeighbors(size_t index, float rsq, std::vector<size_t>& out) const;
		}
	}
}

