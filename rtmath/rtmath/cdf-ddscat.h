/*
 * cdf-ddscat.h
 *
 *  Created on: Nov 18, 2011
 *      Author: rhoneyager
 */
#pragma once

namespace rtmath {
	namespace ddscat {

		namespace cdf {
			enum cdfParam
			{
				// File id
				fid,
				// Dimension ids
				dtheta,
				dphi,
				didnum,
				// Dimension Variable ids
				theta,
				phi,
				idnum,
				// Other Variable ids
				S, // time is real/imag 11,12,21,22

				P, // time is 11,12,13,14,21,...

				K, // time is 11,12,13,14,21,...

				NUM_PARAMS

			};
		}; // end cdf

		class cdfParams
		{
		public:
			cdfParams()
			{
				using namespace cdf;
				for (int i=0; i<NUM_PARAMS;i++)
					p[i] = -1;
			}
			int p[cdf::NUM_PARAMS];
		};

	}; // end ddscat
}; // end rtmath
