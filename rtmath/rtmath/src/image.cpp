#include "Stdafx-images.h"
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include "../rtmath/images/image.h"

#undef min
#undef max

namespace rtmath {
	namespace registry {
		template struct IO_class_registry_writer<
			::rtmath::images::image>;

		template struct IO_class_registry_reader<
			::rtmath::images::image>;

		template class usesDLLregistry<
			::rtmath::images::image_IO_input_registry,
			IO_class_registry_reader<::rtmath::images::image> >;

		template class usesDLLregistry<
			::rtmath::images::image_IO_output_registry,
			IO_class_registry_writer<::rtmath::images::image> >;
	}

	namespace images {

		image::~image() {}

		image::image(const std::string &filename)
		{
			this->read(filename);
		}

		image::image() {}

		void image::doStats()
		{
			auto mat = imageMaps.at("raw");
			using namespace boost::accumulators;
			accumulator_set<float, boost::accumulators::stats
				<tag::mean, tag::min, tag::max, tag::variance> > m_x, m_y, m_val;
			numFilled = 0;
			numTotal = (size_t) (mat->rows() * mat->cols());
			rows = (size_t) mat->rows();
			cols = (size_t) mat->cols();
			// Excluding the outermost pixels since the raw images have a pizel or two set in a corner.
			for (int i = 1; i < mat->rows()-1; i++)
				for (int j=1; j < mat->cols()-1; j++)
			{
				float val = (*mat)(i,j);
				if (val > 0.05)
				{
					m_x((float) (i));
					m_y((float) (j));
					m_val(val);
					numFilled++;
				}
			}
			mins(0) = boost::accumulators::min(m_x);
			mins(1) = boost::accumulators::min(m_y);
			mins(2) = boost::accumulators::min(m_val);

			maxs(0) = boost::accumulators::max(m_x);
			maxs(1) = boost::accumulators::max(m_y);
			maxs(2) = boost::accumulators::max(m_val);

			means(0) = boost::accumulators::mean(m_x);
			means(1) = boost::accumulators::mean(m_y);
			means(2) = boost::accumulators::mean(m_val);

			variances(0) = boost::accumulators::variance(m_x);
			variances(1) = boost::accumulators::variance(m_y);
			variances(2) = boost::accumulators::variance(m_val);

			frac = ((float) numFilled) / ((float) numTotal);
		}
	}
}
