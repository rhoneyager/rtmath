/// \brief Provides tsv file IO
#define _SCL_SECURE_NO_WARNINGS


#include <array>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tuple>


#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/plugin.h"
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>

#include "plugin-tsv.h"

using std::shared_ptr;
namespace rtmath {
	namespace plugins {
		namespace tsv {
			shared_ptr<::Ryan_Debug::registry::IOhandler>
				export_tsv_ddori_iso_data
				(shared_ptr<::Ryan_Debug::registry::IOhandler> sh, shared_ptr<::Ryan_Debug::registry::IO_options> opts,
				const boost::shared_ptr<const ::rtmath::ddscat::ddOutput > s);

			shared_ptr<::Ryan_Debug::registry::IOhandler>
				export_tsv_ddori_iso_small_data
				(shared_ptr<::Ryan_Debug::registry::IOhandler> sh, shared_ptr<::Ryan_Debug::registry::IO_options> opts,
				const boost::shared_ptr<const ::rtmath::ddscat::ddOutput > s);

			shared_ptr<::Ryan_Debug::registry::IOhandler>
				export_tsv_ddori_ori_data
				(shared_ptr<::Ryan_Debug::registry::IOhandler> sh, shared_ptr<::Ryan_Debug::registry::IO_options> opts,
				const boost::shared_ptr<const ::rtmath::ddscat::ddOutput > s);

			shared_ptr<::Ryan_Debug::registry::IOhandler>
				export_tsv_ddori_stats
				(shared_ptr<::Ryan_Debug::registry::IOhandler> sh, shared_ptr<::Ryan_Debug::registry::IO_options> opts,
				const boost::shared_ptr<const ::rtmath::ddscat::ddOutput >s);
		}
	}
}
namespace Ryan_Debug {
	namespace registry {
		using rtmath::ddscat::ddOutput;

		template<>
		shared_ptr<IOhandler>
			write_file_type_multi<::rtmath::ddscat::ddOutput>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			const boost::shared_ptr<const ::rtmath::ddscat::ddOutput > s)
		{
			std::string exporttype = opts->exportType();
			if (exporttype == "isotropic_data") return ::rtmath::plugins::tsv::export_tsv_ddori_iso_data(sh, opts, s);
			else if (exporttype == "isotropic_data_small") return ::rtmath::plugins::tsv::export_tsv_ddori_iso_small_data(sh, opts, s);
			else if (exporttype == "orientation_data") return ::rtmath::plugins::tsv::export_tsv_ddori_ori_data(sh, opts, s);
			else if (exporttype == "stats") return ::rtmath::plugins::tsv::export_tsv_ddori_stats(sh, opts, s);
			else { RDthrow(Ryan_Debug::error::xUnimplementedFunction()); }
			return nullptr;
		}


	}
}
