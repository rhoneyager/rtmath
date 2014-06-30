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
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-tsv.h"

using std::shared_ptr;
namespace rtmath {
	namespace plugins {
		namespace tsv {
			shared_ptr<::rtmath::registry::IOhandler>
				export_tsv_ddori_iso_data
				(shared_ptr<::rtmath::registry::IOhandler> sh, shared_ptr<::rtmath::registry::IO_options> opts,
				const ::rtmath::ddscat::ddOutput *s);

			shared_ptr<::rtmath::registry::IOhandler>
				export_tsv_ddori_ori_data
				(shared_ptr<::rtmath::registry::IOhandler> sh, shared_ptr<::rtmath::registry::IO_options> opts,
				const ::rtmath::ddscat::ddOutput *s);
		}
	}
	namespace registry {
		using rtmath::ddscat::ddOutput;

		template<>
		shared_ptr<IOhandler>
			write_file_type_multi<::rtmath::ddscat::ddOutput>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			const ::rtmath::ddscat::ddOutput *s)
		{
			std::string exporttype = opts->exportType();
			if (exporttype == "isotropic_data") return ::rtmath::plugins::tsv::export_tsv_ddori_iso_data(sh, opts, s);
			else if (exporttype == "orientation_data") return ::rtmath::plugins::tsv::export_tsv_ddori_ori_data(sh, opts, s);
			else { RTthrow debug::xUnimplementedFunction(); }
			return nullptr;
		}


	}
}
