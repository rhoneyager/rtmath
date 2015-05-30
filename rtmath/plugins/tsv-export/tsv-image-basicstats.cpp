/// \brief Provides tsv file IO
#define _SCL_SECURE_NO_WARNINGS

#pragma warning( disable : 4503 ) // decorated length exceeded

#include <array>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tuple>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>


#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/images/image.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-tsv.h"

#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

using namespace rtmath::registry;
namespace rtmath {
	namespace plugins {
		namespace tsv {
			using std::shared_ptr;
			using rtmath::ddscat::ddOutput;

			struct tsv_image_basicstats_handle : public rtmath::registry::IOhandler
			{
				tsv_image_basicstats_handle(const char* filename, IOtype t) : IOhandler(PLUGINID_IMAGE) { open(filename, t); }
				virtual ~tsv_image_basicstats_handle() {}
				void open(const char* filename, IOtype t)
				{
					using namespace boost::filesystem;
					switch (t)
					{
					case IOtype::EXCLUSIVE:
					case IOtype::DEBUG:
					case IOtype::READONLY:
						RDthrow(debug::xOtherError());
						break;
					case IOtype::CREATE:
						if (exists(path(filename))) RDthrow(debug::xFileExists());
					case IOtype::TRUNCATE:
						file = std::shared_ptr<std::ofstream>(new std::ofstream(filename, std::ios_base::trunc));
						writeHeader();
						break;
					case IOtype::READWRITE:
						{
							bool e = false;
							if (exists(path(filename))) e = true;
							file = std::shared_ptr<std::ofstream>(new std::ofstream(filename, std::ios_base::app));
							if (!e) writeHeader(); // If the file had to be created, give it a header
						}
						break;
					}
				}
				void writeHeader()
				{
					(*(file.get())) << "Filename\trows\tcols\t"
						"min_x\tmin_y\tmin_val\t"
						"max_x\tmax_y\tmax_val\t"
						"mean_x\tmean_y\tmean_val\t"
						"variance_x\tvariance_y\tvariance_val\t"
						"nTotal\tnFilled\tfrac"
						<< std::endl;
					;
				}
				std::shared_ptr<std::ofstream> file;
			};

			shared_ptr<IOhandler>
				export_tsv_image_basicstats
				(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
				const boost::shared_ptr<const rtmath::images::image > s)
			{
				std::string filename = opts->filename();
				IOhandler::IOtype iotype = opts->iotype();
				bool onlyFilename = opts->getVal<bool>("onlyFilename", false);


				using std::shared_ptr;
				std::shared_ptr<tsv_image_basicstats_handle> h;
				if (!sh)
				{
					h = std::shared_ptr<tsv_image_basicstats_handle>(new tsv_image_basicstats_handle(filename.c_str(), iotype));
				}
				else {
					if (sh->getId() != PLUGINID_IMAGE) RDthrow(debug::xDuplicateHook());
					h = std::dynamic_pointer_cast<tsv_image_basicstats_handle>(sh);
				}

				std::string source = opts->getVal<std::string>("source", "");

				boost::filesystem::path psource(source);
				if (onlyFilename) psource = psource.filename();

				(*(h->file.get())) << psource.string() << "\t" << s->rows << "\t" << s->cols << "\t"
					<< s->mins(0) << "\t" << s->mins(1) << "\t" << s->mins(2) << "\t"
					<< s->maxs(0) << "\t" << s->maxs(1) << "\t" << s->maxs(2) << "\t"
					<< s->means(0) << "\t" << s->means(1) << "\t" << s->means(2) << "\t"
					<< s->variances(0) << "\t" << s->variances(1) << "\t" << s->variances(2) << "\t"
					<< s->numTotal << "\t" << s->numFilled << "\t" << s->frac
					<< std::endl;

				return h; // Pass back the handle
			}


		}
	}

	namespace registry {
		using rtmath::images::image;

		template<>
		std::shared_ptr<IOhandler>
			write_file_type_multi<rtmath::images::image>
			(std::shared_ptr<IOhandler> sh, std::shared_ptr<IO_options> opts,
			const boost::shared_ptr<const rtmath::images::image > s)
		{
			std::string exporttype = opts->exportType();
			if (exporttype == "image_basicstats") return ::rtmath::plugins::tsv::export_tsv_image_basicstats(sh, opts, s);
			else { RDthrow(debug::xUnimplementedFunction()); }
			return nullptr;
		}


	}
}
