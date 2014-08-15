/// \brief Provides netcdf file IO
#define _SCL_SECURE_NO_WARNINGS

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <string>
#include <boost/algorithm/string/trim.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/data/arm_info.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-netcdf.h"
#include <netcdf.h>


namespace rtmath
{
	namespace registry
	{
		using std::shared_ptr;
		using namespace rtmath::plugins::netcdf;

		template<>
		shared_ptr<IOhandler>
			read_file_type_multi<::rtmath::data::arm::arm_info>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			::rtmath::data::arm::arm_info *s)
		{
			std::string filename = opts->filename();
			IOhandler::IOtype iotype = opts->getVal<IOhandler::IOtype>("iotype", IOhandler::IOtype::READONLY);
			//IOhandler::IOtype iotype = opts->iotype();
			std::string key = opts->getVal<std::string>("key");
			using std::shared_ptr;
			std::shared_ptr<netcdf_handle> h = registry::construct_handle
				<registry::IOhandler, netcdf_handle>(
				sh, PLUGINID, [&](){return std::shared_ptr<netcdf_handle>(
				new netcdf_handle(filename.c_str(), iotype)); });

			boost::filesystem::path pfile(filename);
			s->filesize = static_cast<size_t>(boost::filesystem::file_size(pfile));
			auto pfilename = pfile.filename();
			std::string sfilename = pfilename.string();
			s->filename = sfilename;

			auto getSiteFromFilename = [](const std::string &filename, std::string &site)
			{
				site = filename.substr(0, 3);
			};
			auto getSubsiteFromFilename = [](const std::string &filename, std::string &subsite)
			{
				auto fp = filename.find_first_of('.');
				auto ep = fp;
				fp--;
				// Most subsites follow C1, but some are like E12.
				size_t len = 1;
				while (std::isdigit(filename[fp])) { fp--; len++; }
				subsite = filename.substr(fp, len);
				return fp;
			};
			auto getDatalevelFromFilename = [](const std::string &filename, std::string &datalevel)
			{
				auto fp = filename.find_first_of('.');
				auto ep = fp;
				fp++;
				datalevel = filename.substr(fp, 2);
				return fp;
			};
			getSiteFromFilename(sfilename, s->site); // short site name
			auto iSubsite = getSubsiteFromFilename(sfilename, s->subsite); // short subsite id
			getDatalevelFromFilename(sfilename, s->datalevel); // short datalevel
			s->stream = sfilename.substr(3, iSubsite - 3); // ARM-based stream name

			const char* prods[] = { "mwr", "arscl", "aeri", "irt", "915rwp", "disdrometer", "rss",
				"rl", "mpl", "sonde", "sirs", "qcrad", "pbl", "surf", "aerosol", "aos", "twr",
				"mfrsr", "mergesonde", "swf", "aip", "sas", "rlccn", "surfspecalb", "tdmaaps", "armbe",
				"armbe", "arscl", "mmcrmode", // all mmcrmodde go under arscl
				"microbase", "ripbe", "rwp", "wsi", "tlcv", "sirs", "bsrn", "nimfr", "mfr" // must go after mfrsr
				"org", "rain", "sws", "thwaps", "tsi", "vceil", "vdis", "dl", "tdma", "co2flx",
				"pgs", "mmcr", "swacr", "mwacr", "wacr", "nfov", "brs", "kazr", "sasze", "csphot",
				"kasacr", "wsacr", "g12", "iap", "g8", "blc", "ceil" // alias for vceil
				"xsacr", "xsapr", "csapr", "co", "ls", "" };

			int i = 0;
			while (prods[i] != "" && sfilename.find(std::string(prods[i])) == std::string::npos) i++;
			
			s->product = std::string(prods[i]);
			if (s->product == "mmcrmode") s->product = "arscl";
			if (s->product == "ceil") s->product = "vceil";
			s->productFull = s->product;
			//s->hash = rtmath::HASHfile(filename);
			
			auto getAttrString = [&](const char* name, int varid) -> std::string
			{
				int status = 0;
				size_t len = 0;
				status = nc_inq_attlen(h->file, varid, name, &len);
				if (status) h->handle_error(status);
				std::unique_ptr<char[]> txtbuf(new char[len + 1]);
				status = nc_get_att_text(h->file, varid, name, txtbuf.get());
				if (status) h->handle_error(status);
				std::string res(txtbuf.get(), len + 1); // The netcdf storage of some of the arm attributes was flawed
				return res;
			};

			if (AttrExists(h->file, NC_GLOBAL, "facility_id").first)
				s->subsiteFull = getAttrString("facility_id", NC_GLOBAL);
			else s->subsiteFull = s->subsite;

			// Read in datasets lat, lon, alt, base_time, time_offsets
			auto time_offsets = getMatrix<double>("time_offset", h);
			auto base_time = getMatrix<int>("base_time", h);
			using namespace boost::posix_time;
			using namespace boost::gregorian;
			date b(1970, Jan, 1);
			s->startTime = ptime(b, seconds(static_cast<long>(base_time(0, 0))));

			s->endTime = s->startTime + seconds(static_cast<long>(time_offsets.bottomRows(1)(0, 0)));
			try {
				auto lat = getMatrix<float>("lat", h);
				s->lat = lat(0, 0);
				auto lon = getMatrix<float>("lon", h);
				s->lon = lon(0, 0);
				auto alt = getMatrix<float>("alt", h);
				s->alt = alt(0, 0);
			}
			catch (debug::xArrayOutOfBounds&)
			{
				// Just skip over the fields
				s->lat = 0;
				s->lon = 0;
				s->alt = 0;
			}

			return h;
		}

		/*
		template<>
		shared_ptr<IOhandler>
			read_file_type_multi<::rtmath::data::arm::arm_info>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			::rtmath::data::arm::arm_info *s)
		{
				std::string filename = opts->filename();
				IOhandler::IOtype iotype = opts->getVal<IOhandler::IOtype>("iotype", IOhandler::IOtype::READONLY);
				//IOhandler::IOtype iotype = opts->iotype();
				std::string key = opts->getVal<std::string>("key");
				using std::shared_ptr;
				std::shared_ptr<netcdf_handle> h;
				if (!sh)
				{
					h = std::shared_ptr<netcdf_handle>(new netcdf_handle(filename.c_str(), iotype));
				}
				else {
					if (sh->getId() != PLUGINID) RTthrow debug::xDuplicateHook("Bad passed plugin");
					h = std::dynamic_pointer_cast<netcdf_handle>(sh);
				}


				//s->hash = rtmath::HASHfile(filename);
				// Product id from filename
				boost::filesystem::path pfile(filename);
				s->filesize = static_cast<size_t>(boost::filesystem::file_size(pfile));
				auto pfilename = pfile.filename();
				std::string sfilename = pfilename.string();
				s->filename = sfilename;
				// The filename follows the pattern {nsa}{product}{subsite(2 chars)}. ...
				s->productFull = sfilename.substr(3, sfilename.find_first_of('.') - 5);

				auto getAttrString = [&](const char* name, int varid) -> std::string
				{
					int status = 0;
					size_t len = 0;
					status = nc_inq_attlen(h->file, varid, name, &len);
					if (status) h->handle_error(status);
					std::unique_ptr<char[]> txtbuf(new char[len + 1]);
					status = nc_get_att_text(h->file, varid, name, txtbuf.get());
					if (status) h->handle_error(status);
					std::string res(txtbuf.get(), len+1); // The netcdf storage of some of the arm attributes was flawed
					return res;
				};

				auto getSubsiteFromFilename = [](const std::string &filename, std::string &subsite)
				{
					auto fp = filename.find_first_of('.');
					auto ep = fp;
					fp--;
					// Most subsites follow C1, but some are like E12.
					size_t len = 1;
					while (std::isdigit(filename[fp])) { fp--; len++; }
					subsite = filename.substr(fp, len);
				};

				// Read attributes site_id, facility_id, data_level
				// Add sanity checks here to filter out early data, like sbsswacr without any of these attributes.
				if (AttrExists(h->file, NC_GLOBAL, "site_id").first)
				{
					s->site = getAttrString("site_id", NC_GLOBAL);
					if (s->site.length() > 3) s->site = s->filename.substr(0,3); // pyemwrlos error in netcdf attribute
					s->subsiteFull = getAttrString("facility_id", NC_GLOBAL);
					//s->subsite = s->subsiteFull.substr(0, 2);
					getSubsiteFromFilename(sfilename, s->subsite); // sfilename.substr(sfilename.find_first_of('.') - 2, 2); // mmcrspeccmaskblC1.a0... inconsistency
					//if (AttrExists(h->file, NC_GLOBAL, "proc_level").first) // proc_level failed for tsiskycover... annoying.
					//	s->datalevel = getAttrString("proc_level", NC_GLOBAL);
					//else {
						// Pull from filename. It is after the first period.
						s->datalevel = sfilename.substr(sfilename.find_first_of('.') + 1, 2);
					//}
					if (s->datalevel.length() > 2) s->datalevel = s->datalevel.substr(0,2); // pyemwrlos error in netcdf attribute
				} else {
					// Annoying Steamboat Springs data...
					s->site = s->filename.substr(0,3);
					getSubsiteFromFilename(sfilename, s->subsite); //s->subsite = sfilename.substr(sfilename.find_first_of('.') - 2, 2);
					s->subsiteFull = s->subsite;
					s->datalevel = sfilename.substr(sfilename.find_first_of('.') + 1, 2);
				}

				// Attempt to separate product name from stream
				// Unfortunately, ARM cdf files store this indirectly and inconsistently
				{
					std::pair<bool, int> attrInputSource, attrIngest, attrCmd;

					// gndrad, skyrad, swacr (fails for mfr)
					attrInputSource = AttrExists(h->file, NC_GLOBAL, "input_source");
					// mwr, gvr, mfr, (mfrsr fails), (skyrad fails)
					attrIngest = AttrExists(h->file, NC_GLOBAL, "ingest_software");
					// command-line reading (beflux1long)
					attrCmd = AttrExists(h->file, NC_GLOBAL, "Command_Line");

					using std::string;
					if (attrInputSource.first)
					{
						string astr = getAttrString("input_source", NC_GLOBAL);
						// String format like /data/collection//nsa/nsagndradC1.00/20050102000000.dat 
						// Find subsite and scan backwards to the previous '/'.
						// The full product + stream ID is known from the filename. The substr has only the 
						// product, so the stream name can be found.

						auto subsiteLoc = astr.find(s->subsite);
						if (subsiteLoc == string::npos) 
							if (AttrExists(h->file, NC_GLOBAL, "ingest_software").first)
							{
								astr = getAttrString("zeb_platform", NC_GLOBAL);
								subsiteLoc = astr.find(s->subsite);
								if (subsiteLoc == string::npos) 
									RTthrow debug::xBadInput(filename.c_str());
							} else {
								RTthrow debug::xBadInput(filename.c_str());
							}
						auto pstart = astr.rfind('/', subsiteLoc);
						if (pstart == string::npos)
							pstart = 3;
						else pstart += 4; // Pass beyond /nsa (site id).
						s->product = astr.substr(pstart, subsiteLoc - pstart);

						// Now that the product is known, the stream can also be extracted
						s->stream = s->productFull.substr(s->product.size());
						
						// Correct if "command_line" or "ingest_software" is present and is readable
						if (attrIngest.first)
						{
							string astr = getAttrString("ingest_software", NC_GLOBAL);
							boost::algorithm::trim(astr);
							if (astr.find("sirs") != string::npos) { // gndrad falls through
							} else if (astr.find("rwp") != string::npos) { // 915rwp falls through
							} else if (astr.substr(0,7) == "ingest-") { // several
								string newproduct = astr.substr(7, astr.find_first_of('-',8) - 7);
								s->stream = s->filename.substr(newproduct.size() + 3, sfilename.find_first_of('.') - newproduct.size() - 3 - 2);
								//s->stream = s->product.substr(newproduct.size());
								s->product = newproduct;
							} else if (astr.find("_filter") != string::npos) { // mmcrspecmask
								string newproduct = astr.substr(0, astr.find_first_of('_',0));
								s->stream = s->filename.substr(newproduct.size() + 3, sfilename.find_first_of('.') - newproduct.size() - 3 - 2);
								//s->stream = s->product.substr(newproduct.size());
								s->product = newproduct;
							} else if (astr.find("_ingest.c") != string::npos) { // mwr, mfr (and NOT gndrad)
								string newproduct = astr.substr(0, astr.find_first_of('_',0));
								if (newproduct == "mfrcdl") newproduct = "mfr"; // mfrsr is labeled differently inside the file
								s->stream = s->filename.substr(newproduct.size() + 3, sfilename.find_first_of('.') - newproduct.size() - 3 - 2);
								//s->stream = s->product.substr(newproduct.size());
								s->product = newproduct;
							}
						}
					}
					else if (attrIngest.first)
					{
						string astr = getAttrString("ingest_software", NC_GLOBAL);
						// Need to remove any leading spaces from the retrieved string
						boost::algorithm::trim(astr);

						// String format like mwr_ingest.c,v 1.21 ...
						// or mfrcdl_ingest.c,v 1.27 (note the old prodct name...)
						// The goal is to pull out enough matching information
						auto firstUnderscore = astr.find('_');
						if (firstUnderscore == string::npos) RTthrow debug::xBadInput(astr.c_str());
						//string pcand = astr.substr(0, firstUnderscore);
						// The product is the part of pcand that matches s->productFull.
						size_t i = 0;
						while (astr.length() > i && s->productFull.length() > i)
						{
							if (astr.at(i) == s->productFull.at(i)) ++i;
							else break;
						}
						s->product = astr.substr(0, i);
						//auto matchres = std::mismatch(astr.begin(), astr.end(), s->productFull.begin());
						//std::copy(pcand.begin(), matchres.first, s->product.begin());
						s->stream = s->productFull.substr(s->product.size());
						i = 0;
					} else if (s->filename.find("swacr") != string::npos) {
						s->product = "swacr";
						s->stream = s->filename.substr(s->product.size() + 3, sfilename.find_first_of('.') - s->product.size() - 3 - 2);
					} else if (s->filename.find("aeri01") != string::npos) {
						s->product = "aeri01";
						s->stream = s->filename.substr(s->product.size() + 3, sfilename.find_first_of('.') - s->product.size() - 3 - 2);
					} else if (s->filename.find("aeri") != string::npos) {
						s->product = "aeri";
						s->stream = s->filename.substr(s->product.size() + 3, sfilename.find_first_of('.') - s->product.size() - 3 - 2);
					} else if (s->filename.find("mmcr") != string::npos) {
						s->product = "mmcr";
						s->stream = s->filename.substr(s->product.size() + 3, sfilename.find_first_of('.') - s->product.size() - 3 - 2);
					} else if (s->filename.find("mwr") != string::npos) {
						s->product = "mwr";
						s->stream = s->filename.substr(s->product.size() + 3, sfilename.find_first_of('.') - s->product.size() - 3 - 2);
					} else if (attrCmd.first) {
						string astr = getAttrString("Command_Line", NC_GLOBAL);
						boost::algorithm::trim(astr);
						auto plastslash = astr.rfind('/');
						if (plastslash == string::npos) plastslash = 0;
						else plastslash++;
						auto pspace = astr.find(' ', plastslash);
						if (pspace == string::npos) pspace = astr.length() - 1;
						s->product = astr.substr(plastslash, pspace-plastslash);
						// No stream in this fall-through case
					} else {
						RTthrow debug::xArrayOutOfBounds();
					}
					
				}
				if (!s->subsiteFull.size()) s->subsiteFull = s->subsite;

				//if (!s->product.size() && s->stream.size())
				//	s->product.swap(s->stream);

				// Read in datasets lat, lon, alt, base_time, time_offsets
				auto time_offsets = getMatrix<double>("time_offset", h);
				auto base_time = getMatrix<int>("base_time", h);
				using namespace boost::posix_time;
				using namespace boost::gregorian;
				date b(1970, Jan, 1);
				s->startTime = ptime(b, seconds(static_cast<long>(base_time(0, 0))));

				s->endTime = s->startTime + seconds(static_cast<long>(time_offsets.bottomRows(1)(0, 0)));
				try {
					auto lat = getMatrix<float>("lat", h);
					s->lat = lat(0, 0);
					auto lon = getMatrix<float>("lon", h);
					s->lon = lon(0, 0);
					auto alt = getMatrix<float>("alt", h);
					s->alt = alt(0, 0);
				} catch (debug::xArrayOutOfBounds&)
				{
					// Just skip over the fields
					s->lat = 0;
					s->lon = 0;
					s->alt = 0;
				}

				//std::cerr << s->startTime << std::endl;
				//std::cerr << s->endTime << std::endl;

				return h;
		}
		*/

		template<>
		std::shared_ptr<IOhandler>
			read_file_type_vector<::rtmath::data::arm::arm_info>
			(std::shared_ptr<IOhandler> sh, std::shared_ptr<IO_options> opts,
			std::vector<boost::shared_ptr<::rtmath::data::arm::arm_info> > &s)
		{
				RTthrow debug::xUnimplementedFunction();
				return sh;
		}
	}
}
