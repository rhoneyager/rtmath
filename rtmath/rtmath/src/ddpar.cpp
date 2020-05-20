#include "Stdafx-ddscat_base.h"
#include "../rtmath/defs.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <complex>
#include <boost/algorithm/string.hpp>
#include <boost/iostreams/filter/newline.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <cmath>
#include <Ryan_Debug/Serialization.h>
#include "../rtmath/ddscat/ddpar.h"
#include "../rtmath/ddscat/ddVersions.h"
#include <Ryan_Debug/config.h>
#include "../rtmath/config.h"
#include <Ryan_Debug/splitSet.h>
#include "../rtmath/ddscat/rotations.h"
#include <Ryan_Debug/error.h>

namespace {
	
	boost::filesystem::path pDefaultPar;
	const std::string ddparDefaultInternal = 
		"' ========= Parameter file for v7.3 =================== '\n"
		"'**** Preliminaries ****'\n"
		"'NOTORQ' = CMTORQ * 6 (NOTORQ, DOTORQ) --either do or skip torque calculations\n"
		"'PBCGS2' = CMDSOL * 6 (PBCGS2, PBCGST, PETRKP) --select solution method\n"
		"'GPFAFT' = CMDFFT * 6 (GPFAFT, FFTMKL)-- - FFT method\n"
		"'GKDLDR' = CALPHA * 6 (GKDLDR, LATTDR)\n"
		"'NOTBIN' = CBINFLAG(NOTBIN, ORIBIN, ALLBIN)\n"
		"'**** Initial Memory Allocation ****'\n"
		"101 101 101 = dimension\n"
		"'**** Target Geometry and Composition ****'\n"
		"'FROM_FILE' = CSHAPE * 9 shape directive\n"
		"101 101 101 = shape parameters 1 - 3\n"
		"1 = NCOMP = number of dielectric materials\n"
		"'diel.tab' = file with refractive index 1\n"
		"'**** Additional Nearfield calculation? ****'\n"
		"0 = NRFLD(= 0 to skip nearfield calc., = 1 to calculate nearfield E)\n"
		"0 0 0 0 0 0 = (fract.extens.of calc.vol.in - x, +x, -y, +y, -z, +z)\n"
		"'**** Error Tolerance ****'\n"
		"1e-05 = TOL = MAX ALLOWED(NORM OF | G >= AC | E>-ACA | X>) / (NORM OF AC | E>)\n"
		"'**** Maximum number of iterations ****'\n"
		"300 = MXITER\n"
		"'**** Integration cutoff parameter for PBC calculations ****'\n"
		"0.005 = GAMMA(1e-2 is normal, 3e-3 for greater accuracy)\n"
		"'**** Angular resolution for calculation of <cos>, etc. ****'\n"
		"0.5 = ETASCA(number of angles is proportional to[(3 + x) / ETASCA] ^ 2)\n"
		"'**** Vacuum wavelengths (micron) ****'\n"
		"3189.28 3189.28 1 'LIN' = wavelengths\n"
		"'**** Refractive index of ambient medium'\n"
		"1 = NAMBIENT\n"
		"'**** Effective Radii (micron) **** '\n"
		"341.49 341.49 1 'LIN' = aeff\n"
		"'**** Define Incident Polarizations ****'\n"
		"(0, 0) (1, 0) (0, 0) = Polarization state e01(k along x axis)\n"
		"2 = IORTH(= 1 to do only pol.state e01; = 2 to also do orth.pol.state)\n"
		"'**** Specify which output files to write ****'\n"
		"1 = IWRKSC(= 0 to suppress, = 1 to write \".sca\" file for each target orient.\n"
		"'**** Specify Target Rotations ****'\n"
		"0. 360. 18 = BETAMI, BETAMX, NBETA(beta = rotation around a1)\n"
		"0. 180. 19 = THETMI, THETMX, NTHETA(theta = angle between a1 and k)\n"
		"0. 360. 18 = PHIMIN, PHIMAX, NPHI(phi = rotation angle of a1 around k)\n"
		"'**** Specify first IWAV, IRAD, IORI (normally 0 0 0) ****'\n"
		"0 0 0 = first IWAV, first IRAD, first IORI(0 0 0 to begin fresh)\n"
		"'**** Select Elements of S_ij Matrix to Print ****'\n"
		"6 = NSMELTS = number of elements of S_ij to print(not more than 9)\n"
		"11 12 21 22 31 41 = indices ij of elements to print\n"
		"'**** Specify Scattered Directions ****'\n"
		"'LFRAME' = CMDFRM(LFRAME, TFRAME for Lab Frame or Target Frame)\n"
		"2 = NPLANES = number of scattering planes\n"
		"0 0 180 10 = phi, thetan_min, thetan_max, dtheta(in deg) for plane 1\n"
		"90 0 180 10 = phi, thetan_min, thetan_max, dtheta(in deg) for plane 2\n"
		;

	/// \todo This function should contain much of ddPar::defaultInstance
	void initPaths()
	{
		static bool loaded = false;
		if (loaded) return;
		using std::string;
		using namespace rtmath;
		using boost::filesystem::path;

		try {
			// First try to load using rtmath.conf location
			auto cRoot = rtmath::config::loadRtconfRoot();
			if (!cRoot) RDthrow(Ryan_Debug::error::xMissingRyan_DebugConf());
			auto ccnf = cRoot->getChild("RTMATH");
			if (!ccnf) RDthrow(Ryan_Debug::error::xMissingRyan_DebugConf());
			auto rtddscat = ccnf->getChild("ddscat");

			string sBasePar, scwd;
			if (rtddscat) {
				rtddscat->getVal<string>("DefaultFile", sBasePar);
				rtddscat->getCWD(scwd);
			} else RDthrow(Ryan_Debug::error::xBadInput())
				<< Ryan_Debug::error::otherErrorText("rtmath configutation file missing ddscat branch");

			path pscwd(scwd), psBasePar(sBasePar);
			pscwd.remove_filename();

			if (psBasePar.is_relative()) psBasePar = pscwd / psBasePar;
			pDefaultPar = psBasePar;
		} catch (std::exception&)
		{
			// If rtmath.conf cannot be found, or if the loading fails, default to the internal file.
		}

		loaded = true;
	}

	std::set<std::string> mtypes;
	std::mutex mlock;
}


namespace Ryan_Debug {
	namespace registry {
		template struct IO_class_registry_writer
			< ::rtmath::ddscat::ddPar > ;
		template class usesDLLregistry <
			::rtmath::ddscat::ddPar_IO_output_registry,
			IO_class_registry_writer<::rtmath::ddscat::ddPar> > ;
		template struct IO_class_registry_reader
			< ::rtmath::ddscat::ddPar > ;
		template class usesDLLregistry <
			::rtmath::ddscat::ddPar_IO_input_registry,
			IO_class_registry_reader<::rtmath::ddscat::ddPar> > ;
	}
	namespace io {
		template <>
		boost::shared_ptr<::rtmath::ddscat::ddPar> customGenerator()
		{
			boost::shared_ptr<::rtmath::ddscat::ddPar> res
				= ::rtmath::ddscat::ddPar::generate();
			//	(new ::rtmath::ddscat::ddPar);
			return res;
		}
	}
}
namespace rtmath {
	namespace ddscat {

		implementsDDPAR::implementsDDPAR() :
			Ryan_Debug::io::implementsIObasic<ddPar, ddPar_IO_output_registry,
			ddPar_IO_input_registry, ddPar_Standard>(ddPar::writeDDSCAT, ddPar::readDDSCATdef, known_formats())
		{}

		const std::set<std::string>& implementsDDPAR::known_formats()
		{
			// Moved to hidden file scope to avoid race condition
			//static std::set<std::string> mtypes;
			//static std::mutex mlock;
			// Prevent threading clashes
			{
				std::lock_guard<std::mutex> lck(mlock);
				if (!mtypes.size())
					mtypes.insert(".par");
				if (Ryan_Debug::io::TextFiles::serialization_handle::compressionEnabled())
				{
					std::string sctypes;
					std::set<std::string> ctypes;
					Ryan_Debug::serialization::known_compressions(sctypes, ".par");
					Ryan_Debug::splitSet::splitSet(sctypes, ctypes);
					for (const auto & t : ctypes)
						mtypes.emplace(t);
				}
			}
			return mtypes;
		}

		ddPar::ddPar()
		{
			_init();
			// Fill in the blanks from the default file. Needed here to avoid
			// crashed associated with accessing missing keys.
			//_populateDefaults(false);
		}
		/*
		ddPar::ddPar()
		{
			_init();
			_populateDefaults(false);
		}
		*/

		boost::shared_ptr<ddPar> ddPar::generate(const boost::shared_ptr<const ddPar> src)
		{
			boost::shared_ptr<ddPar> res(new ddPar(*(src.get())));
			return res;
		}

		boost::shared_ptr<ddPar> ddPar::generate()
		{
			boost::shared_ptr<ddPar> res(new ddPar);
			return res;
		}

		boost::shared_ptr<ddPar> ddPar::generate(const std::string &filename, bool popDefaults)
		{
			boost::shared_ptr<ddPar> res(new ddPar);
			res->readFile(filename);
			if (popDefaults)
				res->populateDefaults(false);
			return res;
		}

		ddPar::~ddPar()
		{
		}

		bool ddPar::operator==(const ddPar &rhs) const
		{
			std::string sThis, sRhs;
			std::ostringstream oThis, oRhs;
			writeDDSCAT(this->shared_from_this(), oThis, nullptr);
			writeDDSCAT(rhs.shared_from_this(), oRhs, nullptr);
			sThis = oThis.str();
			sRhs = oRhs.str();
			return (sThis == sRhs);
		}

		bool ddPar::operator!=(const ddPar &rhs) const
		{
			return !(operator==(rhs));
		}

		ddPar & ddPar::operator=(const ddPar &rhs)
		{
			if (this != &rhs)
			{
				_version = rhs._version;
				std::ostringstream out;
				writeDDSCAT(rhs.shared_from_this(), out, nullptr);
				std::string data = out.str();
				std::istringstream in(data);
				read(in);
			}
			return *this;
		}

		ddPar::ddPar(const ddPar &src)
		{
			// Expensive copy constructor. Implements cloning to avoid screwups.
			_version = src._version;
			std::ostringstream out;
			writeDDSCAT(src.shared_from_this(), out, nullptr);
			std::string data = out.str();
			std::istringstream in(data);
			read(in);
		}

		boost::shared_ptr<ddPar> ddPar::clone() const
		{
			boost::shared_ptr<ddPar> lhs(new ddPar);

			lhs->_version = _version;
			
			std::ostringstream out;
			writeDDSCAT(this->shared_from_this(), out, nullptr);
			std::string data = out.str();
			std::istringstream in(data);

			lhs->read(in);

			return lhs;
		}

		Ryan_Debug::hash::HASH_t ddPar::hash() const
		{
			std::string res;
			std::ostringstream out;
			write(out);
			res = out.str();
			return Ryan_Debug::hash::HASH(res.c_str(), (int)res.size());
		}

		/*
		void ddPar::readFile(const std::string &filename, bool overlay)
		{
			// Check file existence
			using namespace std;
			using namespace boost::filesystem;
			using namespace Ryan_Serialization;
			std::string cmeth, target, uncompressed;
			// Combination of detection of compressed file, file type and existence.
			if (!detect_compressed(filename, cmeth, target))
				throw rtmath::debug::xMissingFile(filename.c_str());
			uncompressed_name(target, uncompressed, cmeth);

			boost::filesystem::path p(uncompressed);
			boost::filesystem::path pext = p.extension(); // Uncompressed extension

			// Serialization gets its own override
			if (Ryan_Serialization::known_format(pext))
			{
				// This is a serialized file. Verify that it has the correct identifier, and 
				// load the serialized object directly
				Ryan_Serialization::read<ddPar>(*this, filename, "rtmath::ddscat::ddPar");
				return;
			}

			this->_filename = filename;

			std::ifstream in(filename.c_str(), std::ios_base::binary | std::ios_base::in);
			// Consutuct an filtering_iostream that matches the type of compression used.
			using namespace boost::iostreams;
			filtering_istream sin;
			if (cmeth.size())
				prep_decompression(cmeth, sin);
			sin.push(boost::iostreams::newline_filter(boost::iostreams::newline::posix));
			sin.push(in);

			read(sin, overlay);
		}

		
		void ddPar::writeFile(const std::string &filename, const std::string &type) const
		{
			populateDefaults();
			//std::ofstream out(filename.c_str());
			//write(out);


			using namespace Ryan_Serialization;
			std::string cmeth, uncompressed;
			uncompressed_name(filename, uncompressed, cmeth);
			boost::filesystem::path p(uncompressed);
			boost::filesystem::path pext = p.extension(); // Uncompressed extension

			std::string utype = type;
			if (!utype.size()) utype = pext.string();

			// Serialization gets its own override
			if (Ryan_Serialization::known_format(utype))
			{
				Ryan_Serialization::write<ddPar>(*this, filename, "rtmath::ddscat::ddPar");
				return;
			}

			std::ofstream out(filename.c_str(), std::ios_base::out | std::ios_base::binary);
			using namespace boost::iostreams;
			filtering_ostream sout;
			if (cmeth.size())
				prep_compression(cmeth, sout);

			sout.push(boost::iostreams::newline_filter(boost::iostreams::newline::posix));
			sout.push(out);
			write(sout);
		}
		*/

		void ddPar::writeDDSCAT(const boost::shared_ptr<const ddPar> p, std::ostream &out, std::shared_ptr<Ryan_Debug::registry::IO_options> opts)
		{
			// Writing is much easier than reading!
			using namespace std;

			// Ensute that all necessary keys exist. If not, create them!!!
			//populateDefaults(); // User's responsibility

			if (!p) RDthrow(Ryan_Debug::error::xNullPointer())
				<< Ryan_Debug::error::otherErrorText("ddPar::writeDDSCAT::p is null");
			// Write file version
			string ver;
			ver = rtmath::ddscat::ddVersions::getVerString(p->_version);
			out << "' ========= Parameter file for v" << ver << " =================== '" << endl;

			// Loop through and write parameters and comments
			for (auto it = p->_parsedData.begin(); it != p->_parsedData.end(); it++)
			{
				// If key is valid for this output version, write it
				if (it->second->versionValid(p->_version))
					it->second->write(out, p->_version);

				// Check here for dielectric write. Always goes after NCOMP.
				if (it->first == ddParParsers::NCOMP)
				{
					int i = 1;
					for (auto ot = p->_diels.begin(); ot != p->_diels.end(); ++ot, ++i)
					{
						ostringstream o;
						// "...file with refractive index" + " #"
						o << " " << i;
						string plid = o.str();
						(*ot)->write(out, p->_version, plid);
					}
				}
			}
			for (auto ot = p->_scaPlanes.begin(); ot != p->_scaPlanes.end(); ++ot)
			{
				// If key is valid for this output version, write it
				if (ot->second->versionValid(p->_version))
				{
					ostringstream o;
					// "...for plane" + " #"
					o << " " << boost::lexical_cast<std::string>(ot->first);
					string plid = o.str();
					ot->second->write(out, p->_version, plid);
				}
			}
		}

		void ddPar::_init()
		{
			_version = rtmath::ddscat::ddVersions::getDefaultVer();
#if USE_RYAN_SERIALIZATION
			::rtmath::io::Serialization::implementsSerialization<
				::rtmath::ddscat::ddPar, ddPar_IO_output_registry, 
				ddPar_IO_input_registry, ddPar_serialization>::set_sname("rtmath::ddscat::ddpar");
#endif
		}

		void ddPar::readDDSCAT(boost::shared_ptr<ddPar> src, std::istream &in, bool overlay)
		{
			src->read(in, overlay);
		}

		void ddPar::readDDSCATdef(boost::shared_ptr<ddPar> src, std::istream &in, std::shared_ptr<Ryan_Debug::registry::IO_options>)
		{
			readDDSCAT(src, in, false);
		}

		void ddPar::write(std::ostream& out) const
		{
			writeDDSCAT(this->shared_from_this(), out, nullptr);
		}

		void ddPar::read(std::istream &stream, bool overlay)
		{
			// Parse until end of stream, line-by-line
			// Split string based on equal sign, and do parsing
			// based on keys
			using namespace std;
			using namespace rtmath::config;
			// _keys are mostly useless. Just used for loading.
			std::map<std::string, std::string> _keys;
			if (!overlay)
			{
				_parsedData.clear();
				_scaPlanes.clear();
				_diels.clear();
			}

			size_t line = 1;
			{
				// First line in file provides version information
				std::string vertag;
				std::getline(stream,vertag);
				
				_version = ddVersions::getVerId(vertag);
			}
			size_t nScaPlane = 0;
			string comment;
			while (stream.good())
			{
				string lin;
				std::getline(stream,lin);
				line++;
				// Check if this is a comment line (ends with ')
				// Need extra logic if line ends in whitespace
				{
					bool skip = false;
					for (auto it = lin.rbegin(); it != lin.rend(); ++it)
					{
						if (*it == ' ') continue;
						if (*it == '\'') 
						{
							// End line parsing
							skip = true; 
							break; 
						}
						// If we make it here, the line is valid for 
						// key-value pair parsing
						skip = false;
						break;
					} // Awkward. TODO: redo.
					if (skip) continue;
				}

				// Split lin based on '='
				// Prepare tokenizer
				typedef boost::tokenizer<boost::char_separator<char> >
					tokenizer;
				boost::char_separator<char> sep("=");
				tokenizer tcom(lin,sep);
				vector<string> vals;
				for (auto it=tcom.begin(); it != tcom.end(); ++it)
					vals.push_back(*it);
				if (vals.size() < 2) 
				{
					continue;
					//ostringstream errmsg;
					//errmsg << "This is not a valid ddscat.par file (error on file line " << line << ").";
					//throw rtmath::debug::xUnknownFileFormat(errmsg.str().c_str());
				}

				// Populate map
				//_keys[vals[1]] = vals[0];
				using namespace rtmath::ddscat::ddParParsers;
				{
					boost::shared_ptr<ddParLine> ptr = mapKeys(vals[1]);
					// Strip trailing whitespace at the end of vals[0].
					// It confuses some of the parsing functions, 
					// like ddParSimple<string>
					std::string vz = boost::algorithm::trim_right_copy(vals[0]);
					ptr->read(vz);
					// Individual dielectric files go into a separate structure
					// Also, if the dielectric files can be found, calculate their hashes
					if (ptr->id() == ddParParsers::IREFR)
					{
						auto p = boost::dynamic_pointer_cast<ddParParsers::ddParLineSimple<std::string> >(ptr);
						_diels.push_back(p);
						std::string dval;
						p->get(dval);
						using namespace boost::filesystem;
						path ppar = path(_filename).remove_filename();
						path pval(dval);
						path prel = boost::filesystem::absolute(pval, ppar);

						if (boost::filesystem::exists(prel))
						{
							_dielHashes.push_back(Ryan_Debug::hash::HASHfile(prel.string()));
						} else _dielHashes.push_back(Ryan_Debug::hash::HASH_t());

						//_dielHashes.push_back(HASHfile(dval));
					}
					// Everything but diels and scattering plane go here
					else if (ptr->id() < ddParParsers::PLANE1)
					{
						if (_parsedData.count(ptr->id()))
						{
							if (overlay)
							{
								_parsedData.erase(ptr->id());
							} else {
								ostringstream ostr;
								ostr << "Duplicate ddscat.par key: ";
								ostr << vals[1];
								RDthrow(Ryan_Debug::error::xBadInput())
									<< Ryan_Debug::error::otherErrorText(ostr.str());
							}
						}
						_parsedData[ptr->id()] = ptr;
					} else if (ptr->id() == ddParParsers::PLANE1)
					{
						// Scattering plane info
						nScaPlane++;
						_scaPlanes[nScaPlane] = 
							boost::dynamic_pointer_cast<ddParParsers::ddParLineSimplePlural<double> >(ptr);
					} else {
						// Unknown key
						ostringstream ostr;
						ostr << "Unknown ddscat.par key: ";
						ostr << vals[1];
						RDthrow(Ryan_Debug::error::xBadInput())
							<< Ryan_Debug::error::otherErrorText(ostr.str());
					}
				}

			}
		}

		boost::shared_ptr<const ddPar> ddPar::defaultInstance()
		{
			using namespace std;
			using namespace boost::filesystem;
			static boost::shared_ptr<ddPar> s_inst(new ddPar);
			static bool loaded = false;
			if (!loaded)
			{
				initPaths();
				if (pDefaultPar.string().size() && boost::filesystem::exists(path(pDefaultPar)))
				{
					s_inst = ddPar::generate(pDefaultPar.string(), false); 
					// = boost::shared_ptr<ddPar>(new ddPar(pDefaultPar.string(), false));
				} else {
					// Attempt to load the internal instance
					try {
						std::istringstream in(ddparDefaultInternal);
						//s_inst = new ddPar;
						readDDSCAT(s_inst, in, false);
						//Ryan_Serialization::readString(s_inst, ddparDefaultInternal, "rtmath::ddscat::ddPar");

					} catch (std::exception&)
					{
						// Cannot get default instance.....
						if (pDefaultPar.string().size())
						{
							RDthrow(Ryan_Debug::error::xMissingFile())
								<< Ryan_Debug::error::file_name(pDefaultPar.string());
						} else {
							RDthrow(Ryan_Debug::error::xOtherError())
								<< Ryan_Debug::error::otherErrorText("Cannot get default instance. Reason unknown.");
						}
					}
				}

				//rtmath::debug::instances::registerInstance( "ddPar::defaultInstance", reinterpret_cast<void*>(s_inst));
				loaded = true;
			}
			return s_inst;
		}

		void ddPar::add_options(
			boost::program_options::options_description &cmdline,
			boost::program_options::options_description &config,
			boost::program_options::options_description &hidden)
		{
			namespace po = boost::program_options;
			using std::string;

			// hash-shape-dir and hash-stats-dir can be found in rtmath.conf. 
			// So, using another config file is useless.
			cmdline.add_options()
				("default-ddpar", po::value<string>(), "Override the default ddscat.par file") // static option
				;

			config.add_options()
				;

			hidden.add_options()
				;
		}

		void ddPar::process_static_options(
			boost::program_options::variables_map &vm)
		{
			namespace po = boost::program_options;
			using std::string;
			using boost::filesystem::path;

			initPaths();
			if (vm.count("default-ddpar")) pDefaultPar = path(vm["default-ddpar"].as<string>());

			// Validate paths
			auto validateFile = [&](path p) -> bool
			{
				while (is_symlink(p))
					p = boost::filesystem::absolute(read_symlink(p), p.parent_path());
				if (!boost::filesystem::exists(p)) return false;
				if (is_directory(p)) return false;
				return true;
			};
			//if (!validateFile(pDefaultPar)) RDthrow Ryan_Debug::error::xMissingFile(pDefaultPar.string().c_str());
		}

	} // end namespace ddscat
} // end rtmath


