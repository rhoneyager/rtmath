#include "Stdafx-ddscat.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <boost/filesystem.hpp>
#include <complex>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/iostreams/filter/newline.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>

#include "../rtmath/ddscat/ddOutput.h"
#include "../rtmath/ddscat/ddOutputSingle.h"
#include "../rtmath/ddscat/ddOutputSingleKeys.h"
#include "../rtmath/ddscat/ddScattMatrix.h"
#include "../rtmath/ddscat/ddVersions.h"
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/refract.h"
#include "../rtmath/units.h"
#include "../rtmath/macros.h"
#include "../rtmath/quadrature.h"
#include "../rtmath/error/error.h"



namespace rtmath {

	namespace ddscat {
		using namespace rtmath::ddscat::ddOutputSingleKeys;

#pragma message("Warning: ddOutputSingle needs correct normalization and default values")
		boost::shared_ptr<ddOutputSingle> ddOutputSingle::normalize() const
		{
			// Following the Mishchenko normalization condition, take and compute the Mueller matrix entries 
			// and normalize them. Note: this directly evaluates an integral using a summation method since DDSCAT 
			// does not provide good default angles for quadrature.

			// pf normalization condition is that 
			// 1 = 1/{4\pi} \int_0^\pi d\Theta \sin \Theta P_11(\Theta,\beta,\phi) \int_0^2\pi d\Phi
			double intres = 1.0 / (4.0 * boost::math::constants::pi<double>());

			// This summation is easier than expected because the values vary sinusoidally. In the case of 
			// phi = 0 and 90 degrees, I can find the mean value just by taking te arithmetic mean.

			// Take each pf and multiply by the normalization constraint
			boost::shared_ptr<ddOutputSingle> res(new ddOutputSingle);
			throw debug::xUnimplementedFunction();
			return res;
		}


		/*
		void ddOutputSingle::writeFile(const std::string &filename, const std::string &type) const
		{
		using namespace Ryan_Serialization;
		std::string cmeth, uncompressed;

		/// \todo Add a table to determine which file types are automatically compressed on saving.
		/// For those types, automatically apply compression.
		#pragma message("TODO: add a table to determine which file types are automatically compressed on saving")

		uncompressed_name(filename, uncompressed, cmeth);
		boost::filesystem::path p(uncompressed);
		boost::filesystem::path pext = p.extension(); // Uncompressed extension

		std::string utype = type;
		if (!utype.size()) utype = pext.string();

		// Serialization gets its own override
		if (Ryan_Serialization::known_format(utype))
		{
		Ryan_Serialization::write<ddOutputSingle>(*this, filename, "rtmath::ddscat::ddOutputSingle");
		return;
		}

		std::ofstream out(filename.c_str(), std::ios_base::out | std::ios_base::binary);
		using namespace boost::iostreams;
		filtering_ostream sout;
		if (cmeth.size())
		prep_compression(cmeth, sout);

		sout.push(boost::iostreams::newline_filter(boost::iostreams::newline::posix));
		sout.push(out);

		if (utype == ".sca")
		{
		writeSCA(sout);
		} else if (utype == ".fml")
		{
		writeFML(sout);
		} else if (utype == ".avg")
		{
		writeAVG(sout);
		} else {
		throw rtmath::debug::xUnknownFileFormat(filename.c_str());
		}
		}

		void ddOutputSingle::readFile(const std::string &filename, const std::string &type)
		{
		// First, detect if the file is compressed.
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
		Ryan_Serialization::read<ddOutputSingle>(*this, filename, "rtmath::ddscat::ddOutputSingle");
		return;
		}

		std::ifstream in(filename.c_str(), std::ios_base::binary | std::ios_base::in);
		// Consutuct an filtering_iostream that matches the type of compression used.
		using namespace boost::iostreams;
		filtering_istream sin;
		if (cmeth.size())
		prep_decompression(cmeth, sin);
		sin.push(boost::iostreams::newline_filter(boost::iostreams::newline::posix));
		sin.push(in);

		if (type.size()) pext = boost::filesystem::path(type); // pext is first set a few lines above
		if (pext.string() == ".sca")
		{
		readSCA(sin);
		}
		else if (pext.string() == ".fml")
		{
		readFML(sin);
		}
		else if (pext.string() == ".avg")
		{
		readAVG(sin);
		}
		else if (p.filename().string().find("avg") != std::string::npos)
		{
		readAVG(sin);
		}
		else {
		throw rtmath::debug::xUnknownFileFormat(filename.c_str());
		}
		}

		*/



		void ddOutputSingle::doExportOri(boost::shared_ptr<ddOutput> p, size_t index, bool isavg)
		{
			auto o = p->oridata->block<1, ddOutput::oriColDefs::NUM_ORICOLDEFS>(index, 0);
			// If avg file, the previous block index is always zero.
			if (isavg)
				o = p->avgoridata->block<1, ddOutput::oriColDefs::NUM_ORICOLDEFS>(index, 0);
			o(ddOutput::oriColDefs::FREQ) = static_cast<float>(freq());
			o(ddOutput::oriColDefs::AEFF) = static_cast<float>(aeff());
			o(ddOutput::oriColDefs::BETA) = static_cast<float>(beta());
			o(ddOutput::oriColDefs::THETA) = static_cast<float>(theta());
			o(ddOutput::oriColDefs::PHI) = static_cast<float>(phi());

			auto cn = getConnector();
			o(ddOutput::oriColDefs::E01XR) = static_cast<float>(cn->e01x.real());
			o(ddOutput::oriColDefs::E01XI) = static_cast<float>(cn->e01x.imag());
			o(ddOutput::oriColDefs::E01YR) = static_cast<float>(cn->e01y.real());
			o(ddOutput::oriColDefs::E01YI) = static_cast<float>(cn->e01y.imag());
			o(ddOutput::oriColDefs::E01ZR) = static_cast<float>(cn->e01z.real());
			o(ddOutput::oriColDefs::E01ZI) = static_cast<float>(cn->e01z.imag());

			o(ddOutput::oriColDefs::E02XR) = static_cast<float>(cn->e02x.real());
			o(ddOutput::oriColDefs::E02XI) = static_cast<float>(cn->e02x.imag());
			o(ddOutput::oriColDefs::E02YR) = static_cast<float>(cn->e02y.real());
			o(ddOutput::oriColDefs::E02YI) = static_cast<float>(cn->e02y.imag());
			o(ddOutput::oriColDefs::E02ZR) = static_cast<float>(cn->e02z.real());
			o(ddOutput::oriColDefs::E02ZI) = static_cast<float>(cn->e02z.imag());

			//Copy from _statTable::QEXT1 through QSCAG3M, inclusively;
			auto s = p->oridata->block<1, 
				ddOutput::oriColDefs::NUM_ORICOLDEFS - ddOutput::oriColDefs::QEXT1>
				(index, ddOutput::oriColDefs::QEXT1);
			for (size_t i = 0; i < ddscat::NUM_STAT_ENTRIES - ddscat::QEXT1; ++i)
				o(ddOutput::oriColDefs::QEXT1 + i) = static_cast<float>(_statTable.at(ddscat::QEXT1 + i));

		}

		void ddOutputSingle::doExportFMLs(boost::shared_ptr<ddOutput> p, size_t startIndex, size_t oriIndex)
		{
			auto o = p->fmldata->block(startIndex, 0, numF(), ddOutput::fmlColDefs::NUM_FMLCOLDEFS);
			size_t i = 0;
			for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
			{
				if ((*it)->id() != scattMatrixType::F) continue;
				auto s = boost::dynamic_pointer_cast<const ddscat::ddScattMatrixF>(*it);

				auto f = s->getF();
				
				o(i, ddOutput::fmlColDefs::ORIINDEX) = static_cast<float>(oriIndex);
				o(i, ddOutput::fmlColDefs::THETAB) = static_cast<float>((*it)->theta());
				o(i, ddOutput::fmlColDefs::PHIB) = static_cast<float>((*it)->phi());
				o(i, ddOutput::fmlColDefs::F00R) = static_cast<float>(f(0, 0).real());
				o(i, ddOutput::fmlColDefs::F00I) = static_cast<float>(f(0, 0).imag());
				o(i, ddOutput::fmlColDefs::F01R) = static_cast<float>(f(0, 1).real());
				o(i, ddOutput::fmlColDefs::F01I) = static_cast<float>(f(0, 1).imag());
				o(i, ddOutput::fmlColDefs::F10R) = static_cast<float>(f(1, 0).real());
				o(i, ddOutput::fmlColDefs::F10I) = static_cast<float>(f(1, 0).imag());
				o(i, ddOutput::fmlColDefs::F11R) = static_cast<float>(f(1, 1).real());
				o(i, ddOutput::fmlColDefs::F11I) = static_cast<float>(f(1, 1).imag());

				i++;
			}
		}

		/*
		void ddOutputSingle::doExportSCAs(boost::shared_ptr<ddOutput> p, size_t startIndex, size_t oriIndex)
		{
			auto o = p->scadata->block(startIndex, 0, numP(), ddOutput::fmlColDefs::NUM_FMLCOLDEFS);
			size_t i = 0;
			for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
			{
				if ((*it)->id() != scattMatrixType::P) continue;
				auto s = boost::dynamic_pointer_cast<const ddscat::ddScattMatrixP>(*it);

				auto P = s->getP();

				o(i, ddOutput::ORIINDEX) = static_cast<float>(oriIndex);
				o(i, ddOutput::THETAB) = static_cast<float>((*it)->theta());
				o(i, ddOutput::PHIB) = static_cast<float>((*it)->phi());
				o(i, ddOutput::PHIB) = static_cast<float>((*it)->pol());
				o(i, ddOutput::PHIB) = static_cast<float>((*it)->polLin());
				o(i, ddOutput::F00R) = static_cast<float>(P(0, 0));
				o(i, ddOutput::F00I) = static_cast<float>(P(0, 1));
				o(i, ddOutput::F01R) = static_cast<float>(P(0, 2));
				o(i, ddOutput::F01I) = static_cast<float>(P(0, 3));
				o(i, ddOutput::F10R) = static_cast<float>(P(1, 0));
				o(i, ddOutput::F10I) = static_cast<float>(P(1, 1));
				o(i, ddOutput::F11R) = static_cast<float>(P(1, 2));
				o(i, ddOutput::F11I) = static_cast<float>(P(1, 3));

				i++;
			}
		}
		*/

		void ddOutputSingle::version(size_t nv)
		{
			auto obj = ddOutputSingleObj::constructObj("version");
			auto objc = boost::dynamic_pointer_cast<ddver>(obj);
			objc->version(nv);
			_objMap["version"] = obj;
			//_version = nv;
			_statTable_Size_ts.at(stat_entries_size_ts::VERSION) = nv;
		}

		void ddOutputSingle::getTARGET(std::string &target) const
		{
			target = _objMap.at("target")->value();
		}

		void ddOutputSingle::setTARGET(const std::string &target)
		{
			auto obj = ddOutputSingleObj::constructObj("target");
			auto objc = boost::dynamic_pointer_cast<ddtarget>(obj);
			objc->setTarget(target);
			_objMap["target"] = obj;
		}

		void ddOutputSingle::readHeader(std::istream &in, const std::string &sstop)
		{
			using namespace std;
			std::string lin;
			bool headerDone = false;
			size_t line = 0;
			while (!headerDone)
			{
				std::getline(in, lin);
				line++;
				if (lin.find(sstop) != string::npos)
				{
					headerDone = true;
					//cerr << "Header done on line " << line << endl;
					break;
				}
				string key;
				ddOutputSingleObj::findMap(lin, key);
				if (key == "")
				{
					if (lin == "" || lin == "\r") continue;
					cerr << "Unknown line: " << lin << endl;
					continue;
				}
				auto obj = ddOutputSingleObj::constructObj(key);
				istringstream ii(lin);
				obj->read(ii);
				_objMap[key] = obj;
				if (key == "beta")
					_statTable.at(stat_entries::BETA) = boost::lexical_cast<double>(obj->value());
				if (key == "theta")
					_statTable.at(stat_entries::THETA) = boost::lexical_cast<double>(obj->value());
				if (key == "phi")
					_statTable.at(stat_entries::PHI) = boost::lexical_cast<double>(obj->value());
				if (key == "wave") _statTable.at(stat_entries::WAVE) = boost::lexical_cast<double>(obj->value());
				if (key == "aeff") _statTable.at(stat_entries::AEFF) = boost::lexical_cast<double>(obj->value());
				if (key == "version") _statTable_Size_ts.at(stat_entries_size_ts::VERSION) = boost::lexical_cast<size_t>(obj->value());
				if (key == "d") _statTable.at(stat_entries::DIPOLESPACING) = boost::lexical_cast<double>(obj->value());
				if (key == "numdipoles") _statTable_Size_ts.at(stat_entries_size_ts::NUM_DIPOLES) = boost::lexical_cast<size_t>(obj->value());
			}

			_statTable.at(stat_entries::FREQ) = rtmath::units::conv_spec("um", "GHz").convert(_statTable.at(stat_entries::WAVE));
		}

		boost::shared_ptr<const ddScattMatrixConnector> ddOutputSingle::getConnector() const
		{
			auto obj1 = boost::dynamic_pointer_cast<ddPolVec>(_objMap.at("incpol1lf"));
			auto obj2 = boost::dynamic_pointer_cast<ddPolVec>(_objMap.at("incpol2lf"));
			std::vector<std::complex<double> > vs(6);
			vs[0] = obj1->getPol(0);
			vs[1] = obj1->getPol(1);
			vs[2] = obj1->getPol(2);
			vs[3] = obj2->getPol(0);
			vs[4] = obj2->getPol(1);
			vs[5] = obj2->getPol(2);
			boost::shared_ptr<const ddScattMatrixConnector> cn =
				ddScattMatrixConnector::fromVector(vs);
			return cn;
		}

		void ddOutputSingle::readFML(std::istream &in)
		{
			readHeader(in, "Re(f_11)");
			// Get e1 and e2 in lab frame from the header data
			auto cn = getConnector();
			readF(in, cn);
		}

		std::complex<double> ddOutputSingle::getM() const
		{
			// if (key == "neps") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddM>(new ddM));
			auto obj = boost::dynamic_pointer_cast<ddM>(_objMap.at("neps"));
			return obj->getM();
		}

		void ddOutputSingle::writeF(std::ostream &out) const
		{
			using namespace std;
			out << " theta   phi  Re(f_11)   Im(f_11)   Re(f_21)   Im(f_21)   Re(f_12)   Im(f_12)   Re(f_22)   Im(f_22)";
			for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
			{
				if ((*it)->id() != scattMatrixType::F) continue;
				boost::shared_ptr<const ddscat::ddScattMatrixF> sf(
					boost::dynamic_pointer_cast<const ddscat::ddScattMatrixF>(*it));
				out << endl;
				out.width(6);
				// it->first crds ordering is freq, phi, theta
				out << (*it)->theta() << "\t";
				out << (*it)->phi();
				out.width(11);
				ddScattMatrix::FType f = sf->getF();

				for (size_t j = 0; j < 2; j++)
					for (size_t i = 0; i < 2; i++)
					{
						// Note the reversed coordinates. This matches ddscat.
						out << "\t" << f(i, j).real();
						out << "\t" << f(i, j).imag();
					}
			}
		}

		void ddOutputSingle::writeS(std::ostream &out) const
		{
			using namespace std;
			out << " theta   phi  Re(S_11)   Im(S_11)   Re(S_21)   Im(S_21)   Re(S_12)   Im(S_12)   Re(f_22)   Im(f_22)";
			for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
			{
				if ((*it)->id() != scattMatrixType::F) continue;
				boost::shared_ptr<const ddscat::ddScattMatrixF> sf(
					boost::dynamic_pointer_cast<const ddscat::ddScattMatrixF>(*it));
				out << endl;
				out.width(6);
				// it->first crds ordering is freq, phi, theta
				out << (*it)->theta() << "\t";
				out << (*it)->phi();
				out.width(11);
				ddScattMatrix::FType s = sf->getS();
				for (size_t j = 0; j < 2; j++)
					for (size_t i = 0; i < 2; i++)
					{
						// Note the reversed coordinates. This matches ddscat.
						out << "\t" << s(i, j).real();
						out << "\t" << s(i, j).imag();
					}
			}
		}

		void ddOutputSingle::getRots(rtmath::ddscat::rotations &rots) const
		{
			// betarange, thetarange, phirange
			auto betas = boost::dynamic_pointer_cast<ddRot1d>(_objMap.at("betarange"));
			auto thetas = boost::dynamic_pointer_cast<ddRot1d>(_objMap.at("thetarange"));
			auto phis = boost::dynamic_pointer_cast<ddRot1d>(_objMap.at("phirange"));
			rots = rtmath::ddscat::rotations(betas->min, betas->max, betas->n,
				thetas->min, thetas->max, thetas->n,
				phis->min,
				phis->max,
				phis->n);
		}

		/// \todo Extend to select a nonzero refractive index entry
		double ddOutputSingle::guessTemp() const
		{
			return rtmath::refract::guessTemp(freq(), getM());
		}

		/*
		void a()
		{
			Eigen::MatrixXd d(2, stat_entries::NUM_STAT_ENTRIES);
			//Eigen::Block<double, 1, stat_entries::NUM_STAT_ENTRIES> b(;
			//b = d.block<1, stat_entries::NUM_STAT_ENTRIES>(0,0);
		}
		*/

		/*
		void ddOutputSingle::writeEvans(std::ostream &out, double freq) const
		{
		using namespace std;
		// Takes the scattering data, generates the appropriate
		// phase, extinction matrices and emission vectors for a given
		// frequency, and writes the necessary file.

		// Generate quadrature points
		set<double> qangles;
		// Let's get the quadrature angles from gaussian quadrature
		// Other quadrature methods can be coded in as well
		const size_t deg = 7;
		quadrature::getQuadPtsLeg(deg,qangles);
		// The quadrature points are on interval (-1,1)
		// Need to get mapping between angles in degrees and these points
		// can handle this by mapping mu = cos(theta).

		// For phase functions, need to look at both incoming and outgoing angle
		// in cos(theta)
		// ddscat output always has incoming angle at zero degrees, with a varying output angle.
		// However, the targets may be rotated to simulate the angle change.
		// In this case, however, we likely just have a single ensemble pf from the data
		// TODO!!!!!
		std::map<coords::cyclic<double>, std::shared_ptr<const ddscat::ddScattMatrix> >
		interped;
		// Need to interpolate phase matrices to the correct quadrature points
		for (auto it = qangles.begin(); it != qangles.end(); it++)
		{
		// TODO: convert angle into cyclic coords
		throw rtmath::debug::xUnimplementedFunction();
		coords::cyclic<double> crd;
		std::shared_ptr<const ddscat::ddScattMatrix> interres;
		interpolate(crd, interres);
		interped[crd] = interres;
		}

		// First, write commented header information
		// This includes where the scattering information is from, and a
		// discription of each of these files
		out << "C  ddscat rtmath output for " << endl;
		out << "C  theta " << _theta << " phi " << _phi << " beta " << _beta << endl;
		out << "C  at f = " << _freq << " GHz" << endl;

		// Next is the degree and type of quadrature
		cout << "   8    0   'GAUSSIAN         '" << endl;

		// Output each scattering matrix at the designated quadrature incoming
		// and outgoing angles
		out << "C   SCATTERING MATRIX" << endl;
		for (auto it = interped.begin(); it != interped.end(); ++it)
		{
		// Write incoming angle, outcoming angle, 0
		}

		// Write the extinction matrix at each quadrature angle
		out << "C   EXTINCTION MATRIX" << endl;
		for (auto it = interped.begin(); it != interped.end(); ++it)
		{
		// Write incoming angle
		}

		// Output the emission vectors
		out << "C   EMISSION VECTOR" << endl;
		for (auto it = interped.begin(); it != interped.end(); ++it)
		{
		// Write incoming angle and the four stokes parameters
		}
		// Evans fortran files lack a newline at EOF.
		}
		*/

		/*
		void ddOutputSingle::loadFile(const std::string &filename)
		{
		using namespace std;
		// File loading routine is important!
		// Load a standard .fml file. Parse each line for certain key words.
		clear();
		bool dataseg = false;
		this->_filename = filename;

		// If a shape is not loaded, then try to load the corresponding shapefile
		using namespace boost::filesystem;
		path pshapepath, p, pfile(filename);
		p = pfile.parent_path();
		string shapepath;
		{
		// Figure out where the shape file is located.
		path ptarget = p / "target.out";
		path pshapedat = p / "shape.dat";
		if (exists(ptarget))
		{ pshapepath = ptarget;
		} else if (exists(pshapedat))
		{ pshapepath = pshapedat;
		} else {
		throw rtmath::debug::xMissingFile("shape.dat or target.out");
		}
		shapepath = pshapepath.string();
		if (exists(pshapepath) && !_shape)
		_shape = boost::shared_ptr<shapefile>(new shapefile(shapepath));
		}

		ifstream in(filename.c_str(), std::ifstream::in);
		while (in.good())
		{
		// Read a line
		string lin;
		std::getline(in,lin);
		istringstream lss(lin);
		// Expand line using convenient expansion function
		//vector<string> seg;
		//splitString(lin, ' ', seg);

		// Are we in the data segment?
		if (dataseg)
		{
		// In data segment
		// Entry not modified after this, because it is const
		std::shared_ptr<const ddScattMatrix> nscat (new ddScattMatrix(_freq,lss));
		// Save to the maps and sets
		// Totally assuming that no duplicate entry exists
		_insert(nscat);
		} else {
		// Still in header segment
		string junk;
		// Search for key strings in file

		// BETA
		if (lin.find("BETA") != string::npos)
		{
		lss >> junk; // get rid of first word
		lss >> junk;
		lss >> _beta;
		}
		// THETA
		if (lin.find("THETA") != string::npos)
		{
		lss >> junk; // get rid of first word
		// Theta is unlike Beta and Phi, as there is
		// no space between THETA and =
		lss >> _theta;
		}
		// PHI
		if (lin.find("PHI") != string::npos)
		{
		lss >> junk; // get rid of first word
		lss >> junk;
		lss >> _phi;
		}
		// NAT0
		if (lin.find("NAT0") != string::npos)
		{
		lss >> _numDipoles;
		}
		// AEFF
		if (lin.find("AEFF") != string::npos)
		{
		lss >> junk; // get rid of first word
		lss >> _reff;
		}
		// WAVE
		if (lin.find("WAVE") != string::npos)
		{
		// BAD --- WAVE runs against size...
		//lss >> junk; // get rid of first word
		//lss >> _wavelength;
		// Instead, read wave from column 7 (starting at 0) to 17
		_wavelength = atof( lin.substr( 7, 10 ).c_str() );
		// Also do a conversion from wavelength to frequency,
		// for easier comparisons later
		units::conv_spec wvtof("um","GHz");
		_freq = wvtof.convert(_wavelength);
		}
		// theta --- indicates last line of header
		if (lin.find("Re(f_11)") != string::npos)
		{
		dataseg = true;
		}
		}
		}
		}
		*/

	} // end ddscat
} // end rtmath



