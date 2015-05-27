#include "Stdafx-core.h"
#include <complex>
#include <algorithm>
#include <cmath>
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <Ryan_Debug/debug.h>
#include "../rtmath/error/debug.h"
#include "../rtmath/phaseFunc.h"
#include "../rtmath/units.h"

namespace Ryan_Debug {
	namespace registry {
		template struct IO_class_registry_writer
			<::rtmath::phaseFuncs::pfRunSetContainer>;

		template struct IO_class_registry_reader
			<::rtmath::phaseFuncs::pfRunSetContainer>;

		template class usesDLLregistry<
			::rtmath::phaseFuncs::pfRunSetContainer_IO_output_registry,
			IO_class_registry_writer<::rtmath::phaseFuncs::pfRunSetContainer> >;

		template class usesDLLregistry<
			::rtmath::phaseFuncs::pfRunSetContainer_IO_input_registry,
			IO_class_registry_reader<::rtmath::phaseFuncs::pfRunSetContainer> >;

	}
}
namespace rtmath {
	
	namespace phaseFuncs
	{

		const int pfRunSetContainer::max_phaseFunc_version = 1;
		pfRunSetContainer::~pfRunSetContainer() {}
		pfRunSetContainer::pfRunSetContainer() {
			_init();
		}
		void pfRunSetContainer::_init()
		{
			ingest_hostname = Ryan_Debug::getHostname();
			ingest_username = Ryan_Debug::getUsername();
			using namespace boost::posix_time;
			using namespace boost::gregorian;
			ptime now = second_clock::local_time();
			ingest_timestamp = to_iso_string(now);
			ingest_rtmath_version = rtmath::debug::rev();
		}
		/*
		bool pfRunSetContainer::needsUpgrade() const
		{
			// Standard case
			//if (this->ingest_rtmath_version < 1636 
			//	&& this->ingest_rtmath_version > 0) return true;
			if (prohibitStats) return false;
			if (forceRecalcStats) return true;
			if (this->_currVersion >= 0 && this->_currVersion < _maxVersion) return true;
			if (_currVersion < 0 && this->ingest_rtmath_version < 1636) return true;
			return false;
		}

		void pfRunSetContainer::upgrade()
		{
			if (!needsUpgrade()) return;
			std::cerr << " upgrading nondda run from version " << this->phaseFunc_version << " to " << max_phaseFunc_version << std::endl;
			load();

			bool baseRecalced = false;
			bool rotsRecalced = false;

			auto recalcBase = [&]() {
				if (baseRecalced) return;
				// Recalculate base stats
				calcStatsBase();
				baseRecalced = true;
			};
			auto recalcRots = [&]() {
				if (rotsRecalced) return;
				// Redo each rotation
				std::set<rotData> oldRotations = rotstats;
				rotstats.clear();
				for (auto rot : oldRotations)
				{
					const basicTable &tbl = rot.get<0>();
					calcStatsRot(tbl[rotColDefs::BETA], tbl[rotColDefs::THETA], tbl[rotColDefs::PHI]);
				}
				rotsRecalced = true;
			};

			// Some logic to keep from having to recalculate everything between version upgrades
			if (this->_currVersion < 4)
			{
				recalcBase();
				recalcRots();
				this->_currVersion = _maxVersion;
			}
			// Can just follow the upgrade chain
			if (this->_currVersion == 4)
			{
				// 4-5 added more volumetric solvers and 2d projections
				calcSrms_sphere(); // These will pick up on recalculating rot stats automatically if needed
				calcSgyration();
				calcSsolid();
				calcVoroCvx(); // To get internal voronoi

				this->_currVersion++;
			}
			// Future upgrades can go here.

			this->_currVersion = _maxVersion;
		}
		*/

		pf_class_registry::~pf_class_registry() {}
		pf_provider::~pf_provider() {}

		pf_class_registry::setup::setup()
			: beta(0), theta(0), phi(0),
			sTheta(0), sTheta0(0), sPhi(0), sPhi0(0),
			wavelength(0)
		{}

		pf_class_registry::inputParamsPartial::inputParamsPartial()
			: aeff(0), aeff_version(aeff_version_type::EQUIV_V_SPHERE),
			m(1.33, 0), shape(shape_type::SPHEROID), eps(1.00001)
		{}

		pf_class_registry::cross_sections::cross_sections() :
			Qbk(-1), Qext(-1), Qsca(-1), Qabs(-1), g(-1), valid(false) {}

		void pf_provider::findHandler(
			pf_class_registry::orientation_type oriType, 
			const char* name, const pf_class_registry *res)
		{
			res = nullptr;
			auto container = getHooks();
			for (const auto &c : *container)
			{
				if (oriType != c.orientations) continue;
				if (name)
					if (std::string(name) != std::string(c.name)) continue;
				res = &c;
			}
		}

		pf_provider::pf_provider(pf_class_registry::orientation_type o, 
			const pf_class_registry::inputParamsPartial& i)
			: iparams(i), otype(o) {}

		void pf_provider::getCrossSections(const pf_class_registry::setup& s,
			resCtype& res) const
		{
			res.clear();
			auto container = getHooks();
			for (const auto &c : *container)
			{
				if (otype != c.orientations) continue;
				if (!c.fCrossSections) continue;
				try {
					pf_class_registry::cross_sections cs;
					c.fCrossSections(s, iparams, cs);
					res.push_back(std::pair<const char*, pf_class_registry::cross_sections>
						(c.name, std::move(cs)));
				}
				catch (...) { std::cerr << "Error in getting cross-sections!" << std::endl; }
			}
		}

		void pf_provider::getPfs(const pf_class_registry::setup& s,
			resPtype& res) const
		{
			res.clear();
			auto container = getHooks();
			for (const auto &c : *container)
			{
				if (otype != c.orientations) continue;
				if (!c.fPfs) continue;
				pf_class_registry::pfs cs;
				c.fPfs(s, iparams, cs);
				res.push_back(std::pair<const char*, pf_class_registry::pfs>
					(c.name, std::move(cs)));
			}
		}

		void convertFtoS(const Eigen::Matrix2cd &f, Eigen::Matrix2cd& Sn, double phi, 
			std::complex<double> a, std::complex<double> b, std::complex<double> c, std::complex<double> d)
		{
			using namespace std;
			typedef complex<double> CD;
			complex<double> i(0,1);

			double rphi = phi * boost::math::constants::pi<double>() / 180.0;
			double cp = cos(rphi);
			double sp = sin(rphi);
			complex<double> CP(cp,0), SP(sp,0);
			Sn(0,0) = -i*((f(0,0)*((a*CP)+(b*SP)))
				+(f(0,1)*((c*CP)+(d*SP))));
			Sn(0,1) = i*((f(0,0)*((b*CP)-(a*SP)))
				+(f(0,1)*((d*CP)-(c*SP))));
			Sn(1,0) = i*((f(1,0)*((a*CP)+(b*SP)))
				+(f(1,1)*((c*CP)+(d*SP))));
			Sn(1,1) = -i*((f(1,0)*((b*CP)-(a*SP)))
				+(f(1,1)*((d*CP)-(c*SP))));
		}

		void selectMueller(const std::string &id,
			std::function<void(const Eigen::Matrix2cd&, Eigen::Matrix4d&)>& f)
		{
			using namespace std;
			string tid = id;
			// Put in lower case
			std::transform(tid.begin(),tid.end(),tid.begin(),::tolower);

			if (tid == "bh" || tid == "ddscat")
				f = muellerBH;
			else if (tid == "tmatrix")
				f = muellerTMATRIX;
			else
				RDthrow(::Ryan_Debug::error::xBadInput())
				<< ::Ryan_Debug::error::key(id); /// \todo use another xError? Create a new one?
		}

		void muellerBH(const Eigen::Matrix2cd& Sn, Eigen::Matrix4d& Snn)
		{
			using std::complex;
			complex<double> scratch;
			// Sn(2) = Sn(0,0)
			// Sn(1) = Sn(1,1)
			// Sn(3) = Sn(0,1)
			// Sn(4) = Sn(1,0)
			const complex<double> &S2 = Sn(0,0), &S1 = Sn(1,1), &S3 = Sn(0,1), &S4 = Sn(1,0);

			Snn(0,0) = 0.5 * ( norm(S1) + norm(S2) + norm(S3) + norm(S4) );
			Snn(0,1) = 0.5 * ( norm(S2) - norm(S1) + norm(S4) - norm(S3) );
			Snn(0,2) = ( (S2*conj(S3)) + (S1*conj(S4)) ).real();
			Snn(0,3) = ( (S2*conj(S3)) - (S1*conj(S4)) ).imag();

			Snn(1,0) = 0.5 * ( norm(S2) - norm(S1) + norm(S3) - norm(S4) );
			Snn(1,1) = 0.5 * ( norm(S1) + norm(S2) - norm(S3) - norm(S4) );
			Snn(1,2) = ( (S2*conj(S3)) - (S1*conj(S4)) ).real();
			Snn(1,3) = ( (S2*conj(S3)) + (S1*conj(S4)) ).imag();

			Snn(2,0) = ( (S2*conj(S4)) + (S1*conj(S3)) ).real();
			Snn(2,1) = ( (S2*conj(S4)) - (S1*conj(S3)) ).real();
			Snn(2,2) = ( (S1*conj(S2)) + (S3*conj(S4)) ).real();
			Snn(2,3) = ( (S2*conj(S1)) + (S4*conj(S3)) ).imag();

			Snn(3,0) = ( (S4*conj(S2)) + (S1*conj(S3)) ).imag();
			Snn(3,1) = ( (S4*conj(S2)) - (S1*conj(S3)) ).imag();
			Snn(3,2) = ( (S1*conj(S2)) - (S3*conj(S4)) ).imag();
			Snn(3,3) = ( (S1*conj(S2)) - (S3*conj(S4)) ).real();

		}

		void muellerTMATRIX(const Eigen::Matrix2cd& Sn, Eigen::Matrix4d& Snn)
		{
			std::complex<double> scratch;

			Snn(0,0) = 0.5 * ( (Sn(0,0)*conj(Sn(0,0))) + (Sn(0,1)*conj(Sn(0,1))) 
				+ (Sn(1,0)*conj(Sn(1,0))) + (Sn(1,1)*conj(Sn(1,1))) ).real();

			Snn(0,1) = 0.5 * ( (Sn(0,0)*conj(Sn(0,0))) - (Sn(0,1)*conj(Sn(0,1))) 
				+ (Sn(1,0)*conj(Sn(1,0))) - (Sn(1,1)*conj(Sn(1,1)))).real();

			scratch = ( (Sn(1,1) * (conj(Sn(1,0)))) + (Sn(0,0) * (conj(Sn(0,1))) ));
			Snn(0,2) = -1.0 * scratch.real();

			scratch = ( (Sn(0,0) * (conj(Sn(0,1)))) - (Sn(1,1) * (conj(Sn(1,0))) ));
			Snn(0,3) = 1.0 * scratch.imag();

			Snn(1,0) = 0.5 * ( (Sn(0,0)*conj(Sn(0,0))) + (Sn(0,1)*conj(Sn(0,1))) 
				- (Sn(1,0)*conj(Sn(1,0))) - (Sn(1,1)*conj(Sn(1,1))) ).real();

			Snn(1,1) = 0.5 * ( (Sn(0,0)*conj(Sn(0,0))) - (Sn(0,1)*conj(Sn(0,1)))
				- (Sn(1,0)*conj(Sn(1,0))) + (Sn(1,1)*conj(Sn(1,1))) ).real();

			scratch = ( (Sn(1,1) * (conj(Sn(1,0)))) - (Sn(0,0) * (conj(Sn(0,1))) ));
			Snn(1,2) = 1.0 * scratch.real();
			scratch = ( (Sn(0,0) * (conj(Sn(0,1)))) + (Sn(1,1) * (conj(Sn(1,0))) ));
			Snn(1,3) = 1.0 * scratch.imag();

			scratch = ( (Sn(0,0) * (conj(Sn(1,0)))) + (Sn(1,1) * (conj(Sn(0,1))) ));
			Snn(2,0) = -1.0 * scratch.real();
			scratch = ( -(Sn(0,0) * (conj(Sn(1,0)))) + (Sn(1,1) * (conj(Sn(0,1))) ));
			Snn(2,1) = 1.0 * scratch.real();

			scratch = ( (Sn(0,0) * (conj(Sn(1,1)))) + (Sn(0,1) * (conj(Sn(1,0))) ));
			Snn(2,2) = scratch.real();
			scratch = ( (Sn(0,0) * (conj(Sn(1,1)))) + (Sn(1,0) * (conj(Sn(0,1))) ));
			Snn(2,3) = -1.0 * scratch.imag();

			scratch = ( (conj(Sn(0,0)) * (Sn(1,0))) + (conj(Sn(0,1)) * (Sn(1,1)) ));
			Snn(3,0) = 1.0 * scratch.imag();
			scratch = ( (conj(Sn(0,0)) * (Sn(1,0))) - (conj(Sn(0,1)) * (Sn(1,1)) ));
			Snn(3,1) = 1.0 * scratch.imag();

			scratch = ( (Sn(1,1) * (conj(Sn(0,0)))) - (Sn(0,1) * (conj(Sn(1,0))) ));
			Snn(3,2) = scratch.imag();
			scratch = ( (Sn(1,1) * (conj(Sn(0,0)))) - (Sn(0,1) * (conj(Sn(1,0))) ));
			Snn(3,3) = scratch.real();
		}

		/// \todo Test rtmath::phasefuncs::invertS
		#pragma message("phaseFunc.cpp: Test invertS")
		void invertS(const Eigen::Matrix4d &Snn, const Eigen::Matrix4d &Knn, double fGHz, Eigen::Matrix2cd& Sn)
		{
			// TODO: make sure it's correct via testing. Am I solving for the correct matrix?
			const double PI = boost::math::constants::pi<double>();

			// Using the Mueller matrix (Snn) and the Stokes Extinction Matrix (Knn), the 
			// forward-scattering amplitude matrix (Sn) may be retreived. This goes in the 
			// reverse direction from the normal procedure.
			// It cannot be done purely from the extinction matrix (S0 and S3 real values are coupled)
			// and it is _quite_ dificult from the phase matrix alone.
			units::conv_spec fc("GHz","Hz");
			const double f = fc.convert(fGHz);

			double val;

			// Imaginary parts

			val = (Knn(0,0) + Knn(0,1)) * f / (4.0 * PI);
			Sn(0,0).imag(val);

			val = Knn(0,0) * f / (2.0 * PI);
			val -= Sn(0,0).imag();
			Sn(1,1).imag(val);

			val = -f * (Knn(0,2) + Knn(1,2)) / (4.0 * PI);
			Sn(0,1).imag(val);

			val = -f * Knn(0,2) / (2.0 * PI);
			val -= Sn(0,1).imag();
			Sn(1,0).imag(val);

			// Real parts

			val = -f * (Knn(0,3) + Knn(1,3)) / (4.0 * PI);
			Sn(0,1).real(val);

			val = -f * Knn(1,3) / (2.0 * PI);
			val -= Sn(0,1).real();
			Sn(1,0).real(val);

			// Need Snn for the last two real components

			val = Snn(0,0) + Snn(1,0);
			val -= (Sn(0,1) * conj(Sn(0,1))).real();
			val -= Sn(0,0).imag() * Sn(0,0).imag();
			Sn(0,0).real(val);

			val = Snn(0,0) - Snn(1,0);
			val -= (Sn(1,0) * conj(Sn(1,0))).real();
			val -= Sn(1,1).imag() * Sn(1,1).imag();
			Sn(1,1).real(val);
		}

#pragma message("phaseFunc.cpp: Test genExtinctionMatrix")
		/// \todo Test rtmath::phaseFuncs::genExtinctionMatrix
		void genExtinctionMatrix(Eigen::Matrix4d &Knn, const Eigen::Matrix2cd &Sn, double fGHz)
		{
			// TODO: make sure it's correct via testing. Am I solving for the correct matrix?


			// Convert f in GHz to Hz
			units::conv_spec cnv("GHz","Hz");
			double f = cnv.convert(fGHz);

			// Do the diagonals first
			for (size_t i=0;i<4;i++)
			{
				Knn(i,i) = (Sn(0,0) + Sn(1,1)).imag();
			}

			// And the remaining 12 (really duplicates, so only six)
			for (size_t i=0; i<4; i++)
				for (size_t j=0; j<4; j++)
				{
					if (i == 0 && j == 1) Knn(i,j) = (Sn(0,0) - Sn(1,1)).imag();
					if (i == 0 && j == 2) Knn(i,j) = -(Sn(0,1) + Sn(1,0)).imag();
					if (i == 0 && j == 3) Knn(i,j) = (-Sn(0,1) + Sn(1,0)).real();
					if (i == 1 && j == 2) Knn(i,j) = (Sn(1,0) - Sn(0,1)).imag();
					if (i == 1 && j == 3) Knn(i,j) = -(-Sn(0,1) + Sn(1,0)).real();
					if (i == 2 && j == 3) Knn(i,j) = (Sn(1,1) - Sn(0,0)).real();

					if (j == 0 && i == 1) Knn(i,j) = (Sn(0,0) - Sn(1,1)).imag(); // Same
					if (j == 0 && i == 2) Knn(i,j) = -(Sn(0,1) + Sn(1,0)).imag();
					if (j == 0 && i == 3) Knn(i,j) = (-Sn(0,1) + Sn(1,0)).real();
					if (j == 1 && i == 2) Knn(i,j) = -(Sn(1,0) - Sn(0,1)).imag(); // Negative
					if (j == 1 && i == 3) Knn(i,j) = (-Sn(0,1) + Sn(1,0)).real();
					if (j == 2 && i == 3) Knn(i,j) = -(Sn(1,1) - Sn(0,0)).real();
				}

				// Go back and multiply by f
				for (size_t i=0; i<4;i++)
					for (size_t j=0; j<4; j++)
						Knn(i,j) *= f;
		}

	}


}

