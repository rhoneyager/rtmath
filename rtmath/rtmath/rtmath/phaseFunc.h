#pragma once
#include "defs.h"

#include <functional>
#include <complex>
#include <map>
#include <string>
#include <Eigen/Core>
#include "registry.h"
//#include "enums.h"
//#include "da/damatrix.h"


#pragma message("Warning: phaseFunc.h needs the pf class moved elsewhere, + interpolation")
namespace rtmath {
	// Forward declaration
	namespace phaseFuncs {
		class pfRunSetContainer;
		class pfRunSetContainer_IO_input_registry {};
		class pfRunSetContainer_IO_output_registry {};
	}
	// Registry declarations
	namespace registry {
		extern template struct IO_class_registry_writer<
			::rtmath::phaseFuncs::pfRunSetContainer>;

		extern template struct IO_class_registry_reader<
			::rtmath::phaseFuncs::pfRunSetContainer>;

		extern template class usesDLLregistry<
			::rtmath::phaseFuncs::pfRunSetContainer_IO_input_registry,
			IO_class_registry_reader<::rtmath::phaseFuncs::pfRunSetContainer> >;

		extern template class usesDLLregistry<
			::rtmath::phaseFuncs::pfRunSetContainer_IO_output_registry,
			IO_class_registry_writer<::rtmath::phaseFuncs::pfRunSetContainer> >;

	}

	/** \brief This namespace provides the different type of radiative 
	* transfer matrix manipulations.
	* 
	* This includes several Mueller matrix generation methods and 
	* the ability to generate an extinction matrix. Eventually, Mueller matrix 
	* inversion routines will also go here.
	*
	* \todo Need to move pf class elsewhere and add interpolation
	**/
	namespace phaseFuncs
	{
		/// Provides a reference to the desired phase function 
		/// routine. This is implemented to allow user choice in Mueller method.
		void DLEXPORT_rtmath_core selectMueller(const std::string &id,
			std::function<void(const Eigen::Matrix2cd&, Eigen::Matrix4d&)>&);

		// Note following conventions: matrix is [[S2, S3][S4,S1]] = [[Sn0, Sn1][Sn2, Sn3]]
		// Sn is the matrix in linear form {S1, S2, S3, S4}, so it should avoid any 
		// of the subsequent issues with forgetting the index transformations.

		void DLEXPORT_rtmath_core muellerBH(const Eigen::Matrix2cd& Sn, Eigen::Matrix4d& Snn);
		void DLEXPORT_rtmath_core muellerTMATRIX(const Eigen::Matrix2cd& Sn, Eigen::Matrix4d& Snn);

		void DLEXPORT_rtmath_core convertFtoS(const Eigen::Matrix2cd &f, Eigen::Matrix2cd& Sn, double phi, 
			std::complex<double> a, std::complex<double> b, std::complex<double> c, std::complex<double> d);

		void DLEXPORT_rtmath_core invertS(const Eigen::Matrix4d &Snn, const Eigen::Matrix4d &Knn, double fGHz, Eigen::Matrix2cd& Sn);

		void DLEXPORT_rtmath_core genExtinctionMatrix(Eigen::Matrix4d &Knn, const Eigen::Matrix2cd &Sn, double fGHz);

		/// \brief This class is used for registration of phase function and cross-section
		/// providers.
		/// \note Full stats will use a connector when passed to this code.
		struct DLEXPORT_rtmath_core pf_class_registry
		{
			virtual ~pf_class_registry();
			/// Module name. Tagging is handled elsewhere.
			const char* name;

			/// Indicates whether the module handles random or aligned orientations.
			enum class orientation_type {
				ISOTROPIC, ORIENTED
			} orientations;

			/// Angle setup
			struct DLEXPORT_rtmath_core setup {
				setup();
				double beta, theta, phi; // ddscat-based rotation angles
				double sTheta, sTheta0, sPhi, sPhi0; // incident and scattered beam angles (degrees)
				double wavelength; // wavelength of incident light
			};

			/// Used to specify basic stats for constructing a run
			struct DLEXPORT_rtmath_core inputParamsPartial {
				inputParamsPartial();
				double aeff; // equivalent-sphere radius
				enum class aeff_version_type
				{
					EQUIV_V_SPHERE,
					EQUIV_SA_SPHERE
				} aeff_version;
                /// Base refractive index
				std::complex<double> m;
                /// Refractive index scaling method
				std::function<void(std::complex<double>, std::complex<double>, 
                    double, std::complex<double> &)> rmeth;
				/// Rescale effective radius
				bool aeff_rescale; 
				/// Volume fraction
				double vFrac; 
				/// Run description (usually a shape hash to indicate the target)
				std::string ref;

				enum class shape_type
				{
					SPHEROID,
					CYLINDER
				} shape;
				/// spheroid / cylinder aspect ratio
				double eps; 
			};

			/// Cross-section return structure
			struct DLEXPORT_rtmath_core cross_sections {
				cross_sections();
				double Qbk, Qext, Qsca, Qabs, g;
				double Qsca_iso, Qabs_iso;
				double Qbk_iso, Qext_iso, g_iso;
				bool valid;
			};

			/// Phase function return structure
			struct pfs {
				typedef Eigen::Matrix4d PnnType;
				typedef Eigen::Matrix2cd FType;
				PnnType mueller;
				FType S;
				bool valid;
			};

			typedef std::function<void(const setup&, const inputParamsPartial&, cross_sections&)> small_c_type;
			typedef std::function<void(const setup&, const inputParamsPartial&, pfs&)> small_p_type;

			/// Get cross-sections from small stats
			small_c_type fCrossSections;
			/// Get pfs from small stats
			small_p_type fPfs;
		};

		/// Dummy class as a component for usesDLLregistry - keeps different registration types separate
		class pf_registry {};

		/// Provides phase function and cross-sectional information 
		/// from multiple sources, such as DDA, Tmatrix, ...
		///
		/// \todo Add stats-conversion code here
		class DLEXPORT_rtmath_core pf_provider :
			virtual public ::rtmath::registry::usesDLLregistry<
			pf_registry, pf_class_registry>
		{
		public:
			pf_provider(pf_class_registry::orientation_type, const pf_class_registry::inputParamsPartial&);
			virtual ~pf_provider();

			/// \brief Find the first matching handler
			/// \param oriType specifies isotropic or oriented run.
			/// \param name forces match to a specific plugin. NULL causes this parameter to be skipped.
			/// \param res is a container for all matching modules.
			static void findHandler(pf_class_registry::orientation_type oriType, 
				const char* name, const pf_class_registry *res);


			typedef std::vector<std::pair<const char*, pf_class_registry::cross_sections> > resCtype;
			typedef std::vector<std::pair<const char*, pf_class_registry::pfs> > resPtype;
			void getCrossSections(const pf_class_registry::setup&, resCtype& res) const;
			void getPfs(const pf_class_registry::setup&, resPtype& res) const;
		private:
			const pf_class_registry::inputParamsPartial& iparams;
			pf_class_registry::orientation_type otype;
		};

		struct pfRunSetContainer :
			virtual public ::rtmath::registry::usesDLLregistry<
			pfRunSetContainer_IO_input_registry,
			::rtmath::registry::IO_class_registry_reader<pfRunSetContainer> >,
			virtual public ::rtmath::registry::usesDLLregistry<
			pfRunSetContainer_IO_output_registry,
			::rtmath::registry::IO_class_registry_writer<pfRunSetContainer> >,
			virtual public ::rtmath::io::implementsStandardWriter<pfRunSetContainer, pfRunSetContainer_IO_output_registry>,
			virtual public ::rtmath::io::implementsStandardReader<pfRunSetContainer, pfRunSetContainer_IO_input_registry>//,
		{
			pfRunSetContainer();
			virtual ~pfRunSetContainer();

			struct csContainer
			{
				const char* providerName;
				pf_class_registry::setup setup;
				pf_class_registry::inputParamsPartial i;
				pf_class_registry::cross_sections cs;
			};
			std::vector<csContainer> runs;
			/// When the run was imported
			std::string ingest_timestamp;
			/// The system that the run was imported on
			std::string ingest_hostname;
			/// The user account that imported the run
			std::string ingest_username;
			/// Revision of the rtmath code for ingest
			int ingest_rtmath_version;
			/// Version of the phaseFunc code
			int phaseFunc_version;
			/// Current version of the phasefunc code
			const static int max_phaseFunc_version;
			/// Should the results be recalculated in the newest version?
			//bool needsUpgrade() const;
			/// Recalculate all results, using the newest version of the code
			//void upgrade();
		private:
			void _init();
		};
	}

	/* // These will be reimplemented by the da code?
	class phaseFunc // TODO: rewrite this.
	{
	public:
		// This fuction is designed to be pure virtual, so that mie and other
		// phase functions can be included in atmospheric layers
		phaseFunc(void) {}
		virtual ~phaseFunc(void) {}
		// eval evaluates P for reduced mur and phir (mu-mu0, phi-phi0) to get proper directional data
		//virtual void eval(double mu, double mun, double phir, double Pnn[4][4], double res[4][4]);
		// calc constructs the base P matrix, centered on mu=mu0
		//virtual void calc(double mu, std::complex<double> &m, double x, double Pnn[4][4]) = 0;
		//virtual std::shared_ptr<matrixop> eval(double alpha) const = 0;
	protected:
		//mutable std::map<double,std::shared_ptr<matrixop> > _eval_cache;
	};

	class scattMatrix
	{
	public:
		scattMatrix(void) {}
		virtual ~scattMatrix(void) {}
		// Calculate the scattering amplitude matrix for a given mu.
		// Here, mu = cos(alpha), the total scattering angle.
		virtual void calc(double mu, double Snn[4][4], std::complex<double> Sn[4]) = 0;
	public:
		// f is the frequency
		//static void _genMuellerMatrix(double Snn[4][4], const std::complex<double> Sn[4]);

	};
	*/
}
