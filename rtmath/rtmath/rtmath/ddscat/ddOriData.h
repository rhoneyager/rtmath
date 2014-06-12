#pragma once
#include "../defs.h"
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <complex>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
//#include "../interpolatable.h"
#include "../registry.h"
#include "../common_templates.h"
//#include "../phaseFunc.h"
#include "ddOutput.h"
#include "ddVersions.h"
#include "ddScattMatrix.h"
#include "../io.h"

namespace rtmath {
	namespace ddscat {
		class ddOriData;
		class rotations;
		class ddOriData_IO_input_registry {};
		class ddOriData_IO_output_registry {};
		class ddOriData_Standard {};
		class ddScattMatrixConnector;
	}

	namespace registry {
		extern template struct IO_class_registry_writer <
			::rtmath::ddscat::ddOriData > ;

		extern template class usesDLLregistry <
			::rtmath::ddscat::ddOriData_IO_output_registry,
			IO_class_registry_writer<::rtmath::ddscat::ddOriData> > ;

		extern template struct IO_class_registry_reader <
			::rtmath::ddscat::ddOriData > ;

		extern template class usesDLLregistry <
			::rtmath::ddscat::ddOriData_IO_input_registry,
			IO_class_registry_reader<::rtmath::ddscat::ddOriData> > ;
	}

	namespace ddscat {
		
		/// \brief Converts the stat_entries id to a string for display.
		/// \todo Add reverse case, converting from string to id.
		//std::string DLEXPORT_rtmath_ddscat getStatNameFromId(stat_entries);
		//std::string DLEXPORT_rtmath_ddscat getStatNameFromId(stat_entries_size_ts);

		/// Provides local readers and writers for ddscat data (it's a binder)
		class DLEXPORT_rtmath_ddscat implementsDDRES :
			private rtmath::io::implementsIObasic<ddOriData, ddOriData_IO_output_registry,
			ddOriData_IO_input_registry, ddOriData_Standard>
		{
		public:
			virtual ~implementsDDRES() {}
		protected:
			implementsDDRES();
		private:
			static const std::set<std::string>& known_formats();
		};


		/** Class contains the output of a single ddscat fml / sca or avg file
		* This means one radius, wavelength, and orientation.
		**/
		class DLEXPORT_rtmath_ddscat ddOriData :
			public boost::enable_shared_from_this<ddOriData>,
			virtual public ::rtmath::registry::usesDLLregistry<
			::rtmath::ddscat::ddOriData_IO_output_registry,
			::rtmath::registry::IO_class_registry_writer<ddOriData> >,
			virtual public ::rtmath::registry::usesDLLregistry<
			::rtmath::ddscat::ddOriData_IO_input_registry,
			::rtmath::registry::IO_class_registry_reader<ddOriData> >,
			virtual public ::rtmath::io::implementsStandardWriter<ddOriData, ddOriData_IO_output_registry>,
			virtual public ::rtmath::io::implementsStandardReader<ddOriData, ddOriData_IO_input_registry>,
			virtual public implementsDDRES
		{
			friend class ddOutput;

			void doExportFMLs(size_t startIndex) const;
			void doImportFMLs(size_t startIndex, size_t n);
			void doImportFMLs();
		public:
			ddOriData(ddOutput &parent, size_t row,
				const std::string &filenameSCA = "", const std::string &filenameFML = "");
			//ddOriData(ddOutput &parent, size_t row, )
			virtual ~ddOriData();

			// Binders for the standard ddscat formats
			static void readDDSCAT(ddOriData*, std::istream&, std::shared_ptr<registry::IO_options>);
			static void writeDDSCAT(const ddOriData*, std::ostream &, std::shared_ptr<registry::IO_options>);

			double guessTemp(size_t dielIndex = 0) const;

			typedef std::vector<std::pair<size_t, size_t> > mMuellerIndices;

			void writeFML(std::ostream &out) const;
			void writeSCA(std::ostream &out) const;
			void writeAVG(std::ostream &out) const;
			static const mMuellerIndices& mMuellerIndicesDefault();
			void writeMueller(std::ostream &out, const mMuellerIndices &mi = mMuellerIndicesDefault()) const;
			void writeS(std::ostream &out) const;
			void writeF(std::ostream &out) const;
			void writeStatTable(std::ostream &out) const;

			void readFML(std::istream &in);
			void readSCA(std::istream &in);
			void readAVG(std::istream &in);
			//void readHeader(std::istream &in, const std::string &sstop = "Qext");
			void readStatTable(std::istream &in);
			void readMueller(std::istream &in);
			void readF(std::istream &in, boost::shared_ptr<const ddScattMatrixConnector>);

			
#define accessorRW(name,id,valtype) \
	inline valtype name() const { return __getSimple<valtype>((int) id); } \
	inline void name(const valtype &v) { __setSimple<valtype>((int) id, v); }
#define accessorRO(name,id,valtype) \
	inline valtype name() const { return __getSimple<valtype>((int) id); }

			accessorRW(version, ddOutput::stat_entries::VERSION, size_t);
			accessorRW(beta, ddOutput::stat_entries::BETA, double);
			accessorRW(theta, ddOutput::stat_entries::THETA, double);
			accessorRW(phi, ddOutput::stat_entries::PHI, double);
			// Wavelength and frequency set need a special override (affects WAVE, FREQ and D)
			accessorRO(wave, ddOutput::stat_entries::WAVE, double);
			accessorRO(freq, ddOutput::stat_entries::FREQ, double);
			accessorRO(aeff, ddOutput::stat_entries::AEFF, double);
			accessorRO(dipoleSpacing, ddOutput::stat_entries::D, double);
			accessorRO(numDipoles, ddOutput::stat_entries::NUM_DIPOLES, size_t);
			

			std::complex<double> M(size_t dielIndex = 0) const;
			void M(const std::complex<double>&, size_t dielIndex = 0);
			size_t numM() const;

			/// Convenience function to extract the rotation information
			/// \todo Add rotation information reader and writer, and extend storage table
			//void getRots(rotations &rots) const;

			/// Get a connector representing the polarization state
			boost::shared_ptr<const ddScattMatrixConnector> getConnector() const;
			void setConnector(boost::shared_ptr<const ddScattMatrixConnector> cn);


			/// Provides ordering in sets, based on wavelength, aeff, and rotations.
			bool operator<(const ddOriData &rhs) const;
			//typedef std::map<size_t, std::pair<size_t, size_t> > mMuellerIndices;

			/** Need sorting only on load. **/
			/// \todo Need special handling for SCA matrix! Needed for cases where only an avg file gets loaded!!!!!
			typedef std::vector < ddscat::ddScattMatrixF >
				scattMatricesContainer;
			//typedef std::vector < boost::shared_ptr<const ddscat::ddScattMatrix> >
			//	scattMatricesContainer;
			//typedef std::set<boost::shared_ptr<const ddscat::ddScattMatrix>,
			//	sharedComparator<boost::shared_ptr<const ddscat::ddScattMatrix> > >
			//	scattMatricesContainer;

			/// Extract all scattering matrices. Used in ddscat-test.
			//void getScattMatrices(scattMatricesContainer&) const;
			/// Get internal scattering matrix object. Allows direct manipulation of stored data.
			/// \note This can be dangerous, as the returned object will be 
			/// invalidated when the ddOriData object is destroyed.
			//scattMatricesContainer& getScattMatrices();

			/// Count the scattering P matrices
			//size_t numP() const;
			/// Count the scattering F matrices
			//size_t numF() const;
			/// Count the scattering matrices
			size_t numMat() const;

			// All of the stat entries are part of a ddOutput object! This increases performance 
			// by reducing cache misses. As such, this class is an overlay.
			//typedef std::array<std::string, stat_entries::NUM_STAT_ENTRIES_STRINGS> statTableStringType;
			//typedef std::array<double, stat_entries::NUM_STAT_ENTRIES_DOUBLES> statTableDoubleType;
			//typedef std::array<size_t, stat_entries::NUM_STAT_ENTRIES_INTS> statTableSizetType;
			//typedef std::vector<std::complex<double> > refrTableType;
			/// Extract the entire stat table. Used in ddscat-test, io and fast weighting.
			//void getStatTable(statTableDoubleType&) const;
			//void getStatTable(statTableSizetType&) const;
			//void getStatTable(statTableStringType&) const;
			/// Get an individual stat entry.
			/// \see stat_entries
			//double getStatEntry(stat_entries::stat_entries_doubles e) const;
			//size_t getStatEntry(stat_entries::stat_entries_size_ts e) const;
			//std::string getStatEntry(stat_entries::stat_entries_strings e) const;

		protected:

			template< class valtype>
			valtype __getSimple(int id) const { throw; }
			template<> size_t __getSimple<size_t>(int id) const { return _parent.oridata_i(_row, id); }
			template<> double __getSimple<double>(int id) const { return _parent.oridata_d(_row, id); }
			template<> std::string __getSimple<std::string>(int id) const { return _parent.oridata_s.at(_row).at(id); }
			template<class valtype>
			void __setSimple(int id, const valtype val) { __setSimpleRef(id, val); }
			template<class valtype>
			void __setSimpleRef(int id, const valtype &val) { throw; }
			template<> void __setSimpleRef<size_t>(int id, const size_t &val) { _parent.oridata_i(_row, id) = val; }
			template<> void __setSimpleRef<double>(int id, const double &val) { _parent.oridata_d(_row, id) = val; }
			template<> void __setSimpleRef<std::string>(int id, const std::string &val) { _parent.oridata_s.at(_row).at(id) = val; }


			/// Handles role of delegated constructor
			void _init();
			//void _populateDefaults();
			/// Container for the file header
			
			/// Containers for the stat table
			/// \see stat_entries
			//statTableType _statTable;
			//statTableTypeSize_ts _statTable_Size_ts;
			//statTableStringType _statTable_Strings;
			//refrTableType _refrs;

			
			mutable boost::shared_ptr<const ddScattMatrixConnector> _connector;

			/// Container for sca and fml scattering matrices
			/// \see ddScattMatrix
			scattMatricesContainer _scattMatricesRaw;
			/// Container for refractive indices
			//std::vector<std::complex<double> > ms;
			/// Binding to the relevant ddOutput object
			ddOutput &_parent;
			/// Row in the ddOutput tables
			size_t _row;
		};


	}
}