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
#include "../phaseFunc.h"
#include "ddScattMatrix.h"
#include "shapefile.h"
#include "ddVersions.h"
#include "../io.h"
//#include "../da/daStatic.h"
//#include "../coords.h"
//#include "../depGraph.h"

namespace rtmath
{
	namespace ddscat
	{
		class ddOutputSingleObj;
		class ddOutputSingle;
		class rotations;
		class ddOutputSingle_IO_input_registry {};
		class ddOutputSingle_IO_output_registry {};
		class ddOutputSingle_serialization {};
		class ddOutputSingle_Standard {};
	}

	namespace registry {
		extern template struct IO_class_registry_writer<
			::rtmath::ddscat::ddOutputSingle>;

		extern template class usesDLLregistry<
			::rtmath::ddscat::ddOutputSingle_IO_output_registry,
			IO_class_registry_writer<::rtmath::ddscat::ddOutputSingle> >;

		extern template struct IO_class_registry_reader<
			::rtmath::ddscat::ddOutputSingle>;

		extern template class usesDLLregistry<
			::rtmath::ddscat::ddOutputSingle_IO_input_registry,
			IO_class_registry_reader<::rtmath::ddscat::ddOutputSingle> >;
	}

	namespace ddscat {

		class ddOutputSingleObj;

		/// \brief The listing of the stat table entries
		///
		/// \note These have to be aligned with ddOutput::oriColDefs
		/// \see ddOutput::oriColDefs
		enum stat_entries
		{
			BETA,THETA,PHI,WAVE,FREQ,AEFF,DIPOLESPACING,
			QEXT1,QABS1,QSCA1,G11,G21,QBK1,QPHA1,
			QEXT2,QABS2,QSCA2,G12,G22,QBK2,QPHA2,
			QEXTM,QABSM,QSCAM,G1M,G2M,QBKM,QPHAM,
			QPOL,DQPHA,
			QSCAG11,QSCAG21,QSCAG31,ITER1,MXITER1,NSCA1,
			QSCAG12,QSCAG22,QSCAG32,ITER2,MXITER2,NSCA2,
			QSCAG1M,QSCAG2M,QSCAG3M,
			NUM_STAT_ENTRIES
		};

		enum stat_entries_size_ts
		{
			VERSION,NUM_DIPOLES,NUMP,NUMF,
			NUM_STAT_ENTRIES_INTS
		};

		/// \brief Converts the stat_entries id to a string for display.
		/// \todo Add reverse case, converting from string to id.
		std::string DLEXPORT_rtmath_ddscat getStatNameFromId(stat_entries);
		std::string DLEXPORT_rtmath_ddscat getStatNameFromId(stat_entries_size_ts);

		/// Provides local readers and writers for ddscat data (it's a binder)
		class DLEXPORT_rtmath_ddscat implementsDDRES :
			private rtmath::io::implementsIObasic<ddOutputSingle, ddOutputSingle_IO_output_registry,
			ddOutputSingle_IO_input_registry, ddOutputSingle_Standard>
		{
		public:
			virtual ~implementsDDRES() {}
		protected:
			implementsDDRES();
		private:
			static const std::set<std::string>& known_formats();
		};

		/** Class contains the output of a single ddscat fml / sca or avg file
		 *
		 * Doesn't quite inherit from daStatic.
		 * \note Ensemble providers inherit from this!
		 * \todo Extend to handle multiple dielectrics
		 **/
		class DLEXPORT_rtmath_ddscat DEPRECATED ddOutputSingle :
			public boost::enable_shared_from_this<ddOutputSingle>,
			virtual public ::rtmath::registry::usesDLLregistry<
			    ::rtmath::ddscat::ddOutputSingle_IO_output_registry,
			    ::rtmath::registry::IO_class_registry_writer<ddOutputSingle> >,
			virtual public ::rtmath::registry::usesDLLregistry<
			    ::rtmath::ddscat::ddOutputSingle_IO_input_registry,
			    ::rtmath::registry::IO_class_registry_reader<ddOutputSingle> >,
			virtual public ::rtmath::io::implementsStandardWriter<ddOutputSingle, ddOutputSingle_IO_output_registry>,
			virtual public ::rtmath::io::implementsStandardReader<ddOutputSingle, ddOutputSingle_IO_input_registry>,
			virtual public implementsDDRES
		{
			friend class ddOutput;
			
			void doExportOri(boost::shared_ptr<ddOutput> parent, size_t index, bool isavg = false);
			void doExportFMLs(boost::shared_ptr<ddOutput> parent, size_t startIndex, size_t oriIndex);
			void doImportOri(boost::shared_ptr<ddOutput> parent, size_t index, bool isavg = false);
			//void doExportSCAs(boost::shared_ptr<ddOutput> parent, size_t startIndex, size_t oriIndex);
		public:
			ddOutputSingle(const std::string &filename = "", const std::string &type = "");
			virtual ~ddOutputSingle();
			/// Deep-copy constructor
			ddOutputSingle(const ddOutputSingle&);

			static void readDDSCAT(ddOutputSingle*, std::istream&, std::shared_ptr<registry::IO_options>);
			static void writeDDSCAT(const ddOutputSingle*, std::ostream &, std::shared_ptr<registry::IO_options>);

			/// Guess the temperature, assuming ice.
			double guessTemp() const;

			/// Output in fml format
			void writeFML(std::ostream &out) const;
			/// Output in sca format
			void writeSCA(std::ostream &out) const;
			/// Output in avg format
			void writeAVG(std::ostream &out) const;
			/// Output Mueller matrices
			void writeMueller(std::ostream &out) const;
			/// Output S matrices
			void writeS(std::ostream &out) const;
			/// Output F matrices (fml files only)
			void writeF(std::ostream &out) const;
			/// Output the Qext, Qbk, Qsca, ... table
			/// \see stat_entries
			void writeStatTable(std::ostream &out) const;

			/// Input in fml format
			void readFML(std::istream &in);
			/// Input in sca format
			void readSCA(std::istream &in);
			/// Input in avg format
			void readAVG(std::istream &in);
			/// Input the header
			void readHeader(std::istream &in, const std::string &sstop = "Qext");
			/// Input the stat table
			/// \see stat_entries
			void readStatTable(std::istream &in);
			/// Input Mueller matrices
			void readMueller(std::istream &in);
			/// Input fml F table
			void readF(std::istream &in, boost::shared_ptr<const ddScattMatrixConnector>);

			/// Normalize based on P11
			/// \todo Needs correct normalization and default values
			boost::shared_ptr<ddOutputSingle> normalize() const; // Normalize based on P11 // TODO

			/// Returns ddscat version of the file
			size_t version() const;
			/// Set ddscat version for file writes
			void version(size_t);
			/// Beta rotation (sca and fml)
			double beta() const;
			/// Theta rotation (sca and fml)
			double theta() const;
			/// Phi rotation (sca and fml)
			double phi() const;
			/// Wavelength
			double wave() const;
			/// Frequency (convenience function to implicitly convert from wavenumber)
			double freq() const;
			/// Effective radius
			double aeff() const;
			/// Interdipole spacing (not all read types)
			double dipoleSpacing() const;
			/// Number of dipoles (not all read types)
			size_t numDipoles() const;

			/// Refractive index
			std::complex<double> getM() const;

			typedef std::vector<double> statTableType;
			typedef std::vector<size_t> statTableTypeSize_ts;

			/// Extract the entire stat table. Used in ddscat-test, io and fast weighting.
			void getStatTable(statTableType&) const;
			void getStatTable(statTableTypeSize_ts&) const;
			/// Get an individual stat entry.
			/// \see stat_entries
			double getStatEntry(stat_entries e) const;
			size_t getStatEntry(stat_entries_size_ts e) const;


			/// Provides ordering in sets, based on wavelength, aeff, and rotations.
			bool operator<(const ddOutputSingle &rhs) const;
			typedef std::map<size_t, std::pair<size_t, size_t> > mMuellerIndices;

//			typedef std::set<boost::shared_ptr<const ddscat::ddScattMatrix> > 
			typedef std::set<boost::shared_ptr<const ddscat::ddScattMatrix>,
					sharedComparator<boost::shared_ptr<const ddscat::ddScattMatrix> > >
				scattMatricesContainer;
			/// Extract all scattering matrices. Used in ddscat-test.
			void getScattMatrices(scattMatricesContainer&) const;
			/// Get internal scattering matrix object. Allows direct manipulation of stored data.
			/// \note This can be dangerous, as the returned object will be 
			/// invalidated when the ddOutputSingle object is destroyed.
			scattMatricesContainer& getScattMatrices();

			/// Count the scattering P matrices
			size_t numP() const;
			/// Count the scattering F matrices
			size_t numF() const;

			typedef std::map< std::string, boost::shared_ptr<ddOutputSingleObj> >
				headerMap;
			/// Extract all headers. Used in ddscat-test.
			void getHeaderMaps(headerMap&) const;
			/// Extract a single header line, by key.
			/// \see ddOutputSingleObj
			boost::shared_ptr<ddOutputSingleObj> getObj(const std::string &id) const;
			/// Convenience function for getting the TARGET information
			void getTARGET(std::string &target) const;
			/// Convenience function for setting the TARGET information
			void setTARGET(const std::string &target);

			/// Convenience function to extract the rotation information
			void getRots(rotations &rots) const;

			/// Get a connector representing the polarization state
			boost::shared_ptr<const ddScattMatrixConnector> getConnector() const;
			/// Set connector representing polarization state.
			void setConnector(boost::shared_ptr<const ddScattMatrixConnector> cn);
		protected:
			// The ddscat version of the file
			//size_t _version;
			/// The listing of the stored Mueller indices
			mMuellerIndices _muellerMap;
			// These values are kept here for ordering purposes
			//double _beta, _theta, _phi, _wave, _aeff;
			/// Handles role of delegated constructor
			void _init();
			//void _populateDefaults();
			/// Container for the file header
			headerMap _objMap;
			
			/// Containers for the stat table
			/// \see stat_entries
			statTableType _statTable;
			statTableTypeSize_ts _statTable_Size_ts;

			/// Container for sca and fml scattering matrices
			/// \see ddScattMatrix
			scattMatricesContainer _scattMatricesRaw;

		};

		/// Base class for ddOutputSingle header entries
		class SHARED_INTERNAL ddOutputSingleObj
		{
			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
		public:
			ddOutputSingleObj();
			virtual ~ddOutputSingleObj();
			std::string getKey() const { return key; }
			/// Write header line using formatting appropriate for the 
			/// specified ddscat version.
			/// \see rtmath::ddscat::ddVersions
			virtual void write(std::ostream &out, size_t version
					= rtmath::ddscat::ddVersions::getDefaultVer()) const = 0;
			/// Parse header line input into object
			virtual void read(std::istream &in) = 0;
			/// Reports object value casted as a std::string. 
			/// Not all derived classes have a value to report.
			virtual std::string value() const = 0; //{return std::string(); }
			/// Check for equality of two objects by writing them and comparing the strings.
			/// Used in ddscat-test
			virtual bool operator==(const ddOutputSingleObj&) const;
			/// Check inequality of two objects
			/// \see operator==
			virtual bool operator!=(const ddOutputSingleObj&) const;
			/// Duplicate an object
			virtual boost::shared_ptr<ddOutputSingleObj> clone() const;
			/// Based on the text in the input line, determine the headerMap key string
			static void findMap(const std::string &line, std::string &res);
			/// Construct a ddOutputSingleObject derived class instance to hold the 
			/// specified key type
			static boost::shared_ptr<ddOutputSingleObj> constructObj
				(const std::string &key);
		private:
			/// Set the 'key' of the object
			void setKey(const std::string&);
			std::string key;
			friend class ddOutputSingle;
		};

	}

}

/// Provides ability to stream a rtmath::ddscat::ddOutputSingleObj
std::ostream & operator<<(std::ostream&, const rtmath::ddscat::ddOutputSingleObj&);
/*
std::ostream & operator<<(std::ostream&, const rtmath::ddscat:: &);
std::istream & operator>>(std::istream&, rtmath::ddscat:: &);
*/

#pragma deprecated(ddOutputSingle)
