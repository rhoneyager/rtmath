#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <complex>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/assume_abstract.hpp>
#include <Eigen/Core>
//#include "../interpolatable.h"
#include "../phaseFunc.h"
#include "ddScattMatrix.h"
#include "shapefile.h"
#include "ddVersions.h"
//#include "../da/daStatic.h"
//#include "../coords.h"
//#include "../depGraph.h"

namespace rtmath
{
	namespace ddscat
	{
		class ddOutputSingleObj;
		class ddOutputSingle;
	}
}

namespace rtmath {

	namespace ddscat {

		class ddOutputSingleObj;

		/// The listing of the stat table entries
		enum stat_entries
		{
			QEXT1,QABS1,QSCA1,G11,G21,QBK1,QPHA1,
			QEXT2,QABS2,QSCA2,G12,G22,QBK2,QPHA2,
			QEXTM,QABSM,QSCAM,G1M,G2M,QBKM,QPHAM,
			QPOL,DQPHA,
			QSCAG11,QSCAG21,GSCAG31,ITER1,MXITER1,NSCA1,
			QSCAG12,QSCAG22,GSCAG32,ITER2,MXITER2,NSCA2,
			QSCAG1M,QSCAG2M,QSCAG3M,
			NUM_STAT_ENTRIES
		};

		/// \brief Converts the stat_entries id to a string for display.
		/// \todo Add reverse case, converting from string to id.
		std::string getStatNameFromId(stat_entries);

		/// Stream-formatting template enums that handle I/O.
		enum ddOutputSingleAsType
		{
			/// I/O as .avg file
			AVG,
			/// I/O as .sca file
			SCA,
			/// I/O as .fml file
			FML
		};
		/// Class to handle stream-formatting.
		class ddOutputSingleAsClass
		{
			/// Private constructor invoked by ddOutputSingle::as<T>
			ddOutputSingleAsClass(ddOutputSingle&, ddOutputSingleAsType);
			friend class ddOutputSingle;
			/// Type of file I/O
			ddOutputSingleAsType _type;
			/// Reference to object
			ddOutputSingle &_ref;
		};

		/** Class contains the output of a single ddscat fml / sca or avg file
		 *
		 * Doesn't quite inherit from daStatic.
		 * \note Ensemble providers inherit from this!
		 * \todo Extend to handle multiple dielectrics
		 **/
		class ddOutputSingle : public boost::enable_shared_from_this<ddOutputSingle>
		{
			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
		public:
			ddOutputSingle(const std::string &filename = "", const std::string &type = "");
			virtual ~ddOutputSingle();

			// Direct reading and writing of ddscat-formatted files (avg, sca and fml)

			/// Read a sca, fml, avg or xml file. Handles compression.
			void readFile(const std::string &filename, const std::string &type = "");
			/// Write a sca, fml, avg or xml file. Handles compression.
			void writeFile(const std::string &filename, const std::string &type = "") const;

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
			void readF(std::istream &in);

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
			/// Effective radius
			double aeff() const;
			/// Interdipole spacing (not all read types)
			double dipoleSpacing() const;

			/// Refractive index
			std::complex<double> getM() const;

			typedef std::vector<double> statTableType;

			/// Extract the entire stat table. Used in ddscat-test.
			void getStatTable(statTableType&) const;
			/// Get an individual stat entry.
			/// \see stat_entries
			double getStatEntry(stat_entries e) const;

			/// Provides ordering in sets, based on wavelength, aeff, and rotations.
			bool operator<(const ddOutputSingle &rhs) const;
			typedef std::map<size_t, std::pair<size_t, size_t> > mMuellerIndices;

//			typedef std::set<boost::shared_ptr<const ddscat::ddScattMatrix> > 
			typedef std::set<boost::shared_ptr<const ddscat::ddScattMatrix>,
					ddscat::sharedComparator<boost::shared_ptr<const ddscat::ddScattMatrix> > >
				scattMatricesContainer;
			/// Extract all scattering matrices. Used in ddscat-test.
			void getScattMatrices(scattMatricesContainer&) const;

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


			/// Function to modify I/O stream type
			template <ddOutputSingleAsType T>
			void as();
			/// Function to modify I/O stream type
			template <ddOutputSingleAsType T>
			void as() const;

			//void asAVG() const;
			//void asSCA() const;
			//void asFML() const;
		protected:
			/// The ddscat version of the file
			size_t _version;
			/// The listing of the stored Mueller indices
			mMuellerIndices _muellerMap;
			// These values are kept here for ordering purposes
			double _beta, _theta, _phi, _wave, _aeff;
			/// Handles role of delegated constructor
			void _init();
			//void _populateDefaults();
			/// Container for the file header
			headerMap _objMap;
			/// Container for the stat table
			/// \see stat_entries
			statTableType _statTable;
			/// Container for sca and fml scattering matrices
			/// \see ddScattMatrix
			scattMatricesContainer _scattMatricesRaw;
		};

		/// Base class for ddOutputSingle header entries
		class ddOutputSingleObj
		{
			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
		public:
			ddOutputSingleObj();
			virtual ~ddOutputSingleObj();
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
			/// Based on the text in the input line, determine the headerMap key string
			static void findMap(const std::string &line, std::string &res);
			/// Construct a ddOutputSingleObject derived class instance to hold the 
			/// specified key type
			static boost::shared_ptr<ddOutputSingleObj> constructObj
				(const std::string &key);
		private:
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

BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingle);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleObj);
BOOST_SERIALIZATION_ASSUME_ABSTRACT(rtmath::ddscat::ddOutputSingleObj);

