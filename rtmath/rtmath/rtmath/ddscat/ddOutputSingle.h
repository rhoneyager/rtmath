#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <complex>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <Eigen/Core>
//#include "../interpolatable.h"
#include "../phaseFunc.h"
#include "ddScattMatrix.h"
#include "shapefile.h"
#include "ddVersions.h"
//#include "../da/daStatic.h"
//#include "../coords.h"
//#include "../depGraph.h"

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

		/** Class contains the output of a single ddscat fml / sca or avg file
		 *
		 * Doesn't quite inherit from daStatic.
		 * \note Ensemble providers inherit from this!
		 * \todo Extend to handle multiple dielectrics
		 **/
		class ddOutputSingle : public boost::enable_shared_from_this<ddOutputSingle>
		{
		public:
			ddOutputSingle(const std::string &filename = "", const std::string &type = "");
			virtual ~ddOutputSingle();

			// Direct reading and writing of ddscat-formatted files (avg, sca and fml)
			void readFile(const std::string &filename, const std::string &type = "");
			void writeFile(const std::string &filename) const;

			void writeFML(std::ostream &out) const;
			void writeSCA(std::ostream &out) const;
			void writeAVG(std::ostream &out) const;
			void writeMueller(std::ostream &out) const;
			void writeS(std::ostream &out) const;
			void writeF(std::ostream &out) const;
			void writeStatTable(std::ostream &out) const;

			void readFML(std::istream &in);
			void readSCA(std::istream &in);
			void readAVG(std::istream &in);
			void readHeader(std::istream &in, const std::string &sstop = "Qext");
			void readStatTable(std::istream &in);
			void readMueller(std::istream &in);
			void readF(std::istream &in);

			/// Normalize based on P11
			/// \todo Needs correct normalization and default values
			boost::shared_ptr<ddOutputSingle> normalize() const; // Normalize based on P11 // TODO

			size_t version() const;
			void version(size_t);
			double beta() const;
			double theta() const;
			double phi() const;
			double wave() const;
			double aeff() const;
			double dipoleSpacing() const;

			typedef std::vector<double> statTableType;

			void getStatTable(statTableType&) const;
			double getStatEntry(stat_entries e) const;

			bool operator<(const ddOutputSingle &rhs) const;
			typedef std::map<size_t, std::pair<size_t, size_t> > mMuellerIndices;

//			typedef std::set<boost::shared_ptr<const ddscat::ddScattMatrix> > 
			typedef std::set<boost::shared_ptr<const ddscat::ddScattMatrix>,
					ddscat::sharedComparator<boost::shared_ptr<const ddscat::ddScattMatrix> > >
				scattMatricesContainer;
			void getScattMatrices(scattMatricesContainer&) const;

			typedef std::map< std::string, boost::shared_ptr<ddOutputSingleObj> >
				headerMap;
			void getHeaderMaps(headerMap&) const;
			boost::shared_ptr<ddOutputSingleObj> getObj(const std::string &id) const;

		protected:
			size_t _version;
			mMuellerIndices _muellerMap;
			double _beta, _theta, _phi, _wave, _aeff;
			void _init();
			//void _populateDefaults();
			headerMap _objMap;
			statTableType _statTable;
			scattMatricesContainer _scattMatricesRaw;
		};

		class ddOutputSingleObj
		{
		public:
			friend class ddOutputSingle;
			ddOutputSingleObj();
			virtual ~ddOutputSingleObj();
			virtual void write(std::ostream &out, size_t version
					= rtmath::ddscat::ddVersions::getDefaultVer()) const = 0;
			virtual void read(std::istream &in) = 0;
			virtual std::string value() const = 0; //{return std::string(); }
			virtual bool operator==(const ddOutputSingleObj&) const;
			virtual bool operator!=(const ddOutputSingleObj&) const;
			static void findMap(const std::string &line, std::string &res); //
			static boost::shared_ptr<ddOutputSingleObj> constructObj
				(const std::string &key);
		};

	}

}

std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::ddOutputSingleObj &ob);

