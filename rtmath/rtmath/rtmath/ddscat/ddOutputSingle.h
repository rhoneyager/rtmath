#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <complex>
#include <boost/shared_ptr.hpp>
#include "../matrixop.h"
#include "../interpolatable.h"
#include "../phaseFunc.h"
#include "ddScattMatrix.h"
#include "shapefile.h"
#include "../da/daStatic.h"
#include "../coords.h"
//#include "../depGraph.h"

namespace rtmath {
	
	namespace ddscat {

		class ddOutputSingleObj;

		class ddOutputSingle
		{
			// Class contains the output of a single ddscat fml / sca or avg file
			// Doesn't quite inherit from daStatic.
			// Note: ensemble providers inherit from this!
		public:
			ddOutputSingle(); //
			virtual ~ddOutputSingle(); //

			// Direct reading and writing of ddscat-formatted files (avg, sca and fml)
			void readFile(const std::string &filename);
			void writeFile(const std::string &filename) const;

			void writeFML(std::ostream &out) const;
			void writeSCA(std::ostream &out) const;
			void writeAVG(std::ostream &out) const;
			void writeStatTable(std::ostream &out) const;

			void readFML(std::istream &in); //
			void readSCA(std::istream &in); //
			void readAVG(std::istream &in);
			void readStatTable(std::istream &in);
			void readMueller(std::istream &in);
			void readS(std::istream &in);

			size_t version() const;
			void version(size_t);
			double beta() const;
			double theta() const;
			double phi() const;
			double wave() const;
			double aeff() const;

			bool operator<(const ddOutputSingle &rhs) const; //
		protected:
			size_t _version; //
			double _beta, _theta, _phi, _wave, _aeff; //
			void _init(); //
			//void _populateDefaults();
			std::map< std::string, boost::shared_ptr<ddOutputSingleObj> >
				_objMap;
			std::vector<double> _statTable;
			std::map< rtmath::coords::cyclic<double> , 
				boost::shared_ptr<const ddscat::ddScattMatrix> >
				_scattMatricesRaw;
		};

		class ddOutputSingleObj
		{
		public:
			friend class ddOutputSingle;
			ddOutputSingleObj();
			virtual ~ddOutputSingleObj();
			virtual void write(std::ostream &out) const {}
			virtual void read(std::istream &in) {}
			static void findMap(const std::string &line, std::string &res); //
			static boost::shared_ptr<ddOutputSingleObj> constructObj
				(const std::string &key);
		};

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
	}

}

