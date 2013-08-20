#pragma once
#include "../defs.h"
#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>
#include <map>
#include <complex>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/version.hpp>
//#include "ddOutputSingle.h"
//#include "ddScattMatrix.h"
//#include "ddweights.h"
#include "../hash.h"

namespace rtmath {

	namespace ddscat {
		class ddOutputSingle;
		class ddOutput;
		class ddOutputGenerator;
		class shapefile;
		class shapeFileStats;


		/**
		 * \brief The ensemble subclasses reweight the data to represent various ensemble schemes
		 **/
		class DLEXPORT_rtmath_ddscat ddOutputGenerator 
			: public boost::enable_shared_from_this<ddOutputGenerator>
		{
			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
			ddOutputGenerator();
		public:
			virtual ~ddOutputGenerator() {}
			virtual boost::shared_ptr<ddOutput> generate(boost::shared_ptr<ddOutput> source) const = 0;
		protected:
			ddOutputGenerator(boost::shared_ptr<ddOutput> source);
			boost::shared_ptr<ddOutput> src, res;
		};
		/*
		/// The 'dumb' ensemble class that assumes that all weights are the same, 
		/// ignoring cos(theta) effects.
		class ddOutputGeneratorIsoAll 
			: public ddOutputGenerator
		{
			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
		protected:
			/// Standard constructor
			ddOutputGeneratorIsoAll(boost::shared_ptr<ddOutput> source);
			/// Constructor used when loading a DDSCAT results folder for the first time
			ddOutputGeneratorIsoAll(std::set<boost::shared_ptr<ddOutputSingle> > &scas);
		public:
			virtual ~ddOutputGeneratorIsoAll() {}
			virtual boost::shared_ptr<ddOutput> generate(boost::shared_ptr<ddOutput> source) const override;
			virtual boost::shared_ptr<ddOutput> generate(std::set<boost::shared_ptr<ddOutputSingle> > &scas) const;
		};

		/// The trivial ensemble class that duplicates DDSCAT results
		class ddOutputGeneratorDDSCAT 
			: public ddOutputGenerator
		{
			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
		protected:
			/// Standard constructor
			ddOutputGeneratorDDSCAT(boost::shared_ptr<ddOutput> source);
			/// Constructor used when loading a DDSCAT results folder for the first time
			ddOutputGeneratorDDSCAT(std::set<boost::shared_ptr<ddOutputSingle> > &scas);
		public:
			virtual ~ddOutputGeneratorDDSCAT() {}
			virtual boost::shared_ptr<ddOutput> generate(boost::shared_ptr<ddOutput> source) const override;
			virtual boost::shared_ptr<ddOutput> generate(std::set<boost::shared_ptr<ddOutputSingle> > &scas) const;
		};

		/// Ensemble class that filters DDSCAT results to only match a certain theta value (usually zero).
		/// Represents the fully-aligned case.
		class ddOutputGeneratorThetaAligned
			: public ddOutputGenerator
		{
			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
		protected:
			/// Standard constructor
			ddOutputGeneratorThetaAligned(boost::shared_ptr<ddOutput> source, double theta = 0);
			/// Constructor used when loading a DDSCAT results folder for the first time
			ddOutputGeneratorThetaAligned(std::set<boost::shared_ptr<ddOutputSingle> > &scas, double theta = 0);
		public:
			virtual ~ddOutputGeneratorThetaAligned() {}
			virtual boost::shared_ptr<ddOutput> generate(boost::shared_ptr<ddOutput> source) const override;
			virtual boost::shared_ptr<ddOutput> generate(std::set<boost::shared_ptr<ddOutputSingle> > &scas) const;
		};
		*/

		/// \todo Add interpolation class elsewhere. The interpolation class will be able to interpolate a rotation and 
		/// a set of scattering angles.


		/// \todo Add truncated Gaussian ensemble generator. Will restrict in one or more dimensions.

		/*
		// Not a child of ddOutputSingle, but a provider for ddOutput
		// This class instead CREATES a ddOutputSingle that
		// reflects the weighting distribution as-is (not updated if original
		// ddOutput gains more ddOutputSingles).
		class ddOutputEnsemble //: public ddOutputSingle
		{
		public:
			ddOutputEnsemble();
			virtual ~ddOutputEnsemble();
			// Long function definition here, even though the invocation is short...
			virtual void genEnsemble(const std::unordered_map<coords::cyclic<double>, 
				boost::shared_ptr<const ddscat::ddOutputSingle>, 
				boost::hash<coords::cyclic<double> > 
				> &_mapOutputSingleRaw, 
				ddOutputSingle &res) const = 0;
		};

		class ddOutputEnsembleGaussian : public ddOutputEnsemble
		{
		public:
			ddOutputEnsembleGaussian(double sigma, size_t coord_varying);
			virtual ~ddOutputEnsembleGaussian();
			virtual void genEnsemble(const std::unordered_map<coords::cyclic<double>, 
				boost::shared_ptr<const ddscat::ddOutputSingle>, 
				boost::hash<coords::cyclic<double> > 
				> &_mapOutputSingleRaw, 
				ddOutputSingle &res) const;
		private:
			double _sigma;
			size_t _coord_varying;
		};

		class ddOutputEnsembleIsotropic : public ddOutputEnsemble
		{
		public:
			ddOutputEnsembleIsotropic(size_t coord_varying);
			virtual ~ddOutputEnsembleIsotropic();
			virtual void genEnsemble(const std::unordered_map<coords::cyclic<double>, 
				boost::shared_ptr<const ddscat::ddOutputSingle>, 
				boost::hash<coords::cyclic<double> > 
				> &_mapOutputSingleRaw, 
				ddOutputSingle &res) const;
		private:
			size_t _coord_varying;
		};

		class ddOutputEnsembleAligned : public ddOutputEnsemble
		{
		public:
			ddOutputEnsembleAligned();
			virtual ~ddOutputEnsembleAligned();
			virtual void genEnsemble(const std::unordered_map<coords::cyclic<double>, 
				boost::shared_ptr<const ddscat::ddOutputSingle>, 
				boost::hash<coords::cyclic<double> > 
				> &_mapOutputSingleRaw, 
				ddOutputSingle &res) const;
		private:
			size_t _coord_varying;
		};
		*/
	}
}


BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputGenerator);
BOOST_SERIALIZATION_ASSUME_ABSTRACT(rtmath::ddscat::ddOutputGenerator);

