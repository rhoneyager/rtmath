#pragma once
#include <Eigen/Core>
#include <boost/serialization/array.hpp>

namespace boost
{
	namespace serialization
	{
		/// Definition to serialize all Eigen::Matrix types
		template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
		inline void serialize(
			Archive & ar, 
			Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & t, 
			const unsigned int file_version
			) 
		{
			//boost::serialization::split_free(ar, *this, version);
			// Doing this all in one function
			using namespace boost::serialization;
			// rows and cols have slightly different types depending on platform and architecture.
			auto rows = t.rows(), cols = t.cols();
			ar & make_nvp("Rows", rows);
			ar & make_nvp("Cols", cols);
			if (rows != t.rows() || cols != t.cols())
				t.resize(rows,cols);
			ar & make_nvp("Data", make_array(t.data(), t.size()));
		}

		/// Definition to serialize all Eigen::Array types
		template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
		inline void serialize(
			Archive & ar, 
			Eigen::Array<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & t, 
			const unsigned int file_version
			) 
		{
			//ar & boost::serialization::make_array(t.data(), t.size());
			using namespace boost::serialization;
			auto rows = t.rows(), cols = t.cols();
			ar & make_nvp("Rows", rows);
			ar & make_nvp("Cols", cols);
			if (rows != t.rows() || cols != t.cols())
				t.resize(rows,cols);
			ar & make_nvp("Data", make_array(t.data(), t.size()));

		}

		//template <class Archive>
		//void serialize(Archive & ar, Eigen::MatrixXd & g, const unsigned int version);
	}
}

