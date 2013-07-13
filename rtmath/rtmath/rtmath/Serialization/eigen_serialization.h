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
/// \todo Allocate matrix
			ar & boost::serialization::make_array(t.data(), t.size());
			//for(size_t i=0; i<t.size(); i++)
			//	ar & t.data()[i];
		}

		/// Definition to serialize all Eigen::Array types
		template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
		inline void serialize(
			Archive & ar, 
			Eigen::Array<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & t, 
			const unsigned int file_version
			) 
		{
/// \todo Allocate matrix
			ar & boost::serialization::make_array(t.data(), t.size());
		}



		//template <class Archive>
		//void serialize(Archive & ar, Eigen::MatrixXd & g, const unsigned int version);
	}
}

