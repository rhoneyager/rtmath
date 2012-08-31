#pragma once

namespace rtmath
{
	namespace tmatrix
	{
		struct tmIn;
		struct tmOut;
		struct tmRun;
		class tmData;
	}
}

namespace boost
{
	namespace serialization
	{
		template<class Archive>
		void serialize(Archive & ar, rtmath::tmatrix::tmIn & g, const unsigned int version);
		template<class Archive>
		void serialize(Archive & ar, rtmath::tmatrix::tmOut & g, const unsigned int version);
		template<class Archive>
		void serialize(Archive & ar, rtmath::tmatrix::tmRun & g, const unsigned int version);
		template<class Archive>
		void serialize(Archive & ar, rtmath::tmatrix::tmData & g, const unsigned int version);
	}
}
