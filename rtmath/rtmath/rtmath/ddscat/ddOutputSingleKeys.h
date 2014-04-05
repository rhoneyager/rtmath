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
#include <boost/serialization/export.hpp>
#include <boost/serialization/assume_abstract.hpp>

#include "ddOutputSingle.h"

#undef min
#undef max
namespace rtmath
{
	namespace ddscat {
		namespace ddOutputSingleKeys {

			template <typename Iterator>
			bool parse_string_ddNval(Iterator first, Iterator last, size_t pos, double &val);

			class SHARED_INTERNAL ddver : public ::rtmath::ddscat::ddOutputSingleObj
			{
			public:
				ddver();
				virtual ~ddver();
				virtual void write(std::ostream &out, size_t) const override;
				virtual void read(std::istream &in) override;
				size_t _version;
				size_t version() const;
				void version(size_t n);
				virtual std::string value() const override;
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version);
			};
			class SHARED_INTERNAL ddstring : public ::rtmath::ddscat::ddOutputSingleObj
			{
			public:
				ddstring();
				virtual ~ddstring();
				virtual void write(std::ostream &out, size_t) const override;
				virtual void read(std::istream &in) override;
				virtual std::string value() const override;
				std::string s;
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version);
			};
			class SHARED_INTERNAL ddtarget : public ::rtmath::ddscat::ddOutputSingleObj
			{
			public:
				ddtarget();
				virtual ~ddtarget();
				virtual void write(std::ostream &out, size_t) const override;
				virtual void read(std::istream &in) override;
				void setTarget(const std::string &n);
				virtual std::string value() const override;
				std::string s;
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version);
			};
			class SHARED_INTERNAL ddSval : public ::rtmath::ddscat::ddOutputSingleObj
			{
			public:
				ddSval(const std::string &tail = "");
				virtual ~ddSval();
				virtual void write(std::ostream &out, size_t) const override;
				virtual void read(std::istream &in) override;
				std::string s, tail;
				virtual std::string value() const override;
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version);
			};

			template <class T>
			class SHARED_INTERNAL ddNval : public ::rtmath::ddscat::ddOutputSingleObj
			{
			public:
				ddNval(size_t pos = 0, const std::string &head = "", const std::string &tail = "") {}
				virtual ~ddNval() {}
				virtual void write(std::ostream &out, size_t) const override
				{
					out << head << val << tail << std::endl;
				}
				virtual std::string value() const override
				{
					std::ostringstream out;
					out << val;
					return out.str();
				}
				virtual void read(std::istream &in) override
				{
					std::string lin, junk;
					std::getline(in, lin);
					double pv = 0;
					parse_string_ddNval(lin.begin(), lin.end(), pos, pv);
					val = static_cast<T>(pv);
				}
				size_t pos;
				T val;
				std::string head, tail;
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & boost::serialization::make_nvp(
						"base",
						boost::serialization::base_object<rtmath::ddscat::ddOutputSingleObj>(*this));
					ar & boost::serialization::make_nvp("pos", pos);
					ar & boost::serialization::make_nvp("val", val);
					ar & boost::serialization::make_nvp("head", head);
					ar & boost::serialization::make_nvp("tail", tail);
				}
			};

			/// Provides access to refractive indices
			class SHARED_INTERNAL ddM : public ::rtmath::ddscat::ddOutputSingleObj
			{
			public:
				ddM();
				virtual ~ddM();
				virtual void write(std::ostream &out, size_t) const override;
				virtual void read(std::istream &in) override;
				virtual std::string value() const override;
				std::complex<double> getM() const;
				std::complex<double> getEps() const;
				float getMkd() const;
				std::complex<double> m, eps;
				float mkd;
				const size_t w;
				size_t subst;
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version);
			};

			enum class frameType { LF, TF };

			/// Provides access to polarization vector information
			class SHARED_INTERNAL ddPolVec
				: public ::rtmath::ddscat::ddOutputSingleObj
			{
			public:
				ddPolVec();
				virtual ~ddPolVec();
				virtual void write(std::ostream &out, size_t) const override;
				virtual void read(std::istream &in) override;
				virtual std::string value() const override;
				std::complex<double> getPol(size_t n) const;
				frameType getFrame() const;
				size_t getVecnum() const;
				std::complex<double> pols[3];
				size_t vecnum;
				frameType frame;
				const size_t w;
				const size_t p;
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version);
			};

			/// Provides access to rotation information
			class SHARED_INTERNAL ddRot1d
				: public ::rtmath::ddscat::ddOutputSingleObj
			{
			public:
				ddRot1d();
				virtual ~ddRot1d();
				virtual void write(std::ostream &out, size_t) const override;
				virtual void read(std::istream &in) override;
				virtual std::string value() const override;

				double min, max;
				size_t n;
				std::string fieldname, fieldnamecaps;

				const size_t w;
				const size_t p;
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version);
			};
		}
	}
}

BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::ddver);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::ddstring);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::ddtarget);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::ddSval);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::ddNval<size_t>);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::ddNval<double>);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::ddM);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::ddPolVec);
