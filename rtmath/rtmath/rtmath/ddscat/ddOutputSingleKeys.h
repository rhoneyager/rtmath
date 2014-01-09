#pragma once
#include "../defs.h"
#include <boost/lexical_cast.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/string.hpp>
//#include <boost/tuple/tuple.hpp> 
#include "../common_templates.h"

namespace rtmath
{
	namespace ddscat
	{

		namespace ddOutputSingleKeys
		{
			/// Enum contains the keys of the mapped objects
			enum class mapKeys
			{
				UNKNOWN,
				VERSION,
				TARGET,
				SOLNMETH,
				POLARIZABILITY,
				SHAPE,
				NUMDIPOLES,
				D_AEFF,
				D,
				PHYSEXTENT,
				AEFF,
				WAVE,
				K_AEFF,
				NAMBIENT,
				NEPS,
				TOL,
				A1TGT,
				A2TGT,
				NAVG,
				KVECLF,
				KVECTF,
				INCPOL1LF,
				INCPOL1TF,
				INCPOL2LF,
				INCPOL2TF,
				BETARANGE,
				THETARANGE,
				PHIRANGE,
				ETASCA,
				AVGNUMORI,
				AVGNUMPOL,
				XTF,
				YTF,
				ZTF,
				BETA,
				THETA,
				PHI,
				TARGETPERIODICITY,
				FMLDOTLINE,
				MDEF
			};

			/// Base class for ddOutputSingle header entries
			class SHARED_INTERNAL ddOutputSingleObj
			{
				friend class ::boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version);
			public:
				ddOutputSingleObj(mapKeys key = mapKeys::UNKNOWN);
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
				//template <size_t i = 0, class U = std::string>
				//virtual U value() const = 0;
				//template <size_t i = 0, class U = std::string>
				//virtual void value(U &newval) = 0;

				/// Check for equality of two objects by writing them and comparing the strings.
				/// Used in ddscat-test
				virtual bool operator==(const ddOutputSingleObj&) const;
				/// Check inequality of two objects
				/// \see operator==
				virtual bool operator!=(const ddOutputSingleObj&) const;
				/// Duplicate an object
				virtual boost::shared_ptr<ddOutputSingleObj> clone() const;
				/*
				/// Based on the text in the input line, determine the headerMap key string
				//static void findMap(const std::string &line, std::string &res);
				/// Construct a ddOutputSingleObject derived class instance to hold the 
				/// specified key type
				//static boost::shared_ptr<ddOutputSingleObj> constructObj
				//	(const std::string &key);
				*/
				/// Function to read a ddscat output file line and store it properly
				static boost::shared_ptr<ddOutputSingleObj> findMap(const std::string &line);
				/// Constructs a value container based on the specified key
				static boost::shared_ptr<ddOutputSingleObj> constructObj(mapKeys);

			private:
				/// Set the 'key' of the object
				void setKey(mapKeys);
				mapKeys key;
				friend class ddOutputSingle;
			};

			/** \todo Boost tuple serialization implementation needs to be 
			 * updated to work with clang.
			 **/
			template <typename... Arguments>
			class hasMultipleValues : public virtual ddOutputSingleObj
			{
			public:
				hasMultipleValues(mapKeys key = mapKeys::UNKNOWN) : ddOutputSingleObj(key) {}
				hasMultipleValues(Arguments... args, mapKeys key = mapKeys::UNKNOWN)
					: ddOutputSingleObj(key), val(args) {}
			public:
				template<size_t i = 0> inline Arguments<i>& value() { return std::get<i>(val); }
				template<size_t i = 0> inline const Arguments<i>& value() const { return std::get<i>(val); }
			protected:
				std::tuple<Arguments...> val;
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & boost::serialization::make_nvp(
						"base",
						boost::serialization::base_object<rtmath::ddscat::ddOutputSingleKeys::ddOutputSingleObj>(*this));
					ar & boost::serialization::make_nvp("values", val);
				}
			};

			template <class T>
			class hasValue : public virtual hasMultipleValues<T>
			{
			public:
				hasValue(mapKeys key = mapKeys::UNKNOWN) : ddOutputSingleObj(key) {}
				hasValue(const T& defaultval, mapKeys key = mapKeys::UNKNOWN) : ddOutputSingleObj(key), val(defaultval) {}
			public:
				inline T& value() { return std::get<0>(val); }
				inline const T& value() const { return std::get<0>(val); }
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & boost::serialization::make_nvp(
						"base",
						boost::serialization::base_object<rtmath::ddscat::ddOutputSingleKeys::hasMultipleValues<T> >(*this));
				}
			};

			/**
			* \param T is the underlying data storage type
			* \todo Specialize as part of hasMultipleValues
			**/
			/*template <class T>
			class hasValue : public virtual ddOutputSingleObj
			{
			public:
				hasValue(mapKeys key = mapKeys::UNKNOWN) : ddOutputSingleObj(key) {}
				hasValue(mapKeys key, const T& defaultval) : ddOutputSingleObj(key), val(defaultval) {}
			public:
				template <class U = std::string>
				inline U value() const
				{
					return boost::lexical_cast<U>(val);
				}
				template<> inline T value() const { return val; }
				template <class U = std::string>
				void value(const U &newval) { val = boost::lexical_cast<T>(val); }
				template<> void value(const T &newval) { val = newval; }
			protected:
				T val;
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & boost::serialization::make_nvp(
						"base",
						boost::serialization::base_object<rtmath::ddscat::ddOutputSingleObj>(*this));
					ar & boost::serialization::make_nvp("value", val);
				}
			};
			*/


			class SHARED_INTERNAL ddver
				: public hasValue<size_t>
			{
			public:
				ddver();
				virtual ~ddver() {}
				virtual void write(std::ostream &out, size_t) const override;
				virtual void read(std::istream &in) override;
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version);
			};

			class SHARED_INTERNAL ddstring
				: public hasValue<std::string>
			{
			public:
				ddstring() {}
				virtual ~ddstring() {}
				virtual void write(std::ostream &out, size_t) const override;
				virtual void read(std::istream &in) override;
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version);
			};
			
			class SHARED_INTERNAL ddtarget
				: public hasValue<std::string>
			{
			public:
				ddtarget() {}
				virtual ~ddtarget() {}
				virtual void write(std::ostream &out, size_t) const override;
				virtual void read(std::istream &in) override;
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version);
			};

			class SHARED_INTERNAL ddSval
				: public hasMultipleValues<std::string, std::string>
			{
			public:
				ddSval(); // (const std::string &tail = "") { this->tail = tail; }
				ddSval(const std::string &head, const std::string &tail);
				virtual ~ddSval() {}
				virtual void write(std::ostream &out, size_t) const override;
				virtual void read(std::istream &in) override;

				
				template <typename... Args>
				auto head(Args&&... args) -> decltype(value<0>(std::forward<Args>(args)...)) {
					return value<0>(std::forward<Args>(args)...); };
				
				template <typename... Args>
				auto tail(Args&&... args) -> decltype(value<1>(std::forward<Args>(args)...)) {
					return value<1>(std::forward<Args>(args)...); };
				
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version);
			};

			class SHARED_INTERNAL ddM
				: public hasMultipleValues<std::complex<double>,
				std::complex<double>, float, size_t >
			{
			public:
				ddM();
				virtual ~ddM() {}
				virtual void write(std::ostream &out, size_t) const override;
				virtual void read(std::istream &in) override;
				std::complex<double>& M() { return std::get<0>(val); }
				std::complex<double>& Eps() { return std::get<1>(val); }
				const std::complex<double>& M() const { return std::get<0>(val); }
				const std::complex<double>& Eps() const { return std::get<1>(val); }
				float& Mkd() { return std::get<2>(val); }
				const float& Mkd() const { return std::get<2>(val); }
				size_t& Substance() { return std::get<3>(val); }
				const size_t& Substance() const { return std::get<3>(val); }
			private:
				const std::size_t w;
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version);
			};
			

			template <class T>
			class SHARED_INTERNAL ddNval
				: public hasMultipleValues<size_t, T, std::string, std::string>
			{
			public:
				dNval(size_t pos = 0, const std::string &head = "", const std::string &tail = "");
				virtual ~ddNval() {}
				virtual void write(std::ostream &out, size_t) const override;
				virtual void read(std::istream &in) override;

				template <typename... Args>
				auto pos(Args&&... args) -> decltype(value<0>(std::forward<Args>(args)...)) {
					return value<0>(std::forward<Args>(args)...);
				};

				/// Confusing - refers to read value
				template <typename... Args>
				auto Nval(Args&&... args) -> decltype(value<1>(std::forward<Args>(args)...)) {
					return value<1>(std::forward<Args>(args)...);
				};

				template <typename... Args>
				auto head(Args&&... args) -> decltype(value<2>(std::forward<Args>(args)...)) {
					return value<2>(std::forward<Args>(args)...);
				};

				template <typename... Args>
				auto tail(Args&&... args) -> decltype(value<3>(std::forward<Args>(args)...)) {
					return value<3>(std::forward<Args>(args)...);
				};

				

			private:
				friend class boost::serialization::access;
				/** Needs to be in header due to MSVC 2012 nested template bug. **/
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & boost::serialization::make_nvp(
						"base",
						boost::serialization::base_object<
						hasMultipleValues<size_t, T, std::string, std::string > >(*this));
				}
			};

			enum class frameType { LF, TF };

			class SHARED_INTERNAL ddPolVec
				: public hasMultipleValues<std::complex<double>,
				std::complex<double>, std::complex<double>, 
				frameType, size_t >
			{
			public:
				ddPolVec();
				virtual ~ddPolVec() {}
				virtual void write(std::ostream &out, size_t) const override;
				virtual void read(std::istream &in) override;
				frameType& Frame() { return std::get<3>(val); }
				const frameType& Frame() const { return std::get<3>(val); }
				size_t& VecNum() { return std::get<4>(val); }
				const size_t& VecNum() const { return std::get<4>(val); }
				template<size_t i>
				std::complex<double>& Pol() { return std::get<i>(val); }
				template<size_t i>
				const std::complex<double>& Pol() const { return std::get<i>(val); }
			protected:
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


BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::ddOutputSingleObj);
BOOST_SERIALIZATION_ASSUME_ABSTRACT(rtmath::ddscat::ddOutputSingleKeys::ddOutputSingleObj);

BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::hasValue<size_t>);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::hasValue<std::string>);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::hasMultipleValues<std::string, std::string>);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::hasMultipleValues<std::complex<double>, std::complex<double>, float, size_t>);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::hasMultipleValues<size_t, size_t, std::string, std::string>);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::hasMultipleValues<size_t, double, std::string, std::string>);

BOOST_SERIALIZATION_ASSUME_ABSTRACT(rtmath::ddscat::ddOutputSingleKeys::hasValue<size_t>);
BOOST_SERIALIZATION_ASSUME_ABSTRACT(rtmath::ddscat::ddOutputSingleKeys::hasValue<std::string>);
BOOST_SERIALIZATION_ASSUME_ABSTRACT(rtmath::ddscat::ddOutputSingleKeys::hasMultipleValues<std::string, std::string>);
BOOST_SERIALIZATION_ASSUME_ABSTRACT(rtmath::ddscat::ddOutputSingleKeys::hasMultipleValues<std::complex<double>, std::complex<double>, float, size_t>);
BOOST_SERIALIZATION_ASSUME_ABSTRACT(rtmath::ddscat::ddOutputSingleKeys::hasMultipleValues<size_t, size_t, std::string, std::string>);
BOOST_SERIALIZATION_ASSUME_ABSTRACT(rtmath::ddscat::ddOutputSingleKeys::hasMultipleValues<size_t, double, std::string, std::string>);

BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::ddver);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::ddstring);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::ddtarget);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::ddSval);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::ddNval<size_t>);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::ddNval<double>);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::ddM);
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::ddOutputSingleKeys::ddPolVec);
