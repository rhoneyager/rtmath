#pragma once


namespace rtmath
{
	namespace ddscat {
		/// Parsers for ddscat files
		namespace ddOriDataParsers {


			struct version
			{
				/// \note Version is specified internally, and defaults to the latest version.
				static void write(std::ostream &out, size_t v);
				static size_t read(std::istream &in, size_t);
			};
			struct simpleString
			{
				static void write(std::ostream &out, size_t, const std::string &s, const std::string &p);
				static void read(std::istream &in, std::string &s);
			};
			struct simpleStringRev
			{
				static void write(std::ostream &out, size_t, const std::string &s, const std::string &p);
				static void read(std::istream &in, std::string &s);
			};

			template <class T>
			struct simpleNumRev
			{
				static void write(std::ostream &out, size_t, const T &s, const std::string &p, size_t pwd = 2, size_t wd = 10)
				{
					std::string sp(" ", pwd);
					out << sp;
					out.width(wd);
					out << std::left << s << " = ";
					out << p << std::endl;
				}
				static void read(std::istream &in, T &s)
				{
					std::string lin;
					std::getline(in, lin);
					size_t p = lin.find("=");
					std::string ss;
					ss = lin.substr(0, p - 1);
					// Remove any leading and lagging spaces
					// Not all Liu avg files are correct in this respect
					boost::algorithm::trim(ss);
					using namespace rtmath::macros;
					s = fastCast<T>(ss);
				}
			};

			template <class T>
			struct simpleNumCompound
			{
				static void write(std::ostream &out, size_t, const T &val, size_t wd,
					const std::string &pre, const std::string &post)
				{
					out << pre;
					out.width(wd);
					out << std::right << val << " = ";
					out << post << std::endl;
				}
				static void read(std::istream &in, T &s)
				{
					std::string lin;
					std::getline(in, lin);
					size_t p = lin.find("=");
					size_t pend = lin.find("=", p + 1);
					std::string ss;
					ss = lin.substr(p + 1, pend - p);
					// Remove any leading and lagging spaces
					// Not all Liu avg files are correct in this respect
					boost::algorithm::trim(ss);
					using namespace rtmath::macros;
					s = fastCast<T>(ss);
				}
			};

			extern template struct simpleNumRev < double >;
			extern template struct simpleNumRev < size_t >;

			extern template struct simpleNumCompound < double >;
			extern template struct simpleNumCompound < size_t >;

			struct refractive
			{
				static void write(std::ostream &out, size_t ver, size_t inum, 
					const std::complex<double> &m, double k, double d);
				static void read(std::istream &in, size_t &subst, std::complex<double> &m);
			};

			struct ddRot1d
			{
				static void write(std::ostream &out, size_t, const std::string &fieldname,
					double min, double max, size_t n, const std::string &fieldnamecaps);
				static void read(std::istream &in, std::string &fieldname, 
					double &min, double &max, size_t &n);
			};

			enum class frameType { LF, TF };
			struct ddPolVec
			{
				static void write(std::ostream &out, size_t ver, 
					const std::vector<std::complex<double> > &pols, size_t vecnum, frameType frame);
				static void read(std::istream &in, std::vector<std::complex<double> > &pols, 
					size_t &vecnum, frameType &frame);
			};

			struct ddAxisVec
			{
				static void write(std::ostream &out, size_t, const std::vector<double > &v,
					size_t axisnum, frameType frame);
				static void read(std::istream &in, std::vector<double> &v, size_t &axisnum, frameType &frame);
			};
			

		}
	}
}
