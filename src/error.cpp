#include <iostream>
#include <sstream>
#include "../Ryan_Debug/error.h"

namespace {
}
namespace Ryan_Debug {
	namespace error {

		xError::xError() throw() {}
		xError::~xError() throw() {}
		const char* xError::what() const throw()
		{
			std::ostringstream out;
			out << "Unclassified error: ";
			out << std::endl;
			errText = out.str();
			addLineInfo();
			return errText.c_str();
		}

		void xError::addLineInfo() const throw()
		{
			std::ostringstream out;
			if( std::string const * mi=boost::get_error_info<source_file_name>(*this) )
				out << " source file: " << *mi << std::endl;
			if( int const * mi=boost::get_error_info<source_file_line>(*this) )
				out << " source line: " << *mi << std::endl;
			if( std::string const * mi=boost::get_error_info<source_func_sig>(*this) )
				out << " function: " << *mi << std::endl;
			errText.append(out.str());
		}

		const char* xDivByZero::what() const throw()
		{
			std::ostringstream out;
			out << "Error: Divide by zero encountered\n";
			if( std::string const * mi=boost::get_error_info<split_range_name>(*this) )
				std::cerr << " statement: " << *mi << std::endl;
			errText = out.str();
			addLineInfo();
			return errText.c_str();
		}

		const char* xInvalidRange::what() const throw()
		{
			std::ostringstream out;
			out << "Error: Invalid range\n";
			if( std::string const * mi=boost::get_error_info<split_range_name>(*this) )
				std::cerr << " statement: " << *mi << std::endl;
			if( std::string const * mi=boost::get_error_info<specializer_type>(*this) )
				std::cerr << " specializer: " << *mi << std::endl;
			errText = out.str();

			addLineInfo();
			return errText.c_str();
		}


	}
}

