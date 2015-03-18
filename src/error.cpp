#include <iostream>
#include <sstream>
#include "../Ryan_Debug/error.h"

namespace {
}
namespace Ryan_Debug {
	namespace error {

		xError::xError() throw() : inWhat(false) {}
		xError::~xError() throw() {}
		const char* xError::what() const throw()
		{
			/*
			if (errText.size() == 0) {
			struct g { 
				bool &b; 
				~g() { 
					b = false; 
				} 
				} guard{inWhat};

				if (inWhat) {
					static const char unc[] = "Unclassified error";
					if (errLbl.size()) return errLbl.c_str();
					else return unc;
				} else {
					inWhat = true;
					errText += boost::diagnostic_information(*this, false);
				}
			}
			return errText.c_str();
			*/
			std::ostringstream out;
			if (errLbl.size()) out << "\n\nError: " << errLbl;
			else out << "\n\nUnclassified error: ";
			out << std::endl;

			out << boost::diagnostic_information_what(*this, true) << std::endl;
			errText = out.str();
			return errText.c_str();
		}

	}
}

