#include <iostream>
#include <sstream>
#include "../Ryan_Debug/debug.h"
#include "../Ryan_Debug/error.h"

namespace {
}
namespace Ryan_Debug {
	namespace error {

		xError::xError() throw() : inWhat(false) {}
		xError::~xError() throw() {}
		const char* xError::getErrLbl() const { return errLbl.c_str(); }
		const char* xError::getErrText() const { return errText.c_str(); }
		void xError::setErrLbl(const char* lbl) { errLbl = std::string(lbl); }
		void xError::setErrText(const char* lbl) { errText = std::string(lbl); }
		const char* xError::what() const throw()
		{
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

