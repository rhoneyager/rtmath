#include "Stdafx-core.h"
#include <iostream>
#include <sstream>
#include "../rtmath/error/error.h"


namespace rtmath {
	namespace debug {
		void SHARED_PRIVATE (*xError::_errHandlerFunc)(const char*) = NULL;

		xError::xError() throw()
		{
			using namespace rtmath::debug::memcheck;
			file = __file__;
			line = (int) __line__;
			caller = __caller__;
		}
		
		bool xError::hasLoc() const
		{
			if (file == 0) return false;
			return true;
		}

		xError::~xError() throw()
		{
		}

		void xError::message(std::string &message) const throw()
		{
			std::ostringstream out;
			if (_message.size())
			{
				out << "\n" << _message;
			} else {
				out << "\nUnknown Error" << std::endl;
			}
			if (caller)
			{
				out << "File: " << file << std::endl;
				out << "Line: " << line << std::endl;
				out << "Caller: " << caller << std::endl;
			}
			message = out.str();
		}

		void xError::Display(std::ostream &out) const throw()
		{
			std::string sout;
			this->message(sout);
			if (_errHandlerFunc)
			{
				// Convert to const char array and send to function
				_errHandlerFunc(sout.c_str());
			} else {
				// Send to std. err.
				out << sout;
			}
		}

		const char* xError::what() const throw()
		{
			std::ostringstream out;
			Display(out);
			std::string w = out.str();
			return w.c_str();
		}

		void xError::setHandler(void (*func)(const char*))
		{
			_errHandlerFunc = func;
		}

		void xAssert::_setmessage()
		{
			_message = "ERROR: Assertion failed!\n Assertion is: ";
			_message.append(_m);
			_message.append("\n");
		}

		void xBadInput::_setmessage()
		{
			_message = "ERROR: The values passed to this function did not make sense.\n";
			_message += _m;
			_message.append("\n");
		}

		void xModelOutOfRange::_setmessage()
		{
			std::ostringstream out;

			out << "ERROR: The model (typically for optical depth) was called for a ";
			out << "value outside of the expected domain of the function. The ";
			out << "results would be nonsensical.\n";
			out << "The value provided was " << _m << ". Consult the rest of the ";
			out << "error text to determine the function involved.\n";
			_message = out.str();
		}

		void xMissingFrequency::_setmessage()
		{
			std::ostringstream out;

			out << "ERROR: The ddscat data loaded does not contain information ";
			out << "for the given frequency.\n";
			out << "The frequency requested was " << _m << ". Consult the rest of the ";
			out << "error text to determine the function involved.\n";
			_message = out.str();
		}

		void xEmptyInputFile::_setmessage()
		{
			_message = "ERROR: Reading an empty input file. \nInput file: ";
			_message.append(_m);
			_message.append("\n");
		}

		void xFileExists::_setmessage()
		{
			_message = "ERROR: Creating a file, but it already exists. \nInput file: ";
			_message.append(_m);
			_message.append("\n");
		}

		void xPathExistsWrongType::_setmessage()
		{
			_message = "ERROR: The requested path already exists and is the wrong type.\nPath: ";
			_message.append(_m);
			_message.append("\n");
		}

		void xUnknownFileFormat::_setmessage()
		{
			_message = "ERROR: File format unknown or unexpected. \nInput file: ";
			_message.append(_m);
			_message.append("\n");
		}

		void xUnsupportedIOaction::_setmessage()
		{
			_message = "ERROR: IO operation unsupported. \nInput file: ";
			_message.append(_m);
			_message.append("\n");
		}

		void xUnimplementedFunction::_setmessage()
		{
			_message = "ERROR: Unimplemented function\n";
		}

		void xArrayOutOfBounds::_setmessage()
		{
			_message = "ERROR: A referenced array went out of bounds.\n";
			_message.append("Please examine the source code at the error region.\n");
		}

		void xLockedNotInCache::_setmessage()
		{
			_message = "ERROR: An attempt was made to call eval() on a locked damatrix.\n";
			_message.append("The evalparams were not found in the cache, so eval() cannot succeed.\n");
		}

		void xObsolete::_setmessage()
		{
			_message = "ERROR: This function is obsolete.\n";
			_message.append("Consult code comments and documentation.\n");
		}

		void xDefective::_setmessage()
		{
			_message = "ERROR: This class is defective.\n";
			_message.append("Consult code comments and documentation to fix known bugs.\n");
		}

		void xSingular::_setmessage()
		{
			_message = "ERROR: Singular matrix detected.\n";
		}

		void xMissingFolder::_setmessage()
		{
			_message = "ERROR: Missing folder: ";
			_message.append(_m);
			_message.append("\n");
		}

		void xMissingFile::_setmessage()
		{
			_message = "ERROR: Missing file: ";
			_message.append(_m);
			_message.append("\n");
		}

		void xDuplicateHook::_setmessage()
		{
			_message = "ERROR: duplicate hook detected: ";
			_message.append(_m);
			_message.append("\n");
		}

		void xHandleInUse::_setmessage()
		{
			_message = "ERROR: handle is already in use: ";
			_message.append(_m);
			_message.append("\n");
		}

		void xHandleNotOpen::_setmessage()
		{
			_message = "ERROR: handle is not open: ";
			_message.append(_m);
			_message.append("\n");
		}

		void xSymbolNotFound::_setmessage()
		{
			_message = "ERROR: symbol not found in DLL: ";
			_message.append(_m);
			_message.append("\n");
			_message.append(_n);
			_message.append("\n");
		}

		void xBadFunctionMap::_setmessage()
		{
			_message = "ERROR: DLL has an incorrect function map: ";
			_message.append(_m);
			_message.append("\n");
		}

		void xBadFunctionReturn::_setmessage()
		{
			_message = "ERROR: DLL has an incorrect function map (2): ";
			_message.append(_m);
			_message.append("\n");
		}

		void xBlockedHookLoad::_setmessage()
		{
			_message = "ERROR: another DLL blocked load: ";
			_message.append(_m);
			_message.append("\n");
			_message.append(_n);
			_message.append("\n");
		}

		void xBlockedHookUnload::_setmessage()
		{
			_message = "ERROR: another DLL blocked unload: ";
			_message.append(_m);
			_message.append("\n");
			_message.append(_n);
			_message.append("\n");
		}

		void xOtherError::_setmessage()
		{
			_message = "ERROR (unknown)\n";
		}


	};
};
