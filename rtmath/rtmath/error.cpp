#include "Stdafx.h"
#include <iostream>
#include "error.h"
#include <sstream>
#include "debug_mem.h"

namespace rtmath {
	namespace debug {
		void (*xError::_errHandlerFunc)(const char*) = NULL;

		xError::xError()
		{
			using namespace rtmath::debug::memcheck;
			file = __file__;
			line = (int) __line__;
			caller = __caller__;
		}

		xError::~xError()
		{
		}

		obsoleted::obsoleted()
		{
			throw rtmath::debug::obsoleted();
		}

		defective::defective()
		{
			throw rtmath::debug::defective();
		}

		void xError::message(std::string &message) const 
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

		void xError::Display() const
		{
			std::string out;
			this->message(out);
			if (_errHandlerFunc)
			{
				// Convert to const char array and send to function
				_errHandlerFunc(out.c_str());
			} else {
				// Send to std. err.
				std::cerr << out;
			}
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
			_message.append("TODO: give a more precise error here.\n");
		}

		void xEmptyInputFile::_setmessage()
		{
			_message = "ERROR: Reading an empty input file. \nInput file: ";
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
			_message = "ERROR: singular matrix detected.\n";
		}
		}

	};
};
