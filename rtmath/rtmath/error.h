/* error.h - handling routines for all errors that can be thrown */
#pragma once

#include "Stdafx.h"
#include <iostream>

namespace rtmath
{
	namespace debug
	{
		class xError
		{
			// This is the parent error class. Everything inherits from this.
		public:
			xError();
			virtual ~xError();
			virtual void message(std::string &message)
			{
				if (message.size())
				{
					message = _message;
				} else {
					message = "Unknown Error\n";
				}
				return;
			}
		protected:
			std::string _message;
		};

		class xUnimplementedFunction : public xError
		{
			// A function has not been defined. Always stops execution.
			xUnimplementedFunction(char* funcsig)
			{
				_message = "Unimplemented function: ";
				_message.append(funcsig);
			}
		};
	}; // end debug
}; // end rtmath