#pragma once
#include <QtCore/QString>
#include <QtCore/QByteArray>
#include <string>

namespace rtmath
{
	namespace qt
	{
		static inline std::string toUtf8(const QString& s) 
		{ 
			QByteArray sUtf8 = s.toUtf8(); 
			return std::string(sUtf8.constData(), sUtf8.size()); 
		}

	}
}

