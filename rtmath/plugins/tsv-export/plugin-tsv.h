#pragma once
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <iostream>
#include <memory>
#include <string>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/plugin.h"
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>


#define PLUGINID "B77570C4-C2D2-471C-B24A-A279061E363B"
#define PLUGINID_SHP2 "B77570C4-C2D2-471C-B24A-C475031C363A"
#define PLUGINID_ARS "A5421845-3006-420B-8839-0A3281E041A8"
#define PLUGINID_VORO "3EF27E71-3011-4AFE-98BD-63C0924F8FC8"
#define PLUGINID_VORO_SHP "2053b025-e1b5-478f-bef8-e67a7c2da81e"
#define PLUGINID_DDORI "41D4122C-7C50-4A72-AAFC-7DE8AB93FB33"
#define PLUGINID_DDISO "0F7B7781-28C3-46AB-9067-8835C408BF2D"
#define PLUGINID_DDISOSMALL "7895AD9E-F1FB-47ec-A43B-439A77093EEA"
#define PLUGINID_DDSTATS "85814CBB-41EE-4B26-8A05-31E494A7BC4C"
#define PLUGINID_IMAGE "DFB4AD7C-303F-4C44-91E5-0288A3302BA1"
#define PLUGINID_SACR_REFL "22CDCF7E-72FA-4F4E-BD61-73857EAC7DA9"

namespace rtmath {
	namespace ddscat {
		class ddOutput;
		class ddOutputSingle;
		class ddScattMatrix;
		namespace shapefile { class shapefile; }
		namespace stats { class shapeFileStats; }
	}
	namespace images {
		class image;
	}

	namespace plugins {
		namespace tsv {

			struct tsv_handle : public Ryan_Debug::registry::IOhandler
			{
				tsv_handle(const char* filename, IOtype t, const char* id) : IOhandler(id) {  }
				virtual ~tsv_handle() {}
				void open(const char* filename, IOtype t)
				{
					using namespace boost::filesystem;
					switch (t)
					{
					case IOtype::EXCLUSIVE:
					case IOtype::DEBUG:
					case IOtype::READONLY:
						RDthrow(Ryan_Debug::error::xOtherError());
						break;
					case IOtype::CREATE:
						if (exists(path(filename))) RDthrow(Ryan_Debug::error::xFileExists());
					case IOtype::TRUNCATE:
						file = std::shared_ptr<std::ofstream>(new std::ofstream(filename, std::ios_base::trunc));
						writeHeader();
						break;
					case IOtype::READWRITE:
					{
						bool e = false;
						if (exists(path(filename))) e = true;
						file = std::shared_ptr<std::ofstream>(new std::ofstream(filename, std::ios_base::app));
						if (!e) writeHeader(); // If the file had to be created, give it a header
					}
						break;
					}
				}
				virtual void writeHeader() = 0;
				std::shared_ptr<std::ofstream> file;
			};
		}
	}
}


