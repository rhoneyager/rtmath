#pragma once

#include "registry.h"


#define gcc_init(x) void __attribute__((constructor)) plugin_gcc_init() { x(); }
#define msvc_init(x) BOOL APIENTRY DllMain(HANDLE hModule, DWORD dwReason, LPVOID lpReserved) \
{ if (dwReason == DLL_PROCESS_ATTACH) x(); return true; }

#ifndef _MSC_FULL_VER
#define rtmath_plugin_init(x) gcc_init(x)
#else
#define rtmath_plugin_init(x) msvc_init(x)
#endif

#ifdef _WIN32
#include "windows.h"
typedef HINSTANCE dlHandleType;
#endif
#ifdef __unix__
#include "dlfcn.h"
typedef void* dlHandleType;
#endif

// Convenience functions to register with a class

namespace rtmath
{
	namespace registry
	{
		template <class T>
		std::shared_ptr<rtmath::registry::IOhandler> write_file_type_multi
			(std::shared_ptr<rtmath::registry::IOhandler> sh, 
			std::shared_ptr<rtmath::registry::IO_options> opts,
			const T *obj);

		template <class T>
		std::shared_ptr<rtmath::registry::IOhandler> read_file_type_multi
			(std::shared_ptr<rtmath::registry::IOhandler> sh, 
			std::shared_ptr<rtmath::registry::IO_options> opts,
			std::vector<boost::shared_ptr<T> > &vec);

		/** \brief Convenience function for setting up the IO_class_registry_writer objects
		*
		* \param T is the object type to be written
		* \param U is the object multi-writer handler type
		* \param extension is the extension of the file (hdf5, silo, tsv, ...)
		* \param writeMulti is an optional multi-type writer. If not specified, the writer has a default name that must be specialized.
		* \param custom_writeSingle is an optional single type writer that is not the custom one.
		**/
		template<class T>
		IO_class_registry_writer<T>
			genIOregistry_writer(
			const char* extension,
			const char* pluginid,
			const char* exportType = "")
		{
			IO_class_registry_writer<T> res;
			//res.io_matches = std::bind(match_file_type, _1, _2, extension);
			auto opts2 = IO_options::generate();
			opts2->extension(extension);
			opts2->exportType(exportType);
			res.io_multi_matches = std::bind(match_file_type_multi, std::placeholders::_1, pluginid, std::placeholders::_2, opts2);

			res.io_multi_processor = write_file_type_multi<T>;
			return res;
		}

		/** \brief Convenience function for setting up the IO_class_registry_writer objects
		*
		* \param T is the object type to be written
		* \param U is the object multi-writer handler type
		* \param extension is the extension of the file (hdf5, silo, tsv, ...)
		* \param writeMulti is an optional multi-type writer. If not specified, the writer has a default name that must be specialized.
		* \param custom_writeSingle is an optional single type writer that is not the custom one.
		**/
		template<class T>
		IO_class_registry_reader<T>
			genIOregistry_reader(
			const char* extension,
			const char* pluginid,
			const char* exportType = "")
		{
			IO_class_registry_reader<T> res;
			//res.io_matches = std::bind(match_file_type, _1, _2, extension);
			auto opts2 = IO_options::generate();
			opts2->extension(extension);
			opts2->exportType(exportType);
			res.io_multi_matches = std::bind(match_file_type_multi, std::placeholders::_1, pluginid, std::placeholders::_2, opts2);

			res.io_multi_processor = read_file_type_multi<T>;
			return res;
		}

		template <class T, class IO_reg_class>
		void genAndRegisterIOregistry_writer(
			const char* extension,
			const char* pluginid,
			const char* exportType = "")
		{
			auto res = genIOregistry_writer<T>(extension, pluginid, exportType);
#ifdef _MSC_FULL_VER
			T::usesDLLregistry<IO_reg_class, IO_class_registry_writer<T> >::registerHook(res);
#else
			T::template usesDLLregistry<IO_reg_class, IO_class_registry_writer<T> >::registerHook(res);
#endif
		}

		template <class T, class IO_reg_class>
		void genAndRegisterIOregistry_reader(
			const char* extension,
			const char* pluginid,
			const char* exportType = "")
		{
			auto res = genIOregistry_reader<T>(extension, pluginid, exportType);
#ifdef _MSC_FULL_VER
			T::usesDLLregistry<IO_reg_class, IO_class_registry_reader<T> >::registerHook(res);
#else
			T::template usesDLLregistry<IO_reg_class, IO_class_registry_reader<T> >::registerHook(res);
#endif
		}

		template <class T, class IO_reg_class>
		void genAndRegisterIOregistryPlural_writer(
			size_t nExt,
			const char** extensions,
			const char* pluginid,
			const char* exportType = "")
		{
			for (size_t i = 0; i < nExt; ++i)
			{
				genAndRegisterIOregistry_writer<T, IO_reg_class>(extensions[i],
					pluginid, exportType);
			}
		}

	}
}



