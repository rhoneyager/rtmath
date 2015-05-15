#pragma once
#include "info.h"
#include "dlls.h"
#include "registry.h"

// When loading, Ryan_Debug::registry asks for symbol dllVer.
// dllVer is a function like void(Ryan_Debug::versioning::versionInfo&, void*, void*)
// First is the compiled Ryan_Debug version info.
// Second is rdcheck - the plugin-reported address &(Ryan_Debug::registry::dump_hook_table).
// Third is vfStart - the address of the successful initialization function.
//
// So, upon gcc_init or msvc_init, call a function that 

#define D_Ryan_Debug_start() extern "C" dllInitResult FORCE_DLEXPORT dllStart()
#define D_Ryan_Debug_validator() extern "C" void FORCE_DLEXPORT dllVer(Ryan_Debug::versioning::versionInfo& vf, void** rd) \
		{ \
		Ryan_Debug::versioning::genVersionInfo(vf); \
		*rd = &(Ryan_Debug_registry_register_dll); }
//#define gcc_init(x) void __attribute__((constructor)) plugin_gcc_init() { x(); }
//#define msvc_init(x) BOOL APIENTRY DllMain(HANDLE hModule, DWORD dwReason, LPVOID lpReserved) \
//{ if (dwReason == DLL_PROCESS_ATTACH) x(); return true; }

#ifndef _MSC_FULL_VER
#define Ryan_Debug_plugin_init(x) d_dllVer(x); //gcc_init();
#else
#define Ryan_Debug_plugin_init(x) d_dllVer(x); //msvc_init();
#endif

// Convenience functions to register with a class

namespace Ryan_Debug
{
	namespace registry
	{
		// These have to be kept consistent with registry.h
		template <class T>
		std::shared_ptr<Ryan_Debug::registry::IOhandler> write_file_type_multi
			(std::shared_ptr<Ryan_Debug::registry::IOhandler> sh, 
			std::shared_ptr<Ryan_Debug::registry::IO_options> opts,
			const boost::shared_ptr<const T > obj);

		template <class T>
		std::shared_ptr<Ryan_Debug::registry::IOhandler> read_file_type_multi
			(std::shared_ptr<Ryan_Debug::registry::IOhandler> sh, 
			std::shared_ptr<Ryan_Debug::registry::IO_options> opts,
			boost::shared_ptr<T > obj,
			std::shared_ptr<const Ryan_Debug::registry::collectionTyped<T> > filter);

		template <class T>
		std::shared_ptr<Ryan_Debug::registry::IOhandler> read_file_type_vector
			(std::shared_ptr<Ryan_Debug::registry::IOhandler> sh, 
			std::shared_ptr<Ryan_Debug::registry::IO_options> opts,
			std::vector<boost::shared_ptr<T> > &vec, 
			std::shared_ptr<const Ryan_Debug::registry::collectionTyped<T> > filter);

		/** \brief Template (designed to be overridable) that can be used to 
		 * force a different writing function for io.
		 *
		 * \param T is the class being written.
		 * \param IO_reg_class is the registry used. Used only in specialization to force 
		 * a custom write function with a different naming scheme (e.g. core library writes).
		 **/
		/*
		template <class T, class IO_reg_class>
		typename IO_class_registry_writer<T>::io_multi_type
			selectWriter()
		{
			return write_file_type_multi<T>;
		}
		*/

		/** \brief Convenience function for setting up the IO_class_registry_writer objects
		*
		* \param T is the object type to be written
		* \param U is the object multi-writer handler type
		* \param IO_reg_class is used ONLY for selecting the writer, to allow overrides. If not 
		* attempting to override, the parameter is unused.
		* \param extension is the extension of the file (hdf5, silo, tsv, ...)
		* \param writeMulti is an optional multi-type writer. If not specified, the writer has a default name that must be specialized.
		* \param custom_writeSingle is an optional single type writer that is not the custom one.
		**/
		template<class T> //, class IO_reg_class>
		IO_class_registry_writer<T>
			genIOregistry_writer(
			const char* extension,
			const char* pluginid,
			const char* exportType = "")
		{
			IO_class_registry_writer<T> res;
			//res.io_multi_matches = std::bind(match_file_type, _1, _2, extension);
			auto opts2 = IO_options::generate();
			opts2->extension(extension);
			opts2->exportType(exportType);
			res.io_multi_matches = std::bind(match_file_type_multi, std::placeholders::_1, pluginid, std::placeholders::_2, opts2);

			//res.io_multi_processor = writer;
			res.io_multi_processor = write_file_type_multi<T>;
			//res.io_multi_processor = selectWriter<T, IO_reg_class>();
			return res;
		}

		/// Encapsulates msvc / gcc differences in calling a base class template function
		template <class base, class reg, class obj>
		void doRegisterHook(const obj& res)
		{
#ifdef _MSC_FULL_VER
			base::usesDLLregistry<reg, obj >::registerHook(res);
#else
			base::template usesDLLregistry<reg, obj >::registerHook(res);
#endif
		}

		template <class T, class IO_reg_class>
		void genAndRegisterIOregistry_writer(
			const char* extension,
			const char* pluginid,
			const char* exportType = "")
		{
			auto res = genIOregistry_writer<T>(extension, pluginid, exportType);
			doRegisterHook<T, IO_reg_class, IO_class_registry_writer<T> >(res);
			/*
#ifdef _MSC_FULL_VER
			T::usesDLLregistry<IO_reg_class, IO_class_registry_writer<T> >::registerHook(res);
#else
			T::template usesDLLregistry<IO_reg_class, IO_class_registry_writer<T> >::registerHook(res);
#endif
			*/
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
				res.io_vector_processor = read_file_type_vector<T>;
				return res;
		}


		template <class T, class IO_reg_class>
		void genAndRegisterIOregistry_reader(
			const char* extension,
			const char* pluginid,
			const char* exportType = "")
		{
			auto res = genIOregistry_reader<T>(extension, pluginid, exportType);
			doRegisterHook<T, IO_reg_class, IO_class_registry_reader<T> >(res);
			/*
#ifdef _MSC_FULL_VER
			T::usesDLLregistry<IO_reg_class, IO_class_registry_reader<T> >::registerHook(res);
#else
			T::template usesDLLregistry<IO_reg_class, IO_class_registry_reader<T> >::registerHook(res);
#endif
			*/
		}


		template <class T, class IO_reg_class>
		void genAndRegisterIOregistryPlural_reader(
			size_t nExt,
			const char** extensions,
			const char* pluginid,
			const char* exportType = "")
		{
			for (size_t i = 0; i < nExt; ++i)
			{
				genAndRegisterIOregistry_reader<T, IO_reg_class>(extensions[i],
					pluginid, exportType);
			}
		}
	}
}



