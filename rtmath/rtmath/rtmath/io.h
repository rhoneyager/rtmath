#pragma once
/** \brief Provedes a series of standardized reading and writing functions 
* for classes to use to ensure uniform semantics.
**/
#include <memory>
#include "registry.h"

namespace rtmath
{
	namespace io
	{
		template <class obj_class,
		class output_registry_class>
		class implementsStandardWriter
		{
		protected:
			bool autoCompress;
			implementsStandardWriter(bool autoCompress = false) : 
				autoCompress(autoCompress)
			{}

			/// This actually handles the template writing i/o. It can report the 
			/// success of the write to a calling parent class.
			bool baseWrite(const std::string &filename, const std::string &outtype) const
			{
				auto opts = rtmath::registry::options::generate();
				opts->setVal<bool>("autocompress", autoCompress);
				auto res = writeMulti(filename.c_str(), nullptr, filename.c_str(),
					outtype.c_str(), registry::IOhandler::IOtype::TRUNCATE, opts);
				if (res) return true;
				return false;
			}
		public:
			virtual ~implementsStandardWriter() {}

			void write(const std::string &filename, const std::string &outtype) const
			{
				baseWrite(filename, outtype);
			}

			std::shared_ptr<registry::IOhandler> writeMulti(
				const char* key,
				std::shared_ptr<registry::IOhandler> handle,
				const char* filename,
				const char* ctype = "",
				registry::IOhandler::IOtype accessType = registry::IOhandler::IOtype::TRUNCATE,
				std::shared_ptr<registry::options> opts = nullptr) const
			{
				// All of these objects can handle their own compression
				typename ::rtmath::registry::IO_class_registry<obj_class>::io_multi_type dllsaver = nullptr;
				// Process dll hooks first
				auto hooks = ::rtmath::registry::usesDLLregistry<output_registry_class,
					::rtmath::registry::IO_class_registry<obj_class> >::getHooks();
				for (const auto &hook : *hooks)
				{
					if (!hook.io_multi_matches) continue; // Sanity check
					if (!hook.io_multi_processor) continue; // Sanity check
					if (hook.io_multi_matches(filename, ctype, handle))
					{
						dllsaver = hook.io_multi_processor;
						break;
					}
				}
				if (dllsaver)
				{
					// Most of these types aren't compressible or implement their
					// own compression schemes. So, it's not handled at this level.
					return dllsaver(handle, filename, dynamic_cast<const obj_class*>(this), key, accessType);
				} else {
					// Cannot match a file type to save.
					// Should never occur.
					RTthrow debug::xUnknownFileFormat(filename);
				}
				return nullptr; // Should never be reached
			}


		};

	}
}
