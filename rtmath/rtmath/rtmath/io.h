#pragma once
/** \brief Provides a series of standardized reading and writing functions
* for classes to use to ensure uniform semantics.
**/
#include <string>
#include <set>
#include <thread>
#include <memory>
#include <mutex>
#include <Ryan_Serialization/serialization.h>
#include "registry.h"
#include "plugin.h"
#include "defs.h"

namespace rtmath
{
	namespace io
	{
		namespace Serialization
		{
			/// Opaque pointer to hide boost stream internals
			class hSerialization;

			/// Handles serialization IO for various classes.
			struct DLEXPORT_rtmath_core serialization_handle : public rtmath::registry::IOhandler
			{
				serialization_handle(const char* filename, 
					rtmath::registry::IOhandler::IOtype t);
				virtual ~serialization_handle();
				void open(const char* filename, IOtype t);

				/// Separate definition from getID(), as this is static
				static const char* getSHid();

				/** \brief Match file type for serialized data
				* \param filename is the file name.
				* \param type is the file's extension (provided by the filename / saver).
				* \param op is the export operation (provided by the lib caller).
				**/
				static bool match_file_type(
					const char* filename,
					const char* type,
					const char* op = "");

				/** \brief Match file type for serialized data
				* \param h is and existing IO handler.
				* \param pluginid is the plugin identifier (fails if not the serialization id).
				* \param opts are the IO_oprions for the desired file read/write.
				**/
				static bool match_file_type_multi(
					std::shared_ptr<rtmath::registry::IOhandler> h,
					const char* pluginid,
					std::shared_ptr<rtmath::registry::IO_options> opts);

				std::shared_ptr<std::ostream> writer;
				std::shared_ptr<std::istream> reader;

				static const std::set<std::string>& known_formats();

			private:
				/// Load file (prepare input stream)
				void load(const char*);
				/// Create file (prepare output stream)
				void create(const char*);

				std::unique_ptr<hSerialization> h;
			};

			/// Template that registers object serialization with the io registry.
			template <class obj_class,
			class output_registry_class,
			class input_registry_class>
			class implementsSerialization
			{
			public:
				~implementsSerialization() {}
			protected:
				/// \brief This function can be used in place of entering it in the constructor, 
				/// for compilers that don't support delegating constructors (to save on typing).
				void set_sname(const char* nname) { sname = nname; }
				const char* sname;
				/// \param sname is the serialization object key
				implementsSerialization(const char* sname = "") : sname(sname)
				{
					// Call the binder code
					static bool inited = false; // No need for a static class def.
					static std::mutex mlock;
					// Prevent threading clashes
					{
						std::lock_guard<std::mutex> lck(mlock);
						if (!inited)
						{
							setup();
							inited = true;
						}
					}
				}

				void setup()
				{
					using namespace registry;
					// Automatically implement the actual io functions using lambdas!
					auto writeFunc = [&](
						shared_ptr<IOhandler> sh, 
						shared_ptr<IO_options> opts,
						const obj_class *obj)
						-> shared_ptr<IOhandler>
					{
						std::string exporttype = opts->exportType();
						std::string filename = opts->filename();
						IOhandler::IOtype iotype = opts->iotype();
						std::string key = opts->getVal<std::string>("key", "");
						using std::shared_ptr;
						
						std::shared_ptr<serialization_handle> h;
						if (!sh)
							h = std::shared_ptr<serialization_handle>(new serialization_handle(filename.c_str(), iotype));
						else {
							if (sh->getId() != std::string(serialization_handle::getSHid())) 
								RTthrow debug::xDuplicateHook("Bad passed plugin");
							h = std::dynamic_pointer_cast<serialization_handle>(sh);
						}

						// serialization_handle handles compression details

						// Write to a stream, not to a file
						Ryan_Serialization::serialization_method sm = Ryan_Serialization::select_format(filename);
						if (sm == serialization_method::XML)
							::Ryan_Serialization::write<obj_class, boost::archive::xml_oarchive>(obj, h->writer, sname);
						else if (sm == serialization_method::TEXT)
							::Ryan_Serialization::write<obj_class, boost::archive::text_oarchive>(obj, h->writer, sname);
						else RTthrow debug::xUnknownFileFormat("Unknown serialization method");

						return h; // Pass back the handle
					};

					// Link the functions to the registry
					// Note: the standard genAndRegisterIOregistryPlural_writer will not work here, 
					// as function names would clash.

					// Custom matcher function will match all serialization-supported types!
					const std::set<std::string> &exts = serialization_handle::known_formats();
					for (const auto &ext : exts)
					{
						// ! Generate writer
						IO_class_registry_writer<obj_class> writer;
						writer.io_multi_matches = std::bind(serialization_handle::match_file_type_multi,
							std::placeholders::_1, serialization_handle::getSHid(), std::placeholders::_2);
						writer.io_multi_processor = writeFunc; // The lambda from above
						//writer.io_multi_processor = write_file_type_multi<T>;

						// ! Register writer
#ifdef _MSC_FULL_VER
						obj_class::usesDLLregistry<output_registry_class, IO_class_registry_writer<obj_class> >::registerHook(writer);
#else
						obj_class::template usesDLLregistry<output_registry_class, IO_class_registry_writer<obj_class> >::registerHook(writer);
#endif


						// ! Generate reader
						/*
						IO_class_registry_reader<obj_class> reader;
						reader.io_multi_matches = std::bind(serialization_handle::match_file_type_multi,
							std::placeholders::_1, serialization_handle::getSHid(), std::placeholders::_2);
						reader.io_multi_processor = readFunc; // The lambda from above

						// ! Register reader
#ifdef _MSC_FULL_VER
						obj_class::usesDLLregistry<input_registry_class, IO_class_registry_reader<obj_class> >::registerHook(reader);
#else
						obj_class::template usesDLLregistry<input_registry_class, IO_class_registry_reader<obj_class> >::registerHook(reader);
#endif
						*/
					}
				}
			};
		}

		template <class obj_class,
		class output_registry_class>
		class implementsStandardWriter
		{
		protected:
			/// \brief Variable controls if compression is used for certain recognized 
			/// file types (usually variants of text files). Can be turned off in 
			/// class constructor, and can be overridden in IO_options.
			bool autoCompress;
			// Controls whether boost::serialization can write this file.
			//bool serializable;
			implementsStandardWriter(bool autoCompress = true, bool canSerialize = false) :
				autoCompress(autoCompress)//, serializable(canSerialize)
			{}

			/// This actually handles the template writing i/o. It can report the 
			/// success of the write to a calling parent class.
			bool baseWrite(const std::string &filename, const std::string &outtype) const
			{
				auto opts = rtmath::registry::IO_options::generate();
				opts->setVal<bool>("autocompress", autoCompress);
				opts->filename(filename);
				opts->setVal("key", filename);
				registry::IOhandler::IOtype accessType = registry::IOhandler::IOtype::TRUNCATE;
				opts->iotype(accessType);
				opts->filetype(outtype);
				auto res = writeMulti(nullptr, opts);
				//auto res = writeMulti(filename.c_str(), nullptr, filename.c_str(),
				//	outtype.c_str(), registry::IOhandler::IOtype::TRUNCATE, opts);
				if (res) return true;
				return false;
			}
		public:
			virtual ~implementsStandardWriter() {}

			virtual void write(const std::string &filename, const std::string &outtype) const
			{
				baseWrite(filename, outtype);
			}

			std::shared_ptr<registry::IOhandler> writeMulti(
				std::shared_ptr<rtmath::registry::IOhandler> handle,
				std::shared_ptr<rtmath::registry::IO_options> opts
				) const
			{
				// All of these objects can handle their own compression
				typename ::rtmath::registry::IO_class_registry_writer<obj_class>::io_multi_type dllsaver = nullptr;
				// Process dll hooks first
				auto hooks = ::rtmath::registry::usesDLLregistry<output_registry_class,
					::rtmath::registry::IO_class_registry_writer<obj_class> >::getHooks();
				for (const auto &hook : *hooks)
				{
					if (!hook.io_multi_matches) continue; // Sanity check
					if (!hook.io_multi_processor) continue; // Sanity check
					//if (hook.io_multi_matches(filename, ctype, handle))
					if (hook.io_multi_matches(handle, opts))
					{
						dllsaver = hook.io_multi_processor;
						break;
					}
				}
				if (dllsaver)
				{
					// Most of these types aren't compressible or implement their
					// own compression schemes. So, it's not handled at this level.
					return dllsaver(handle, opts, dynamic_cast<const obj_class*>(this));
					//return dllsaver(handle, filename, dynamic_cast<const obj_class*>(this), key, accessType);
				}
				else {
					// Cannot match a file type to save.
					// Should never occur.
					RTthrow debug::xUnknownFileFormat(opts->filename().c_str());
				}
				return nullptr; // Should never be reached
			}

			bool canWriteMulti(
				std::shared_ptr<rtmath::registry::IOhandler> handle,
				std::shared_ptr<rtmath::registry::IO_options> opts
				) const
			{
				auto hooks = ::rtmath::registry::usesDLLregistry<output_registry_class,
					::rtmath::registry::IO_class_registry_writer<obj_class> >::getHooks();
				for (const auto &hook : *hooks)
				{
					if (!hook.io_multi_matches) continue; // Sanity check
					if (!hook.io_multi_processor) continue; // Sanity check
					if (hook.io_multi_matches(handle, opts))
					{
						return true;
					}
				}
				return false;
			}
		};

		template <class obj_class,
		class input_registry_class>
		class implementsStandardReader
		{
		protected:
			implementsStandardReader()
			{}

			/// This actually handles the template reading i/o. It can report the 
			/// success of the write to a calling parent class.
			bool baseRead(const std::string &filename, const std::string &outtype) const
			{
				auto opts = rtmath::registry::IO_options::generate();
				opts->setVal<bool>("autocompress", autoCompress);
				opts->filename(filename);
				opts->setVal("key", filename);
				registry::IOhandler::IOtype accessType = registry::IOhandler::IOtype::TRUNCATE;
				opts->iotype(accessType);
				opts->filetype(outtype);
				auto res = writeMulti(nullptr, opts);
				//auto res = writeMulti(filename.c_str(), nullptr, filename.c_str(),
				//	outtype.c_str(), registry::IOhandler::IOtype::TRUNCATE, opts);
				if (res) return true;
				return false;
			}
		public:
			virtual ~implementsStandardReader() {}

			virtual void read(const std::string &filename, const std::string &outtype) const
			{
				baseRead(filename, outtype);
			}

			std::shared_ptr<registry::IOhandler> readMulti(
				std::shared_ptr<rtmath::registry::IOhandler> handle,
				std::shared_ptr<rtmath::registry::IO_options> opts
				) const
			{
				// All of these objects can handle their own compression
				typename ::rtmath::registry::IO_class_registry_reader<obj_class>::io_multi_type dllsaver = nullptr;
				// Process dll hooks first
				auto hooks = ::rtmath::registry::usesDLLregistry<input_registry_class,
					::rtmath::registry::IO_class_registry_reader<obj_class> >::getHooks();
				for (const auto &hook : *hooks)
				{
					if (!hook.io_multi_matches) continue; // Sanity check
					if (!hook.io_multi_processor) continue; // Sanity check
					//if (hook.io_multi_matches(filename, ctype, handle))
					if (hook.io_multi_matches(handle, opts))
					{
						dllsaver = hook.io_multi_processor;
						break;
					}
				}
				if (dllsaver)
				{
					// Most of these types aren't compressible or implement their
					// own compression schemes. So, it's not handled at this level.
					return dllsaver(handle, opts, dynamic_cast<const obj_class*>(this));
					//return dllsaver(handle, filename, dynamic_cast<const obj_class*>(this), key, accessType);
				}
				else {
					// Cannot match a file type to save.
					// Should never occur.
					RTthrow debug::xUnknownFileFormat(opts->filename().c_str());
				}
				return nullptr; // Should never be reached
			}

			bool canReadMulti(
				std::shared_ptr<rtmath::registry::IOhandler> handle,
				std::shared_ptr<rtmath::registry::IO_options> opts
				) const
			{
				auto hooks = ::rtmath::registry::usesDLLregistry<input_registry_class,
					::rtmath::registry::IO_class_registry_reader<obj_class> >::getHooks();
				for (const auto &hook : *hooks)
				{
					if (!hook.io_multi_matches) continue; // Sanity check
					if (!hook.io_multi_processor) continue; // Sanity check
					if (hook.io_multi_matches(handle, opts))
					{
						return true;
					}
				}
				return false;
			}
		};
	}
}
