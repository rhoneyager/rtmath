#pragma once
/** \brief Provides a series of standardized reading and writing functions
* for classes to use to ensure uniform semantics.
**/
#include <string>
#include <set>
#include <thread>
#include <memory>
#include <mutex>
//#include <boost/bind.hpp>
//#include <boost/bind/protect.hpp>
#include <Ryan_Serialization/serialization.h>
#include "registry.h"
#include "plugin.h"
#include "defs.h"

namespace rtmath
{
	namespace io
	{
		/// Provides uniform access semantics for compressible text file reading and writing
		namespace TextFiles
		{
			/// Opaque pointer to hide boost stream internals
			class hSerialization;

			/// Handles serialization IO for various classes. Also works for compressible text files (ddpar, ...)
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

			template <class obj_class>
			void writeSerialization(const obj_class* obj, 
				std::ostream &out, 
				std::shared_ptr<rtmath::registry::IO_options> opts,
				const char* sname)
			{
				using namespace Ryan_Serialization;
				serialization_method sm = select_format(opts->filename());
				if (sm == serialization_method::XML)
					::Ryan_Serialization::write<obj_class, boost::archive::xml_oarchive>(*obj, out, sname);
				else if (sm == serialization_method::TEXT)
					::Ryan_Serialization::write<obj_class, boost::archive::text_oarchive>(*obj, out, sname);
				else RTthrow debug::xUnknownFileFormat("Unknown serialization method");
			}

			template <class obj_class>
			void readSerialization(obj_class *obj, 
				std::istream &in, 
				std::shared_ptr<const rtmath::registry::IO_options> opts,
				const char* sname)
			{
				using namespace Ryan_Serialization;
				serialization_method sm = select_format(opts->filename());
				if (sm == serialization_method::XML)
					::Ryan_Serialization::read<obj_class, boost::archive::xml_iarchive>(*obj, in, sname);
				else if (sm == serialization_method::TEXT)
					::Ryan_Serialization::read<obj_class, boost::archive::text_iarchive>(*obj, in, sname);
				else RTthrow debug::xUnknownFileFormat("Unknown serialization method");

			}

			template <class obj_class>
			std::shared_ptr<rtmath::registry::IOhandler> writeFunc(
				std::shared_ptr<rtmath::registry::IOhandler> sh,
				std::shared_ptr<rtmath::registry::IO_options> opts,
				const obj_class *obj,
				const std::function<void(const obj_class*, std::ostream&, 
				std::shared_ptr<rtmath::registry::IO_options>)> &writer)
			{
				std::string exporttype = opts->exportType();
				std::string filename = opts->filename();
				registry::IOhandler::IOtype iotype = opts->iotype();
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
				writer(obj, *(h->writer.get()), opts); //, filename);
				
				return h; // Pass back the handle
			};

			template <class obj_class>
			std::shared_ptr<rtmath::registry::IOhandler> readFunc(
				std::shared_ptr<rtmath::registry::IOhandler> sh,
				std::shared_ptr<rtmath::registry::IO_options> opts,
				obj_class *obj,
				const std::function<void(obj_class*, std::istream&,
				std::shared_ptr<rtmath::registry::IO_options>)> &reader)
			{
				std::string exporttype = opts->exportType();
				std::string filename = opts->filename();
				registry::IOhandler::IOtype iotype = opts->getVal<registry::IOhandler::IOtype>
					("iotype", registry::IOhandler::IOtype::READONLY);
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
				// Read from a stream, not to a file. Filename is for serialization method detection.
				reader(obj, *(h->reader.get()), opts); //, filename);

				return h; // Pass back the handle
			};

		}

		/** \brief Template that registers reading and writing methods with the io registry
		 * 
		 * This is the base template that is used when implementing a custom reader/writer to the 
		 * core library code, such as the ddscat readers in ddPar, ddOutputSingle and shapefile.
		 * It is also leveraged in the serialization code.
		 * Any code that reads text files serially and would like optional compression can take advantage of this.
		 **/
		template <class obj_class,
		class output_registry_class,
		class input_registry_class>
		class implementsIO
		{
		public:
			virtual ~implementsIO() {}
		private:
			const std::set<std::string> &matchExts;
		protected:
			implementsIO(const std::set<std::string> &exts) : matchExts(exts)
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
			virtual void makeWriter(rtmath::registry::IO_class_registry_writer<obj_class> &writer) = 0;
			virtual void makeReader(rtmath::registry::IO_class_registry_reader<obj_class> &reader) = 0;

			virtual void setup()
			{
				using namespace registry;
				using namespace std;


				// Link the functions to the registry
				// Note: the standard genAndRegisterIOregistryPlural_writer will not work here, 
				// as function names would clash.

				// Custom matcher function will match all serialization-supported types!
				//const std::set<std::string> &exts = serialization_handle::known_formats();
				for (const auto &ext : matchExts)
				{
					// ! Generate writer
					using namespace std::placeholders;
					IO_class_registry_writer<obj_class> writer;
					makeWriter(writer);
					// ! Register writer
#ifdef _MSC_FULL_VER
					obj_class::usesDLLregistry<output_registry_class, IO_class_registry_writer<obj_class> >::registerHook(writer);
#else
					obj_class::template usesDLLregistry<output_registry_class, IO_class_registry_writer<obj_class> >::registerHook(writer);
#endif

					// ! Generate reader
					IO_class_registry_reader<obj_class> reader;
					makeReader(reader);
					// ! Register reader
#ifdef _MSC_FULL_VER
					obj_class::usesDLLregistry<input_registry_class, IO_class_registry_reader<obj_class> >::registerHook(reader);
#else
					obj_class::template usesDLLregistry<input_registry_class, IO_class_registry_reader<obj_class> >::registerHook(reader);
#endif

				}
			};
		};

		template <class obj_class,
		class output_registry_class,
		class input_registry_class>
		class implementsIObasic :
			protected implementsIO<obj_class, output_registry_class,
			input_registry_class>
		{
		public:
			virtual ~implementsIObasic() {}
		private:
			typedef const std::function<void(const obj_class*, std::ostream&)> outFunc;
			outFunc &outF;
			typedef const std::function<void(obj_class*, std::istream&)> inFunc;
			inFunc &inF;
		protected:
			implementsIObasic(outFunc &outF, inFunc &inF, const std::set<std::string> &exts) : 
				outF(outF), inF(inF) , implementsIO<obj_class, output_registry_class,
				input_registry_class>(exts) {}
			//{}
			virtual void makeWriter(rtmath::registry::IO_class_registry_writer<obj_class> &writer)
			{
				writer.io_multi_matches = std::bind(
					rtmath::io::TextFiles::serialization_handle::match_file_type_multi,
					std::placeholders::_1, 
					rtmath::io::TextFiles::serialization_handle::getSHid(), std::placeholders::_2);
				auto writerBinder = [&](
					std::shared_ptr<rtmath::registry::IOhandler> sh,
					std::shared_ptr<rtmath::registry::IO_options> opts,
					const obj_class* obj) -> std::shared_ptr<rtmath::registry::IOhandler>
				{
					using namespace rtmath::registry;
					using namespace rtmath::io::TextFiles;
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
					outF(obj, *(h->writer.get()));

					return h; // Pass back the handle
				};
				writer.io_multi_processor = writerBinder; // std::bind(writerBinder, _1, _2, _3);
			}
			virtual void makeReader(rtmath::registry::IO_class_registry_reader<obj_class> &reader)
			{
				reader.io_multi_matches = std::bind(
					rtmath::io::TextFiles::serialization_handle::match_file_type_multi,
					std::placeholders::_1, rtmath::io::TextFiles::serialization_handle::getSHid(), 
					std::placeholders::_2);
				auto readerBinder = [&](
					std::shared_ptr<rtmath::registry::IOhandler> sh,
					std::shared_ptr<rtmath::registry::IO_options> opts,
					obj_class *obj) -> std::shared_ptr<rtmath::registry::IOhandler>
				{
					using namespace rtmath::registry;
					using namespace rtmath::io::TextFiles;
					std::string exporttype = opts->exportType();
					std::string filename = opts->filename();
					IOhandler::IOtype iotype = opts->getVal<IOhandler::IOtype>("iotype", IOhandler::IOtype::READONLY);
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
					// Read from a stream, not to a file. Filename is for serialization method detection.
					inF(obj, *(h->reader.get()));
					//reader(obj, *(h->reader.get()), filename);

					return h; // Pass back the handle
				};
				reader.io_multi_processor = readerBinder;
				//reader.io_multi_processor = std::bind(TextFiles::readFunc<obj_class>, _1, _2, _3,
				//	std::bind(TextFiles::readSerialization<obj_class>, _1, _2, _3, sname));
			}
		};

		namespace Serialization
		{
			
			/// Template that registers object serialization with the io registry.
			template <class obj_class,
			class output_registry_class,
			class input_registry_class>
			class implementsSerialization : 
				private implementsIO<obj_class, output_registry_class,
				input_registry_class>
			{
			public:
				virtual ~implementsSerialization() {}
			protected:
				/// \brief This function can be used in place of entering it in the constructor, 
				/// for compilers that don't support delegating constructors (to save on typing).
				void set_sname(const char* nname) { sname = nname; }
				const char* sname;
				/// \param sname is the serialization object key
				implementsSerialization(const char* sname = "") : sname(sname), 
					implementsIO<obj_class, output_registry_class,
					input_registry_class>(io::TextFiles::serialization_handle::known_formats())
				{}

			private:
				/** \brief Fixes a deficiency in the C++11 STL
				*
				* Unfortunately, std::bind is less capable than boost::bind in that it isn't as friendly with
				* function chaining, and there is no real equivalent for boost::protect in the C++11 standard.
				* I prefer using pure C++11 for the dll interface, so I have to define a template forwarder 
				* function that handles the read and write serialization operations. 
				*
				* \note This entire class is designed to be static.
				**/
				template <class T, class stream>
				struct chainedSerializer
				{
					typedef std::function<void(T*, stream&, std::shared_ptr<registry::IO_options>, const char*)> InnerFunc;
					typedef std::function<std::shared_ptr<registry::IOhandler>(std::shared_ptr<registry::IOhandler>, 
						std::shared_ptr<registry::IO_options>, T*, const InnerFunc&)> OuterFunc;
					
					typedef std::pair<InnerFunc, OuterFunc> funcs; // Doing this to stay under 5 terms in the bind

					static std::shared_ptr<registry::IOhandler> func(
						std::shared_ptr<registry::IOhandler> ioh , std::shared_ptr<registry::IO_options> ioo, T* obj,
						funcs f, const char* sname)
					{
						f.second(ioh, ioo, obj, std::bind(f.first,
							std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, sname));
					}
					
				private:
					chainedSerializer() {}
				};

				virtual void makeWriter(rtmath::registry::IO_class_registry_writer<obj_class> &writer)
				{
					writer.io_multi_matches = std::bind(TextFiles::serialization_handle::match_file_type_multi,
						std::placeholders::_1, TextFiles::serialization_handle::getSHid(), std::placeholders::_2);
					//const auto boundWriterFunc = boost::bind(TextFiles::writeSerialization<obj_class>, 
					//	std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, sname);
					//writer.io_multi_processor = std::bind(TextFiles::writeFunc<obj_class>, 
					//	std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
					//	boost::protect(boundWriterFunc));

					//static chainedSerializer<const obj_class, std::ostream> sout =
					//	chainedSerializer<const obj_class, std::ostream>::generate(
					//	TextFiles::writeSerialization<obj_class>,
					//	TextFiles::writeFunc<obj_class>
					//	);

					chainedSerializer<const obj_class, std::ostream>::funcs p(
						TextFiles::writeSerialization<obj_class>, TextFiles::writeFunc<obj_class>);

					writer.io_multi_processor = std::bind(chainedSerializer<const obj_class, std::ostream>::func,
						std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, 
						p, sname);
						
				}
				virtual void makeReader(rtmath::registry::IO_class_registry_reader<obj_class> &reader)
				{
					reader.io_multi_matches = std::bind(TextFiles::serialization_handle::match_file_type_multi,
						std::placeholders::_1, TextFiles::serialization_handle::getSHid(), std::placeholders::_2);
					//reader.io_multi_processor = writeFunc; // The lambda from above
					auto boundReaderFunc = std::bind(TextFiles::readSerialization<obj_class>, 
						std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, sname);
					//reader.io_multi_processor = std::bind(TextFiles::readFunc<obj_class>, 
					//	std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
					//	boundReaderFunc);
					//reader.io_multi_processor = write_file_type_multi<T>;
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

			/// Duplicate to avoid clashes and having to speify a full template name...
			virtual void writeFile(const std::string &filename, const std::string &outtype = "") const
			{
				baseWrite(filename, outtype);
			}

			virtual void write(const std::string &filename, const std::string &outtype = "") const
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
/*
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
				//opts->setVal<bool>("autocompress", autoCompress);
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
	*/
	}
}
