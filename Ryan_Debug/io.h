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
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/log/sources/global_logger_storage.hpp>
#if USE_RYAN_SERIALIZATION
#include <Ryan_Serialization/serialization.h>
#endif
#include "registry.h"
#include "plugin.h"
#include "defs.h"

namespace Ryan_Debug
{
	namespace io
	{
		BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
			m_io,
			blog::sources::severity_channel_logger_mt< >,
			(blog::keywords::severity = Ryan_Debug::debug::error)(blog::keywords::channel = "io"));

		/// Provides uniform access semantics for compressible text file reading and writing
		namespace TextFiles
		{
			/// Opaque pointer to hide boost stream internals
			class hSerialization;

			/// Handles serialization IO for various classes. Also works for compressible text files (ddpar, ...)
			struct RYAN_DEBUG_DLEXPORT serialization_handle : public Ryan_Debug::registry::IOhandler
			{
				serialization_handle(const char* filename,
					Ryan_Debug::registry::IOhandler::IOtype t);
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
					const std::set<std::string> &mtypes,
					const char* op = "");
				static bool match_file_type(
					const char* filename,
					const char* type,
					const char* op = "")
				{
					return match_file_type(filename, type, known_formats(), op);
				}

				/** \brief Match file type for serialized data
				* \param h is and existing IO handler.
				* \param pluginid is the plugin identifier (fails if not the serialization id).
				* \param opts are the IO_oprions for the desired file read/write.
				**/
				static bool match_file_type_multi(
					std::shared_ptr<Ryan_Debug::registry::IOhandler> h,
					const char* pluginid,
					std::shared_ptr<Ryan_Debug::registry::IO_options> opts, 
					const std::set<std::string> &mtypes);

				std::weak_ptr<std::ostream> writer;
				std::weak_ptr<std::istream> reader;

				/// Provides known extensions (xml, st * compression pairs) to match against
				static const std::set<std::string>& known_formats();

				/// Reports whether compression is enabled at all (master switch in io.cpp)
				static bool compressionEnabled();
			private:
				/// Load file (prepare input stream)
				void load(const char*);
				/// Create file (prepare output stream)
				void create(const char*);

				std::unique_ptr<hSerialization> h;
			};

#if USE_RYAN_SERIALIZATION
			template <class obj_class>
			void writeSerialization(const boost::shared_ptr<const obj_class> obj, 
				std::ostream &out, 
				std::shared_ptr<Ryan_Debug::registry::IO_options> opts,
				const char* sname)
			{
				using namespace Ryan_Serialization;
				serialization_method sm = select_format(opts->filename());
				if (sm == serialization_method::XML)
					::Ryan_Serialization::write<obj_class, boost::archive::xml_oarchive>(*obj, out, sname);
				else if (sm == serialization_method::TEXT)
					::Ryan_Serialization::write<obj_class, boost::archive::text_oarchive>(*obj, out, sname);
				else RDthrow(debug::xUnknownFileFormat())
					<< debug::otherErrorText("Unknown serialization method");
			}

			template <class obj_class>
			void readSerialization(boost::shared_ptr<obj_class> obj, 
				std::istream &in, 
				std::shared_ptr<const Ryan_Debug::registry::IO_options> opts,
				const char* sname)
			{
				using namespace Ryan_Serialization;
				serialization_method sm = select_format(opts->filename());
				if (sm == serialization_method::XML)
					::Ryan_Serialization::read<obj_class, boost::archive::xml_iarchive>(*(obj.get()), in, sname);
				else if (sm == serialization_method::TEXT)
					::Ryan_Serialization::read<obj_class, boost::archive::text_iarchive>(*(obj.get()), in, sname);
				else RDthrow(debug::xUnknownFileFormat())
					<< debug::otherErrorText("Unknown serialization method");

			}

#endif

			template <class obj_class>
			std::shared_ptr<Ryan_Debug::registry::IOhandler> writeFunc(
				std::shared_ptr<Ryan_Debug::registry::IOhandler> sh,
				std::shared_ptr<Ryan_Debug::registry::IO_options> opts,
				const boost::shared_ptr<const obj_class> obj,
				const std::function<void(const boost::shared_ptr<const obj_class>, std::ostream&,
				std::shared_ptr<Ryan_Debug::registry::IO_options>)> &writer)
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
						RDthrow(debug::xDuplicateHook())
						<< debug::otherErrorText("Passed plugin is "
						"the wrong one. It is not the serialization "
						"plugin.");
					h = std::dynamic_pointer_cast<serialization_handle>(sh);
				}

				// serialization_handle handles compression details
				// Write to a stream, not to a file
				writer(obj, *(h->writer.lock().get()), opts); //, filename);
				
				return h; // Pass back the handle
			};

			template <class obj_class>
			std::shared_ptr<Ryan_Debug::registry::IOhandler> readFunc(
				std::shared_ptr<Ryan_Debug::registry::IOhandler> sh,
				std::shared_ptr<Ryan_Debug::registry::IO_options> opts,
				boost::shared_ptr<obj_class> obj,
				const std::function<void(boost::shared_ptr<obj_class>, std::istream&,
				std::shared_ptr<Ryan_Debug::registry::IO_options>)> &reader)
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
						RDthrow(debug::xDuplicateHook())
						<< debug::otherErrorText("Bad passed plugin. "
						"It is not the serialization plugin.");
					h = std::dynamic_pointer_cast<serialization_handle>(sh);
				}

				// serialization_handle handles compression details
				// Read from a stream, not to a file. Filename is for serialization method detection.
				reader(obj, *(h->reader.lock().get()), opts); //, filename);

				return h; // Pass back the handle
			};

		}

		/// Exists to give the implementsIO template a uniform lock, preventing a race condition
		RYAN_DEBUG_DLEXPORT std::mutex&  getLock();

		/** \brief Template that registers reading and writing methods with the io registry
		 * 
		 * This is the base template that is used when implementing a custom reader/writer to the 
		 * core library code, such as the ddscat readers in ddPar, ddOutputSingle and shapefile.
		 * It is also leveraged in the serialization code.
		 * Any code that reads text files serially and would like optional compression can take advantage of this.
		 *
		 * \todo Split into input and output objects.
		 **/
		template <class obj_class,
		class output_registry_class,
		class input_registry_class,
		class obj_io_separator>
		class implementsIO
		{
		public:
			virtual ~implementsIO() {}
		protected:
			/// \note Cannot use a reference because it ruins the assignment operator
			/// \todo Add custom copy constructor / assignment operator
			const std::set<std::string> matchExts;
		protected:
			implementsIO(const std::set<std::string> &exts) : matchExts(exts) {}
			virtual void makeWriter(Ryan_Debug::registry::IO_class_registry_writer<obj_class> &writer) = 0;
			virtual void makeReader(Ryan_Debug::registry::IO_class_registry_reader<obj_class> &reader) = 0;

			virtual void doImplementsIOsetup()
			{
				// Call the binder code
				std::mutex &mlock = getLock();
				// Prevent threading clashes
				{
					std::lock_guard<std::mutex> lck(mlock);
					static bool inited = false; // No need for a static class def.
					if (!inited)
					{
						setup();
						inited = true;
					}
				}
			}
		private:
			virtual void setup()
			{
				auto& lg = Ryan_Debug::io::m_io::get();
				BOOST_LOG_SEV(lg, Ryan_Debug::debug::normal) << "Performing io setup\n";

				using namespace registry;
				using namespace std;

				static std::vector<IO_class_registry_writer<obj_class> > writers;
				static std::vector<IO_class_registry_reader<obj_class> > readers;

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

					BOOST_LOG_SEV(lg, Ryan_Debug::debug::normal) << "Creating & registering reader and writer for extension " << ext << "\n";
					makeWriter(writer);
					writers.push_back(std::move(writer));
					// ! Register writer
#ifdef _MSC_FULL_VER
					obj_class::usesDLLregistry<output_registry_class, IO_class_registry_writer<obj_class> >::registerHook(*(writers.rbegin()));
#else
					obj_class::template usesDLLregistry<output_registry_class, IO_class_registry_writer<obj_class> >::registerHook(*(writers.rbegin()));
#endif

					// ! Generate reader
					IO_class_registry_reader<obj_class> reader;
					makeReader(reader);
					readers.push_back(std::move(reader));
					// ! Register reader
#ifdef _MSC_FULL_VER
					obj_class::usesDLLregistry<input_registry_class, IO_class_registry_reader<obj_class> >::registerHook(*(readers.rbegin()));
#else
					obj_class::template usesDLLregistry<input_registry_class, IO_class_registry_reader<obj_class> >::registerHook(*(readers.rbegin()));
#endif

				}
			};
		};

		template <class obj_class,
		class output_registry_class,
		class input_registry_class,
		class obj_io_separator>
		class implementsIObasic :
			protected implementsIO<obj_class, output_registry_class,
			input_registry_class, obj_io_separator>
		{
		public:
			virtual ~implementsIObasic() {}
		private:
			typedef const std::function<void(const boost::shared_ptr<const obj_class>, std::ostream&, std::shared_ptr<Ryan_Debug::registry::IO_options>)> outFunc;
			outFunc &outF;
			typedef const std::function<void(boost::shared_ptr<obj_class>, std::istream&, std::shared_ptr<Ryan_Debug::registry::IO_options>)> inFunc;
			inFunc &inF;
		protected:
			implementsIObasic(outFunc &outF, inFunc &inF, const std::set<std::string> &exts) : 
				outF(outF), inF(inF), implementsIO<obj_class, output_registry_class,
				input_registry_class, obj_io_separator>(exts)
			{
				implementsIO<obj_class, output_registry_class,
					input_registry_class, obj_io_separator>::doImplementsIOsetup();
			}
			virtual void makeWriter(Ryan_Debug::registry::IO_class_registry_writer<obj_class> &writer)
			{
				auto& lg = Ryan_Debug::io::m_io::get();
				BOOST_LOG_SEV(lg, Ryan_Debug::debug::normal) << "Creating writer (shid " << Ryan_Debug::io::TextFiles::serialization_handle::getSHid()
					<< "), matching extensions : \n";
				for (const auto &e : this->matchExts)
					BOOST_LOG_SEV(lg, Ryan_Debug::debug::normal) << e << "\n";

				writer.io_multi_matches = std::bind(
					Ryan_Debug::io::TextFiles::serialization_handle::match_file_type_multi,
					std::placeholders::_1, 
					Ryan_Debug::io::TextFiles::serialization_handle::getSHid(), 
					std::placeholders::_2, this->matchExts);
				auto writerBinder = [&](
					std::shared_ptr<Ryan_Debug::registry::IOhandler> sh,
					std::shared_ptr<Ryan_Debug::registry::IO_options> opts,
					const boost::shared_ptr<const obj_class> obj, outFunc outF) -> std::shared_ptr<Ryan_Debug::registry::IOhandler>
				{
					using namespace Ryan_Debug::registry;
					using namespace Ryan_Debug::io::TextFiles;
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
							RDthrow(debug::xDuplicateHook())
							<< debug::otherErrorText("Bad passed plugin. It is not the serialization plugin.");
						h = std::dynamic_pointer_cast<serialization_handle>(sh);
					}

					// serialization_handle handles compression details
					// Write to a stream, not to a file
					outF(obj, *(h->writer.lock().get()), opts);

					return h; // Pass back the handle
				};
				//writer.io_multi_processor = std::move(writerBinder); // std::bind(writerBinder, _1, _2, _3);
				writer.io_multi_processor = std::bind(writerBinder,
					std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,
					outF);


			}
			virtual void makeReader(Ryan_Debug::registry::IO_class_registry_reader<obj_class> &reader)
			{
				auto& lg = Ryan_Debug::io::m_io::get();
				BOOST_LOG_SEV(lg, Ryan_Debug::debug::normal) << "Creating reader (shid " << Ryan_Debug::io::TextFiles::serialization_handle::getSHid()
					<< "), matching extensions : \n";
				for (const auto &e : this->matchExts)
					BOOST_LOG_SEV(lg, Ryan_Debug::debug::normal) << e << "\n";

				reader.io_multi_matches = std::bind(
					Ryan_Debug::io::TextFiles::serialization_handle::match_file_type_multi,
					std::placeholders::_1, Ryan_Debug::io::TextFiles::serialization_handle::getSHid(), 
					std::placeholders::_2, this->matchExts);
				auto readerBinder = [&](
					std::shared_ptr<Ryan_Debug::registry::IOhandler> sh,
					std::shared_ptr<Ryan_Debug::registry::IO_options> opts,
					boost::shared_ptr<obj_class> obj, inFunc inF) -> std::shared_ptr<Ryan_Debug::registry::IOhandler>
				{
					using namespace Ryan_Debug::registry;
					using namespace Ryan_Debug::io::TextFiles;
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
							RDthrow(debug::xDuplicateHook())
							<< debug::otherErrorText("Bad passed plugin. It is not the serialization plugin.");
						h = std::dynamic_pointer_cast<serialization_handle>(sh);
					}

					// serialization_handle handles compression details
					// Read from a stream, not to a file. Filename is for serialization method detection.
					inF(obj, *(h->reader.lock().get()), opts);
					//reader(obj, *(h->reader.get()), filename);

					return h; // Pass back the handle
				};
				reader.io_multi_processor = std::bind(readerBinder,
					std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,
					inF);

				// The vector reader gets ignored for the standard reader class. The standard file types 
				// get handled by the single object reader.
				/*
				auto vectorReaderBinder = [&](
					std::shared_ptr<Ryan_Debug::registry::IOhandler> sh,
					std::shared_ptr<Ryan_Debug::registry::IO_options> opts,
					std::vector<boost::shared_ptr<obj_class> > &obj, inFunc inF) 
					-> std::shared_ptr<Ryan_Debug::registry::IOhandler>
				{
					using namespace Ryan_Debug::registry;
					using namespace Ryan_Debug::io::TextFiles;
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
							RDthrow debug::xDuplicateHook("Bad passed plugin");
						h = std::dynamic_pointer_cast<serialization_handle>(sh);
					}

					// serialization_handle handles compression details
					// Read from a stream, not to a file. Filename is for serialization method detection.
					inF(obj, *(h->reader.lock().get()), opts); // CHANGE THIS
					//reader(obj, *(h->reader.get()), filename);

					return h; // Pass back the handle
				};
				reader.io_vector_processor = std::bind(vectorReaderBinder,
					std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
					inF);
				*/
			}
		};

#if USE_RYAN_SERIALIZATION
		namespace Serialization
		{
			
			/// Template that registers object serialization with the io registry.
			template <class obj_class,
			class output_registry_class,
			class input_registry_class,
			class obj_io_separator>
			class implementsSerialization : 
				private implementsIO<obj_class, output_registry_class,
				input_registry_class, obj_io_separator>
			{
			public:
				virtual ~implementsSerialization() {}
			protected:
				/// \brief This function can be used in place of entering it in the constructor, 
				/// for compilers that don't support delegating constructors (to save on typing).
				void set_sname(const char* nname) { sname = nname; }
				const char* get_sname() const { return sname; }
				const char* sname;
				/// \param sname is the serialization object key
				implementsSerialization(const char* sname = "") : sname(sname), 
					implementsIO<obj_class, output_registry_class,
					input_registry_class, obj_io_separator>(io::TextFiles::serialization_handle::known_formats())
				{
					implementsIO<obj_class, output_registry_class,
					input_registry_class, obj_io_separator>::doImplementsIOsetup();
				}

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
				template <class T, class stream, class registryType>
				struct chainedSerializer
				{
					typedef std::function<void(T*, stream&, std::shared_ptr<registry::IO_options>)> InnerFuncActual;
					typedef std::function<void(T*, stream&, std::shared_ptr<registry::IO_options>, const char*)> InnerFunc;
					typedef std::function<std::shared_ptr<registry::IOhandler>(std::shared_ptr<registry::IOhandler>, 
						std::shared_ptr<registry::IO_options>, T*, InnerFuncActual)> OuterFunc;

					struct params
					{
						InnerFunc i; 
						OuterFunc o;
						//const char* sname;
						params(InnerFunc i, OuterFunc o) : i(i), o(o) {}
					};

					static typename registryType::io_multi_type 
						genFunc(InnerFunc i, OuterFunc o)
					{
						using namespace Ryan_Debug::registry;
						typename registryType::io_multi_type res;
						auto resfunc = 
							[&](std::shared_ptr<IOhandler> ioh, std::shared_ptr<IO_options> ioo, 
								T* obj, const params& params) -> std::shared_ptr<IOhandler>
						{
							std::shared_ptr<IOhandler> a;
							a = params.o(ioh, ioo, obj, std::bind(params.i,
								std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
								obj->get_sname()));
								//params.sname));
							return a;
						};

						res = std::bind(resfunc, std::placeholders::_1, std::placeholders::_2, 
							std::placeholders::_3, params(i,o));
						return res;
					}

				private:
					chainedSerializer() {}
				};

			protected:
				virtual void makeWriter(Ryan_Debug::registry::IO_class_registry_writer<obj_class> &writer)
				{
					writer.io_multi_matches = std::bind(TextFiles::serialization_handle::match_file_type_multi,
						std::placeholders::_1, TextFiles::serialization_handle::getSHid(), 
						std::placeholders::_2, io::TextFiles::serialization_handle::known_formats());

					writer.io_multi_processor = chainedSerializer<const obj_class, std::ostream, 
						registry::IO_class_registry_writer<obj_class> >::genFunc(
						TextFiles::writeSerialization<obj_class>, TextFiles::writeFunc<obj_class>);
				}
				virtual void makeReader(Ryan_Debug::registry::IO_class_registry_reader<obj_class> &reader)
				{
					reader.io_multi_matches = std::bind(TextFiles::serialization_handle::match_file_type_multi,
						std::placeholders::_1, TextFiles::serialization_handle::getSHid(), 
						std::placeholders::_2, io::TextFiles::serialization_handle::known_formats());
					reader.io_multi_processor = chainedSerializer<obj_class, std::istream, 
						registry::IO_class_registry_reader<obj_class> >::genFunc(
						TextFiles::readSerialization<obj_class>, TextFiles::readFunc<obj_class>);
				}

			};
		}
#endif

		template <class obj_class,
		class output_registry_class>
		class implementsStandardWriter :
			virtual public boost::enable_shared_from_this<obj_class>
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
				auto opts = Ryan_Debug::registry::IO_options::generate();
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
				std::shared_ptr<Ryan_Debug::registry::IOhandler> handle,
				std::shared_ptr<Ryan_Debug::registry::IO_options> opts
				) const
			{
				auto& lg = Ryan_Debug::io::m_io::get();
				BOOST_LOG_SEV(lg, Ryan_Debug::debug::normal) << "Performing write on file " << opts->filename() << "\n";

				// All of these objects can handle their own compression
				typename ::Ryan_Debug::registry::IO_class_registry_writer<obj_class>::io_multi_type dllsaver = nullptr;
				// Process dll hooks first
				auto hooks = ::Ryan_Debug::registry::usesDLLregistry<output_registry_class,
					::Ryan_Debug::registry::IO_class_registry_writer<obj_class> >::getHooks();
				for (const auto &hook : *hooks)
				{
					if (!hook.io_multi_matches) continue; // Sanity check
					if (!hook.io_multi_processor) continue; // Sanity check
					//if (hook.io_multi_matches(filename, ctype, handle))
					if (hook.io_multi_matches(handle, opts))
					{
						BOOST_LOG_SEV(lg, Ryan_Debug::debug::normal) << "Found a match\n";
						dllsaver = hook.io_multi_processor;
						break;
					}
				}
				if (dllsaver)
				{
					try {	
						// Most of these types aren't compressible or implement their
						// own compression schemes. So, it's not handled at this level.
						return dllsaver(handle, opts, this->shared_from_this()); //dynamic_cast<const obj_class*>(this));
						//return dllsaver(handle, filename, dynamic_cast<const obj_class*>(this), key, accessType);
					} catch (::boost::exception &e) {
						BOOST_LOG_SEV(lg, ::Ryan_Debug::debug::error) << "Unable to save file: " << opts->filename() << "\nThrowing error.\n";
						e << ::Ryan_Debug::debug::file_name(opts->filename());
						throw;
					}
					
				} else {
					// Cannot match a file type to save.
					// Should never occur.
					BOOST_LOG_SEV(lg, ::Ryan_Debug::debug::warning) << "File format unknown for file: " << opts->filename() << "\n";
					RDthrow(debug::xUnknownFileFormat())
					<< debug::file_name(opts->filename());
				}
				return nullptr; // Should never be reached
			}

			bool canWriteMulti(
				std::shared_ptr<Ryan_Debug::registry::IOhandler> handle,
				std::shared_ptr<Ryan_Debug::registry::IO_options> opts
				) const
			{
				auto hooks = ::Ryan_Debug::registry::usesDLLregistry<output_registry_class,
					::Ryan_Debug::registry::IO_class_registry_writer<obj_class> >::getHooks();
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
		class implementsStandardSingleReader :
			virtual public boost::enable_shared_from_this<obj_class>
		{
		protected:
			// Controls whether boost::serialization can write this file.
			//bool serializable;
			implementsStandardSingleReader()
			{}

			/// This actually handles the template writing i/o. It can report the 
			/// success of the write to a calling parent class.
			bool baseRead(const std::string &filename, const std::string &intype,
				std::shared_ptr<const Ryan_Debug::registry::collectionTyped<obj_class> > filter = nullptr)
			{
				auto opts = Ryan_Debug::registry::IO_options::generate();
				opts->filename(filename);
				opts->setVal("key", filename);
				registry::IOhandler::IOtype accessType = registry::IOhandler::IOtype::READONLY;
				opts->iotype(accessType);
				opts->filetype(intype);
				auto res = readMulti(nullptr, opts, filter);
				//auto res = readMulti(filename.c_str(), nullptr, filename.c_str(),
				//	outtype.c_str(), registry::IOhandler::IOtype::TRUNCATE, opts);
				if (res) return true;
				return false;
			}
		public:
			virtual ~implementsStandardSingleReader() {}

			/// Duplicate to avoid clashes and having to specify a full template name...
			virtual void readFile(const std::string &filename, const std::string &intype = "",
				std::shared_ptr<const Ryan_Debug::registry::collectionTyped<obj_class> > filter = nullptr)
			{
				baseRead(filename, intype, filter);
			}

			virtual void read(const std::string &filename, const std::string &intype = "",
				std::shared_ptr<const Ryan_Debug::registry::collectionTyped<obj_class> > filter = nullptr)
			{
				baseRead(filename, intype, filter);
			}

			std::shared_ptr<registry::IOhandler> readMulti(
				std::shared_ptr<Ryan_Debug::registry::IOhandler> handle,
				std::shared_ptr<Ryan_Debug::registry::IO_options> opts,
				std::shared_ptr<const Ryan_Debug::registry::collectionTyped<obj_class> > filter = nullptr
				)
			{
				auto& lg = Ryan_Debug::io::m_io::get();
				BOOST_LOG_SEV(lg, Ryan_Debug::debug::normal) << "Performing single read on file " << opts->filename() << "\n";

				// All of these objects can handle their own compression
				typename ::Ryan_Debug::registry::IO_class_registry_reader<obj_class>::io_multi_type dllsaver = nullptr;
				// Process dll hooks first
				auto hooks = ::Ryan_Debug::registry::usesDLLregistry<input_registry_class,
					::Ryan_Debug::registry::IO_class_registry_reader<obj_class> >::getHooks();
				for (const auto &hook : *hooks)
				{
					if (!hook.io_multi_matches) continue; // Sanity check
					if (!hook.io_multi_processor) continue; // Sanity check
					//if (hook.io_multi_matches(filename, ctype, handle))
					if (hook.io_multi_matches(handle, opts))
					{
						BOOST_LOG_SEV(lg, Ryan_Debug::debug::normal) << "Found a match\n";
						dllsaver = hook.io_multi_processor;
						break;
					}
				}
				if (dllsaver)
				{
					// Most of these types aren't compressible or implement their
					// own compression schemes. So, it's not handled at this level.
					return dllsaver(handle, opts, this->shared_from_this(), filter); //dynamic_cast<obj_class*>(this), filter);
					//return dllsaver(handle, filename, dynamic_cast<const obj_class*>(this), key, accessType);
				} else {
					// Cannot match a file type to save.
					// Should never occur.
					BOOST_LOG_SEV(lg, Ryan_Debug::debug::warning) << "File format unknown for file: " << opts->filename() << "\n";
					RDthrow(debug::xUnknownFileFormat()) 
						<< debug::file_name(opts->filename());
				}
				return nullptr; // Should never be reached
			}

			static bool canReadMulti(
				std::shared_ptr<Ryan_Debug::registry::IOhandler> handle,
				std::shared_ptr<Ryan_Debug::registry::IO_options> opts
				)
			{
				auto hooks = ::Ryan_Debug::registry::usesDLLregistry<input_registry_class,
					::Ryan_Debug::registry::IO_class_registry_reader<obj_class> >::getHooks();
				for (const auto &hook : *hooks)
				{
					if (!hook.io_multi_matches) continue; // Sanity check
					if (!hook.io_multi_processor && !hook.io_vector_processor) continue; // Sanity check
					if (hook.io_multi_matches(handle, opts))
					{
						return true;
					}
				}
				return false;
			}
		};

		template <class obj_class>
		boost::shared_ptr<obj_class> customGenerator()
		{
			boost::shared_ptr<obj_class> res(new obj_class);
			return res;
		}

		template <class obj_class,
		class input_registry_class>
		class implementsStandardReader : public implementsStandardSingleReader<obj_class, input_registry_class>
		{
		protected:
			implementsStandardReader() : implementsStandardSingleReader<obj_class, input_registry_class>()
			{}
		public:
			virtual ~implementsStandardReader() {}

			/// Read all matching contained results into a vector
			static std::shared_ptr<registry::IOhandler> readVector(
				std::shared_ptr<Ryan_Debug::registry::IOhandler> handle,
				std::shared_ptr<Ryan_Debug::registry::IO_options> opts,
				std::vector<boost::shared_ptr<obj_class> > &v,
				std::shared_ptr<const Ryan_Debug::registry::collectionTyped<obj_class> > filter
				)
			{
				// Brief invocation of the object constructor to ensure that any custom handlers get registered.
				//boost::shared_ptr<obj_class> obj = 
				customGenerator<obj_class>();

				auto& lg = Ryan_Debug::io::m_io::get();
				BOOST_LOG_SEV(lg, Ryan_Debug::debug::normal) << "Performing vector read on file " << opts->filename() << "\n";

				// All of these objects can handle their own compression
				typename ::Ryan_Debug::registry::IO_class_registry_reader<obj_class>::io_vector_type dllv = nullptr;
				typename ::Ryan_Debug::registry::IO_class_registry_reader<obj_class>::io_multi_type dllm = nullptr;
				// Process dll hooks first
				auto hooks = ::Ryan_Debug::registry::usesDLLregistry<input_registry_class,
					::Ryan_Debug::registry::IO_class_registry_reader<obj_class> >::getHooks();
				for (const auto &hook : *hooks)
				{
					if (!hook.io_multi_matches) continue; // Sanity check
					//if (hook.io_multi_matches(filename, ctype, handle))
					if (hook.io_multi_matches(handle, opts))
					{
						if (hook.io_vector_processor)
							dllv = hook.io_vector_processor;
						else if (hook.io_multi_processor)
							dllm = hook.io_multi_processor;
						else continue; // No vector or multi reader - shouldn't happen if io_multi_matches, but fail to next plugin
						BOOST_LOG_SEV(lg, Ryan_Debug::debug::normal) << "Found a match\n";
						break;
					}
				}
				if (dllv || dllm)
				{
					try {
						// Most of these types aren't compressible or implement their
						// own compression schemes. So, it's not handled at this level.
						if (dllv) return dllv(handle, opts, v, filter);
						else {
							// obj_Class instance created using a generator template, which can be overridden if
							// the obj_class has no publicly-available constructor (if it only can be used 
							// as a shared_ptr, for example).
							//boost::shared_ptr<obj_class> obj = obj_class::generate();
							//boost::shared_ptr<obj_class> obj(new obj_class);
							boost::shared_ptr<obj_class> obj = customGenerator<obj_class>();
							auto res = dllm(handle, opts, obj, filter);
							v.push_back(obj);
							return res;
						}
					} catch (::boost::exception &e) {
						BOOST_LOG_SEV(lg, ::Ryan_Debug::debug::error) << "Unable to read file: " << opts->filename() << "\nThrowing error.\n";
						e << ::Ryan_Debug::debug::file_name(opts->filename());
						throw;
					}
					//return dllsaver(handle, filename, dynamic_cast<const obj_class*>(this), key, accessType);
				} else {
					// Cannot match a file type to read.
					// Should never occur.
					BOOST_LOG_SEV(lg, Ryan_Debug::debug::warning) << "File format unknown for file: " << opts->filename() << "\n";
					RDthrow(debug::xUnknownFileFormat())
						<< debug::file_name(opts->filename());
				}
				return nullptr; // Should never be reached
			}

		};

		/// Quick template to read objects, depending on vector read support.
		template <class T>
		void readObjs(std::vector<boost::shared_ptr<T> > &rinputs, const std::string &fname, 
			std::shared_ptr<const Ryan_Debug::registry::collectionTyped<T> > filter = nullptr)
		{
			auto iopts = registry::IO_options::generate(registry::IOhandler::IOtype::READONLY);
			iopts->filename(fname);
			//if (T::canReadMulti(nullptr, iopts)) // This is wrong. Indicates if it can be read at all, not just for vectors.
				T::readVector(nullptr, iopts, rinputs, filter);
			//else {
			//	boost::shared_ptr<T> s(new T);
			//	s->readFile(fname);
			//	rinputs.push_back(s);
			//}
		};

		/**
		* These don't really fit the standard io plugin spec, as they refer to database entries.
		* Database search and update semantics are different, and it is much better to do bulk searches 
		* and updates than one-by-one.
		* Of course, the io plugins could be usable, but selecting ranges for read with IO_options is bad, 
		* and handling many writes would either involve code complexity or poor code performance.
		**/
		template <class obj_class, class registry_class, class index_class, class comp_class, class query_reg_class>
		class implementsDBbasic
		{
		public:
			static std::shared_ptr<index_class> makeQuery() { return index_class::generate(); }
			/* // Doesn't compile in MSVC. Complains about the cast.
			std::shared_ptr<Ryan_Debug::registry::DBhandler> updateEntry(typename registry_class::updateType t,
				std::shared_ptr<Ryan_Debug::registry::DBhandler> p = nullptr, 
				std::shared_ptr<registry::DB_options> o = nullptr) const
			{
				auto c = makeCollection();
				std::shared_ptr<obj_class> obj = std::shared_ptr<obj_class>(new obj_class);
				*obj = (dynamic_cast<obj_class*>(this)); // TODO:
				c->insert(obj);
				return updateCollection(c, t, p, o);
			}
			*/
			static typename index_class:: collection makeCollection()
			{
				return typename index_class::collection
					(new std::set<boost::shared_ptr<const obj_class>, comp_class >());
			}
			static std::shared_ptr<Ryan_Debug::registry::DBhandler> 
				updateCollection(typename index_class::collection c, 
				typename registry_class::updateType t, 
				std::shared_ptr<Ryan_Debug::registry::DBhandler> p = nullptr, 
				std::shared_ptr<registry::DB_options> o = nullptr)
			{
				auto& lg = Ryan_Debug::io::m_io::get();
				BOOST_LOG_SEV(lg, Ryan_Debug::debug::normal) << "Updating database" << "\n";

				auto hooks = ::Ryan_Debug::registry::usesDLLregistry<query_reg_class, registry_class >::getHooks();
				for (const auto &h : *(hooks.get()))
				{
					if (!h.fMatches) continue;
					if (!h.fInsertUpdate) continue;
					if (h.fMatches(p, o))
						return h.fInsertUpdate(c, t, p, o);
				}
				return nullptr;
			}
		};

	}
}
