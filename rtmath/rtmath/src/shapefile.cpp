#include "Stdafx-ddscat.h"
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <tuple>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/weak_ptr.hpp>

#include <Ryan_Debug/debug.h>
#include "../rtmath/macros.h"
#include "../rtmath/hash.h"
#include "../rtmath/Voronoi/Voronoi.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/splitSet.h"
#include "../rtmath/registry.h"
#include "../rtmath/Serialization/Serialization.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

namespace {
	std::set<std::string> mtypes;
	std::mutex mlock_shp;

	std::map<std::string, 
		boost::weak_ptr<const ::rtmath::ddscat::shapefile::shapefile> > loadedShapes;
	//std::map<rtmath::HASH_t, boost::shared_ptr<shapefile> > stubShapes;
}

namespace rtmath {
	namespace registry {
		template struct IO_class_registry_writer
			<::rtmath::ddscat::shapefile::shapefile>;

		template struct IO_class_registry_reader
			<::rtmath::ddscat::shapefile::shapefile>;

		template class usesDLLregistry<
			::rtmath::ddscat::shapefile::shapefile_IO_output_registry,
			IO_class_registry_writer<::rtmath::ddscat::shapefile::shapefile> >;

		template class usesDLLregistry<
			::rtmath::ddscat::shapefile::shapefile_IO_input_registry,
			IO_class_registry_reader<::rtmath::ddscat::shapefile::shapefile> >;
		
		template class usesDLLregistry<
			::rtmath::ddscat::shapefile::shapefile_query_registry,
			::rtmath::ddscat::shapefile::shapefile_db_registry >;
		
	}

	namespace io {
		template <>
		boost::shared_ptr<::rtmath::ddscat::shapefile::shapefile> customGenerator()
		{
			boost::shared_ptr<::rtmath::ddscat::shapefile::shapefile> res
				(new ::rtmath::ddscat::shapefile::shapefile);
			return res;
		}
	}
	namespace ddscat {
		namespace shapefile {

			implementsDDSHP::implementsDDSHP() :
				rtmath::io::implementsIObasic<shapefile, shapefile_IO_output_registry,
				shapefile_IO_input_registry, shapefile_Standard>(shapefile::writeDDSCAT, shapefile::readDDSCAT, known_formats())
			{}

			const std::set<std::string>& implementsDDSHP::known_formats()
			{
				// Moved to hidden file scope to avoid race condition
				//static std::set<std::string> mtypes;
				//static std::mutex mlock;
				// Prevent threading clashes
				{
					std::lock_guard<std::mutex> lck(mlock_shp);
					if (!mtypes.size())
					{
						mtypes.insert(".shp");
						mtypes.insert("shape.txt");
						mtypes.insert(".dat");
						mtypes.insert("shape.dat");
						mtypes.insert("target.out");
					}
					if (io::TextFiles::serialization_handle::compressionEnabled())
					{
						std::string sctypes;
						std::set<std::string> ctypes;
						serialization::known_compressions(sctypes, ".shp");
						serialization::known_compressions(sctypes, ".dat");
						serialization::known_compressions(sctypes, "shape.txt");
						serialization::known_compressions(sctypes, "shape.dat");
						serialization::known_compressions(sctypes, "target.out");
						rtmath::config::splitSet(sctypes, ctypes);
						for (const auto & t : ctypes)
							mtypes.emplace(t);
					}
				}
				return mtypes;
			}

			shapefile::shapefile() { _init(); }
			shapefile::~shapefile() { }

//#if _MSC_FULL_VER
			shapefile& shapefile::operator=(const shapefile& rhs)
			{
				if (this == &rhs) return *this;
#define cp(x) this->x = rhs.x;
				cp(_localhash);
				cp(filename);
				cp(ingest_timestamp);
				cp(ingest_hostname);
				cp(ingest_username);
				cp(ingest_rtmath_version);
				cp(latticePts);
				cp(latticePtsStd);
				cp(latticePtsNorm);
				cp(latticePtsRi);
				cp(latticeIndex);
				cp(tags);
				cp(latticeExtras);
				cp(numPoints);
				cp(Dielectrics);
				cp(desc);
				cp(a1);
				cp(a2);
				cp(a3);
				cp(d);
				cp(x0);
				cp(xd);
				cp(mins);
				cp(maxs);
				cp(means);

#undef cp
				return *this;
			}
//#endif

			bool shapefile::isHashStored(const std::string &hash)
			{
				if (loadedShapes.count(hash)) return true;
				return false;
			}

			bool shapefile::isHashStored(const HASH_t &hash) { return isHashStored(hash.string()); }

			void shapefile::registerHash() const
			{
				boost::shared_ptr<const shapefile> ptr = this->shared_from_this();
				boost::weak_ptr<const shapefile> wshp(ptr);
				loadedShapes[this->_localhash.string()] = wshp;
				//loadedShapes.emplace(std::pair < std::string,
				//boost::weak_ptr<const ::rtmath::ddscat::shapefile::shapefile> > (
				//this->_localhash.string(), wshp));
			}

			boost::shared_ptr<shapefile> shapefile::generate(const std::string &filename)
			{
				boost::shared_ptr<shapefile> res(new shapefile(filename));
				return res;
			}

			boost::shared_ptr<shapefile> shapefile::generate(std::istream &in)
			{
				boost::shared_ptr<shapefile> res(new shapefile(in));
				return res;
			}

			boost::shared_ptr<shapefile> shapefile::generate()
			{
				boost::shared_ptr<shapefile> res(new shapefile);
				return res;
			}

			boost::shared_ptr<shapefile> shapefile::generate(boost::shared_ptr<const shapefile> p)
			{
				boost::shared_ptr<shapefile> res(new shapefile);
				*res = *p;
				return res;
			}

			shapefile::shapefile(const std::string &filename)
			{
				_init();
				read(filename);
				this->filename = filename;
			}

			shapefile::shapefile(std::istream &in)
			{
				_init();
				std::ostringstream so;
				boost::iostreams::copy(in, so);
				std::string s;
				s = so.str();
				this->_localhash = HASH(s.c_str(), (int)s.size());

				//std::istringstream ss_unc(s);
				readString(s);
			}

			void shapefile::_init()
			{
				numPoints = 0;
				standardD = 0;

				// Add the ingest tags. If using a copy constructor, then these get overridden.
				using namespace boost::posix_time;
				using namespace boost::gregorian;
				ptime now = second_clock::local_time();
				ingest_timestamp = to_iso_string(now);
				ingest_hostname = Ryan_Debug::getHostname();
				ingest_username = Ryan_Debug::getUsername();
				ingest_rtmath_version = rtmath::debug::rev();

				//filename = "";
				//::rtmath::io::Serialization::implementsSerialization<
				//	shapefile, shapefile_IO_output_registry, 
				//	shapefile_IO_input_registry, shapefile_serialization>::set_sname("rtmath::ddscat::shapefile::shapefile");
			}

			void shapefile::setHash(const HASH_t &h)
			{
				_localhash = h;
			}

			HASH_t shapefile::hash() const
			{
				if (_localhash.lower) return _localhash;
				return rehash();
			}

			HASH_t shapefile::rehash() const
			{
				if (numPoints)
				{
					std::string res;
					std::ostringstream out;
					print(out);
					res = out.str();
					this->_localhash = HASH(res.c_str(), (int)res.size());
				}
				return this->_localhash;
			}

			void shapefile::loadHashLocal()
			{
				if (latticePts.rows()) return; // Already loaded
				const auto origtags = tags;
				auto sD = standardD;

				// Tags are stored this way because incomplete entries usually come from databases.
				loadHashLocal(hash());
				
				if (sD) standardD = sD;
				for (const auto &tag : origtags)
					//if (!tags.count(tag.first)) 
					tags[tag.first] = tag.second;
			}

			boost::shared_ptr<const shapefile> shapefile::loadHash(
				const HASH_t &hash)
			{ return loadHash(boost::lexical_cast<std::string>(hash.lower)); }

			void shapefile::loadHashLocal(
				const HASH_t &hash)
			{
				loadHashLocal(boost::lexical_cast<std::string>(hash.lower));
			}

			
			void shapefile::readHeaderOnly(const std::string &str)
			{
				readString(str, true);
			}

			void shapefile::readString(const std::string &in, bool headerOnly)
			{
				size_t headerEnd = 0;
				readHeader(in.c_str(), headerEnd);
				if (!headerOnly)
					readContents(in.c_str(), headerEnd);
			}

			void shapefile::resize(size_t n)
			{
				numPoints = n;
				latticePts.conservativeResize(numPoints, 3);
				latticePtsRi.conservativeResize(numPoints, 3);
				latticePtsStd.conservativeResize(numPoints, 3);
				latticePtsNorm.conservativeResize(numPoints, 3);
				latticeIndex.conservativeResize(numPoints);
				latticeIndex.setLinSpaced(numPoints,1,(int) numPoints);
				//for (auto &extra : latticeExtras)
				//	extra.second->conservativeResize(numPoints, 3);
			}

			void shapefile::readHeader(const char* in, size_t &headerEnd)
			{
				using namespace std;

				// Do header processing using istreams.
				// The previous method used strings, but this didn't work with compressed reads.
				//size_t &pend = headerEnd;
				const char* pend = in;
				const char* pstart = in;

				// The header is seven lines long
				for (size_t i = 0; i < 7; i++)
				{
					pstart = pend;
					pend = strchr(pend, '\n');
					pend++; // Get rid of the newline
					//pend = in.find_first_of("\n", pend+1);
					string lin(pstart, pend - pstart - 1);
					if (*(lin.rbegin()) == '\r') lin.pop_back();
					//std::getline(in,lin);

					size_t posa = 0, posb = 0;
					Eigen::Array3f *v = nullptr;
					switch (i)
					{
					case 0: // Title line
						desc = lin;
						break;
					case 1: // Number of dipoles
					{
								// Seek to first nonspace character
								posa = lin.find_first_not_of(" \t\n", posb);
								// Find first space after this position
								posb = lin.find_first_of(" \t\n", posa);
								size_t len = posb - posa;
								numPoints = rtmath::macros::m_atoi<size_t>(&(lin.data()[posa]), len);
					}
						break;
					case 6: // Junk line
					default:
						break;
					case 2: // a1
					case 3: // a2
					case 4: // d
					case 5: // x0
						// These all have the same structure. Read in three doubles, then assign.
					{
								if (2 == i) v = &a1;
								if (3 == i) v = &a2;
								if (4 == i) v = &d;
								if (5 == i) v = &x0;
								for (size_t j = 0; j < 3; j++)
								{
									// Seek to first nonspace character
									posa = lin.find_first_not_of(" \t\n,", posb);
									// Find first space after this position
									posb = lin.find_first_of(" \t\n,", posa);
									size_t len = posb - posa;
									(*v)(j) = rtmath::macros::m_atof<float>(&(lin.data()[posa]), len);
								}
					}
						break;
					}
				}

				headerEnd = (pend - in) / sizeof(char);
				resize(numPoints);
			}

			void shapefile::writeDDSCAT(const shapefile *s, std::ostream &out, std::shared_ptr<registry::IO_options>)
			{ s->print(out); }

			void shapefile::readDDSCAT(shapefile *s, std::istream &in, std::shared_ptr<registry::IO_options> opts)
			{
				std::ostringstream so;
				boost::iostreams::copy(in, so);
				std::string str = so.str();
				bool headerOnly = opts->getVal<bool>("headerOnly", false);
				s->readString(str, headerOnly);
				
				// This is a standard ddscat file, so add the ingest tags
				s->filename = opts->filename();
				using namespace boost::posix_time;
				using namespace boost::gregorian;
				ptime now = second_clock::local_time();
				s->ingest_timestamp = to_iso_string(now);
				s->ingest_hostname = Ryan_Debug::getHostname();
				s->ingest_username = Ryan_Debug::getUsername();
				s->ingest_rtmath_version = rtmath::debug::rev();

			}

			/*
			void shapefile::writeVTK(const std::string &fname) const
			{
				rtmath::ddscat::convexHull hull(latticePtsStd);
				hull.writeVTKraw(fname);
			}
			*/

			size_t shapefile::decimateDielCount(const convolutionCellInfo& info)
			{ return info.numFilled; }

			size_t shapefile::decimateThreshold(const convolutionCellInfo& info,
				size_t threshold)
			{
				if (info.numFilled >= threshold) return info.initDiel;
				return 0;
			}

			convolutionCellInfo::convolutionCellInfo() : x(0), y(0), z(0),
				initDiel(0), sx(0), sy(0), sz(0), numFilled(0), numTotal(0), index(0)
			{}

			boost::shared_ptr<Voronoi::VoronoiDiagram> shapefile::generateVoronoi(
				const std::string &name,
				std::function < boost::shared_ptr<Voronoi::VoronoiDiagram>(
				const Eigen::Array3f&, const Eigen::Array3f&,
				const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>&, const char*)> f) const
			{
				//if (voronoi_diagrams.count(name))
				//	return voronoi_diagrams[name];
				// Attempt to load the voronoi diagram from the hash database.
				boost::shared_ptr<Voronoi::VoronoiDiagram> res;
				std::cerr << "Requesting voronoi diagrams for " << hash().string() << std::endl;
				res = Voronoi::VoronoiDiagram::loadHash(hash());
				if (!res) {
					std::cerr << " Diagrams not found. Generating." << std::endl;
					res = f(mins, maxs, latticePts, ""); // note that no particular plugin is specified here.
					res->setHash(this->hash());
				}

				//voronoi_diagrams[name] = res;
				return res;
			}

			boost::shared_ptr<const shapefile> shapefile::decimate(size_t dx, size_t dy, size_t dz,
				decimationFunction dFunc) const
			{
				boost::shared_ptr<shapefile> res(new shapefile);

				auto maxX = maxs(0), maxY = maxs(1), maxZ = maxs(2);
				auto minX = mins(0), minY = mins(1), minZ = mins(2);
				size_t spanX = (size_t) (maxX - minX), spanY = (size_t)(maxY - minY), spanZ = (size_t)(maxZ - minZ);
				size_t rsX = (spanX / dx) + 1, rsY = (spanY / dy) + 1, rsZ = (spanZ / dz) + 1;

				auto getIndex = [&](float x, float y, float z) -> size_t
				{
					size_t index = 0;
					size_t sX = (size_t)(x - minX) / dx;
					size_t sY = (size_t)(y - minY) / dy;
					size_t sZ = (size_t)(z - minZ) / dz;
					index = (sX * (rsY * rsZ)) + (sY * rsZ) + sZ;
					return index;
				};

				auto getCrds = [&](size_t index) -> std::tuple<size_t, size_t, size_t>
				{
					size_t x = index / (rsY*rsZ);
					index -= x*rsY*rsZ;
					size_t y = index / rsZ;
					index -= y*rsZ;
					size_t z = index;
					return std::tuple<size_t, size_t, size_t>(x, y, z);
				};


				std::vector<convolutionCellInfo> vals(rsX*rsY*rsZ);
				for (auto &v : vals)
				{
					v.initDiel = 1;
					v.numTotal = dx * dy * dz;
					v.sx = dx;
					v.sy = dy;
					v.sz = dz;
				}

				// Iterate over all points and bin into the appropriate set
				for (size_t i = 0; i < numPoints; ++i)
				{
					auto crdsm = latticePts.block<1, 3>(i, 0);
					float x = crdsm(0), y = crdsm(1), z = crdsm(2);
					size_t index = getIndex(x, y, z);
					auto &v = vals.at(index);
					v.numFilled++;
					v.index = index;
					v.x = x;
					v.y = y;
					v.z = z;
				}

				// Count the decimated points with nonzero values
				size_t num = vals.size() - std::count_if(vals.begin(), vals.end(), [&]
					(const convolutionCellInfo& c){if (c.numFilled == 0) return true; return false; });

				res->a1 = a1;
				res->a2 = a2;
				res->a3 = a3;
				res->d = d;
				res->desc = desc;
				res->Dielectrics = Dielectrics;
				res->filename = filename;
				res->xd = xd;

				

				res->resize(num);

				size_t dielMax = 0;
				// Set the decimated values
				size_t point = 0;
				for (size_t i = 0; i < vals.size(); ++i)
				{
					size_t diel = dFunc(vals.at(i));
					if (diel == 0) continue;
					auto t = getCrds(i);
					auto crdsm = res->latticePts.block<1, 3>(point, 0);
					auto crdsi = res->latticePtsRi.block<1, 3>(point, 0);
					crdsm(0) = (float)std::get<0>(t);
					crdsm(1) = (float)std::get<1>(t);
					crdsm(2) = (float)std::get<2>(t);
					crdsi(0) = (float)diel;
					crdsi(1) = (float)diel;
					crdsi(2) = (float)diel;
					if (dielMax < diel) dielMax = diel;
					point++;
				}
				
				// Shrink the data store
				if (point < num)
					res->resize(point);

				// Set the dielectrics
				res->Dielectrics.clear();
				for (size_t i = 1; i <= dielMax; ++i)
					res->Dielectrics.insert(i);

				res->recalcStats();
				// Rescale x0 to point to the new center
				//res->x0 = x0 / Eigen::Array3f((float)dx, (float)dy, (float)dz);
				// Cannot just rescale because of negative coordinates.
				res->x0 = res->means;


				return res;
			}

			boost::shared_ptr<const shapefile> shapefile::enhance(size_t dx, size_t dy, size_t dz) const
			{
				boost::shared_ptr<shapefile> res(new shapefile);

				size_t maxX = static_cast<size_t>(maxs(0)), maxY = static_cast<size_t>(maxs(1)), maxZ = static_cast<size_t>(maxs(2));
				size_t minX = static_cast<size_t>(mins(0)), minY = static_cast<size_t>(mins(1)), minZ = static_cast<size_t>(mins(2));
				size_t spanX = maxX - minX, spanY = maxY - minY, spanZ = maxZ - minZ;
				size_t rsX = (spanX / dx) + 1, rsY = (spanY / dy) + 1, rsZ = (spanZ / dz) + 1;

				// Resize the resultant shapefile based on the new scale
				size_t nd = dx * dy * dz;
				res->resize(nd * numPoints);

				res->a1 = a1;
				res->a2 = a2;
				res->a3 = a3;
				res->d = d;
				res->desc = desc;
				res->Dielectrics = Dielectrics;
				res->filename = filename;
				res->xd = xd;
				// Rescale x0 to point to the new center
				res->x0 = x0 * Eigen::Array3f((float)dx, (float)dy, (float)dz);


				// Iterate over all points and bin into the appropriate set
				for (size_t i = 0; i < numPoints; ++i)
				{
					auto crdsm = latticePts.block<1, 3>(i, 0);
					auto crdsi = latticePtsRi.block<1, 3>(i, 0);

					for (size_t x = 0; x < dx; ++x)
					for (size_t y = 0; y < dy; ++y)
					for (size_t z = 0; z < dz; ++z)
					{
						size_t j = (nd * i) + z + (dz*y) + (dz*dy*x);
						auto rdsm = res->latticePts.block<1, 3>(j, 0);
						auto rdsi = res->latticePtsRi.block<1, 3>(j, 0);
						rdsm(0) = crdsm(0) + x;
						rdsm(1) = crdsm(1) + y;
						rdsm(2) = crdsm(2) + z;
						rdsi = crdsi;
					}
				}

				res->recalcStats();

				return res;
			}

			/*
			std::shared_ptr<registry::IOhandler> shapefile::writeMulti(
					const char* key,
					std::shared_ptr<registry::IOhandler> handle,
					const char* filename,
					const char* ctype,
					registry::IOhandler::IOtype accessType) const
			{
				// All of these objects can handle their own compression
				::rtmath::registry::IO_class_registry_writer<shapefile>::io_multi_type dllsaver = nullptr;
				// Process dll hooks first
				auto hooks = usesDLLregistry<shapefile_IO_output_registry,
					::rtmath::registry::IO_class_registry_writer<shapefile> >::getHooks();
				auto opts = registry::IO_options::generate();
				opts->filename(filename);
				opts->filetype(ctype);
				opts->iotype(accessType);
				opts->setVal("key", std::string(key));
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
					return dllsaver(nullptr, opts, this);
					//return dllsaver(handle, filename, this, key, accessType);
				} else {
					// Cannot match a file type to save.
					// Should never occur.
					RTthrow debug::xUnknownFileFormat(filename);
				}
				return nullptr; // Should never be reached
			}

			void shapefile::write(const std::string &filename, bool autoCompress,
				const std::string &outtype) const
			{
				using namespace Ryan_Serialization;
				using namespace std;
				using boost::filesystem::path;

				std::string cmeth;
				std::ostringstream outfile;
				if (Ryan_Serialization::detect_compression(filename, cmeth))
					autoCompress = true;
				if (autoCompress)
					Ryan_Serialization::select_compression(filename, cmeth);

				// If missing the type, autodetect based on file extension
				std::string type = outtype;
				::rtmath::registry::IO_class_registry_writer<shapefile>::io_multi_type dllsaver = nullptr;
				
				std::string uncompressed;
				Ryan_Serialization::uncompressed_name(filename, uncompressed, cmeth);
				path pext = path(uncompressed).extension();

				// Process dll hooks first
				auto hooks = usesDLLregistry<shapefile_IO_output_registry,
					::rtmath::registry::IO_class_registry_writer<shapefile> >::getHooks();
				auto opts = registry::IO_options::generate();
				opts->filename(uncompressed);
				opts->filetype(type);
				for (const auto &hook : *hooks)
				{
					//if (hook.io_multi_matches(uncompressed.c_str(), type.c_str()))
					if (hook.io_multi_matches(nullptr, opts))
					{
						dllsaver = hook.io_multi_processor;
						if (!type.size())
							type = "dll";
						break;
					}
				}
				if (!type.size())
				{
					if (Ryan_Serialization::known_format(uncompressed)) type = "serialized";
					// Default is to write a standard shapefile
					else type = "shp";
				}

				// Now, save the appropriate format based on the type
				/// \todo Ryan_Serialization::select_compression should also return the compressed 
				/// file name as an optional parameter.
				
				if (type == "shp" || type == ".shp")
				{
					outfile << filename;
					if (cmeth.size()) outfile << "." << cmeth;
					std::string soutfile = outfile.str();

					ofstream out(soutfile.c_str(), ios_base::out | ios_base::binary);
					using namespace boost::iostreams;
					filtering_ostream sout;
					if (cmeth.size())
						prep_compression(cmeth, sout);
					sout.push(out);
					write(sout);
				}
				else if (type == "serialized")
				{
					Ryan_Serialization::write<shapefile>(*this, filename, 
						"rtmath::ddscat::shapefile::shapefile");
				}
				else if (dllsaver)
				{
					// Most of these types aren't compressible or implement their
					// own compression schemes. So, it's not handled at this level.
					dllsaver(nullptr, opts, this);
					//dllsaver(filename.c_str(), this);
				} else {
					// Cannot match a file type to save.
					// Should never occur.
					RTthrow debug::xUnknownFileFormat(filename.c_str());
				}
			}
            */

			void shapefile::fixStats()
			{
				x0 = means;
				xd = x0 * d;
			}



			boost::shared_ptr<const shapefile> shapefile::loadHash(
				const std::string &hash)
			{
				if (loadedShapes.count(hash))
				{
					auto wptr = loadedShapes.at(hash);
					if (!wptr.expired()) return wptr.lock();
				}

				boost::shared_ptr<shapefile> res(new shapefile);

				using boost::filesystem::path;
				using boost::filesystem::exists;

				std::shared_ptr<registry::IOhandler> sh;
				std::shared_ptr<registry::IO_options> opts; // No need to set - it gets reset by findHashObj

				if (hashStore::findHashObj(hash, "shape.hdf5", sh, opts))
				{
					opts->setVal<std::string>("key", hash);
					res = boost::shared_ptr<shapefile>(new shapefile);
					res->readMulti(sh, opts);
				}

				res->registerHash();
				return res;
			}

			void shapefile::loadHashLocal(
				const std::string &hash)
			{
				if (loadedShapes.count(hash))
				{
					auto wptr = loadedShapes.at(hash);
					if (!wptr.expired())
					{
						*this = *(wptr.lock());
						return;
					}
				}

				using boost::filesystem::path;
				using boost::filesystem::exists;

				std::shared_ptr<registry::IOhandler> sh;
				std::shared_ptr<registry::IO_options> opts; // No need to set - it gets reset by findHashObj

				if (hashStore::findHashObj(hash, "shape.hdf5", sh, opts))
				{
					opts->setVal<std::string>("key", hash);
					readMulti(sh, opts);
				}
				else {
					RTthrow debug::xMissingHash(hash.c_str(), "shapefile");
				}

				this->registerHash();
			}

		}
	}
}



std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::shapefile::shapefile &ob)
{
	ob.print(stream);
	return stream;
}

