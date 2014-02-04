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
#include <boost/math/constants/constants.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/lexical_cast.hpp>

#include <Ryan_Serialization/serialization.h>
#include "../rtmath/macros.h"
#include "../rtmath/hash.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/registry.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	namespace registry {
		template struct IO_class_registry
			<::rtmath::ddscat::shapefile::shapefile>;
		
		template class usesDLLregistry<
			::rtmath::ddscat::shapefile::shapefile_IO_output_registry,
			IO_class_registry<::rtmath::ddscat::shapefile::shapefile> >;

		template class usesDLLregistry<
			::rtmath::ddscat::shapefile::shapefile_IO_input_registry,
			IO_class_registry<::rtmath::ddscat::shapefile::shapefile> >;
		
	}

	namespace ddscat {
		namespace shapefile {

			shapefile::shapefile() { _init(); }
			shapefile::~shapefile() { }

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

				std::istringstream ss_unc(s);
				read(s);
			}

			void shapefile::_init()
			{
				numPoints = 0;
				//filename = "";
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
					write(out);
					res = out.str();
					this->_localhash = HASH(res.c_str(), (int)res.size());
				}
				return this->_localhash;
			}

			boost::shared_ptr<shapefile> shapefile::loadHash(
				const HASH_t &hash)
			{ return loadHash(boost::lexical_cast<std::string>(hash.lower)); }

			
			void shapefile::readHeaderOnly(const std::string &filename)
			{ read(filename, true); }

			void shapefile::read(const std::string &filename, bool headerOnly)
			{
				using namespace std;
				using namespace boost::interprocess;
				using namespace boost::filesystem;
				// Detect if the input file is compressed
				using namespace Ryan_Serialization;
				std::string cmeth, fname;
				if (!detect_compressed(filename, cmeth, fname))
					throw rtmath::debug::xMissingFile(filename.c_str());

				// Do a direct map into memory. It's faster than stream i/o for reading a large file.
				// Plus, all other operations can be done solely in memory.
				size_t fsize = (size_t)file_size(path(fname)); // bytes

				file_mapping m_file(
					fname.c_str(),
					read_only
					);

				mapped_region region(
					m_file,
					read_only,
					0,
					fsize);

				void* start = region.get_address();
				const char* a = (char*)start;
				this->filename = fname;

				string s(a, fsize);
				std::istringstream ss(s);


				boost::iostreams::filtering_istream sin;
				// sin can contain either compressed or uncompressed input at this point.
				if (cmeth.size())
					prep_decompression(cmeth, sin);
				sin.push(ss);

				string suncompressed;
				suncompressed.reserve(1024 * 1024 * 10);
				std::ostringstream so;
				boost::iostreams::copy(sin, so);
				suncompressed = so.str();
				this->_localhash = HASH(suncompressed.c_str(), (int)suncompressed.size());

				istringstream ss_unc(suncompressed);
				size_t headerEnd = 0;
				readHeader(suncompressed.c_str(), headerEnd);
				if (!headerOnly)
					readContents(suncompressed.c_str(), headerEnd);
			}

			void shapefile::resize(size_t n)
			{
				numPoints = n;
				latticePts.conservativeResize(numPoints, 3);
				latticePtsRi.conservativeResize(numPoints, 3);
				latticePtsStd.conservativeResize(numPoints, 3);
				latticePtsNorm.conservativeResize(numPoints, 3);

				for (auto &extra : latticeExtras)
					extra.second.conservativeResize(numPoints, 3);
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
								numPoints = rtmath::macros::m_atoi(&(lin.data()[posa]), len);
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
									(*v)(j) = (float)rtmath::macros::m_atof(&(lin.data()[posa]), len);
								}
					}
						break;
					}
				}

				headerEnd = (pend - in) / sizeof(char);
				resize(numPoints);
			}

			void shapefile::write(std::ostream &out) const
			{ print(out); }

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

			boost::shared_ptr<shapefile> shapefile::decimate(size_t dx, size_t dy, size_t dz,
				decimationFunction dFunc) const
			{
				boost::shared_ptr<shapefile> res(new shapefile);

				size_t maxX = static_cast<size_t>(maxs(0)), maxY = static_cast<size_t>(maxs(1)), maxZ = static_cast<size_t>(maxs(2));
				size_t minX = static_cast<size_t>(mins(0)), minY = static_cast<size_t>(mins(1)), minZ = static_cast<size_t>(mins(2));
				size_t spanX = maxX - minX, spanY = maxY - minY, spanZ = maxZ - minZ;
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

				// Rescale x0 to point to the new center
				res->x0 = x0 / Eigen::Array3f((float)dx, (float)dy, (float)dz);


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
					if (point == 0)
					{
						res->mins = crdsm;
						res->maxs = crdsm;
						res->means = crdsm;
					}
					else {
						if (crdsm(0) < res->mins(0)) res->mins(0) = crdsm(0);
						if (crdsm(1) < res->mins(1)) res->mins(1) = crdsm(1);
						if (crdsm(2) < res->mins(2)) res->mins(2) = crdsm(2);
						if (crdsm(0) > res->maxs(0)) res->maxs(0) = crdsm(0);
						if (crdsm(1) > res->maxs(1)) res->maxs(1) = crdsm(1);
						if (crdsm(2) > res->maxs(2)) res->maxs(2) = crdsm(2);
						res->means(0) += crdsm(0);
						res->means(1) += crdsm(1);
						res->means(2) += crdsm(2);
					}
					point++;
				}
				res->means /= (float)(point + 1);

				if (point < num)
				{
					// Shrink the data store
					res->resize(point);
				}

				// Set the dielectrics
				res->Dielectrics.clear();
				for (size_t i = 1; i <= dielMax; ++i)
					res->Dielectrics.insert(i);

				res->recalcStats();

				return res;
			}

			boost::shared_ptr<shapefile> shapefile::enhance(size_t dx, size_t dy, size_t dz) const
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
				::rtmath::registry::IO_class_registry<shapefile>::io_processor_type dllsaver = nullptr;
				
				std::string uncompressed;
				Ryan_Serialization::uncompressed_name(filename, uncompressed, cmeth);
				path pext = path(uncompressed).extension();

				// Process dll hooks first
				auto hooks = usesDLLregistry<shapefile_IO_output_registry,
					::rtmath::registry::IO_class_registry<shapefile> >::getHooks();
				for (const auto &hook : *hooks)
				{
					if (hook.io_matches(uncompressed.c_str(), type.c_str()))
					{
						dllsaver = hook.io_processor;
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
					dllsaver(filename.c_str(), this);
				} else {
					// Cannot match a file type to save.
					// Should never occur.
					RTthrow debug::xUnknownFileFormat(filename.c_str());
				}
			}


		}
	}
}



std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::shapefile::shapefile &ob)
{
	ob.print(stream);
	return stream;
}

