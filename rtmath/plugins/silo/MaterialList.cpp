#include <vector>
#include <string>

#define DB_USE_MODERN_DTPTR
#include <silo.h>

#include "remake.h"
#include "MaterialList.h"

namespace rtmath {
	namespace plugins {
		namespace silo {

			MaterialList::MaterialList() : matNames()
			{
				have_mixed = false;
				mix_zone = NULL;
				mix_mat = NULL;
				mix_vf = NULL;
				mix_next = NULL;
				matlist = NULL;

				// initialize private members.
				_array_size = 0;
				_array_index = 1;
				_array_growth = 1000;
			}

			MaterialList::~MaterialList()
			{
				delete[] matlist;
				if (have_mixed)
				{
					delete[] mix_zone;
					delete[] mix_mat;
					delete[] mix_vf;
					delete[] mix_next;
				}
			}

			void MaterialList::AddMaterial(const std::string &mat)
			{
				matNames.push_back(mat);
			}

			void MaterialList::AddClean(int siloZone, int matNumber)
			{
				matlist[siloZone] = matNumber;
			}

			void MaterialList::AddMixed(int siloZone, int *matNumbers, double *matVf, int nMats)
			{
				int i;

				/* Grow the arrays if they will not fit nMats materials. */
				Resize(nMats);

				/* Record the mixed zone as a negative offset into the mix arrays. */
				matlist[siloZone] = -_array_index;

				/* Update the mix arrays. */
				for (i = 0; i < nMats; ++i)
				{
					int index = _array_index - 1;

					mix_zone[index] = siloZone;
					mix_mat[index] = matNumbers[i];
					mix_vf[index] = matVf[i];

					if (i < nMats - 1)
						mix_next[index] = index + 2;
					else
						mix_next[index] = 0;

					++(_array_index);
				}

				/* indicate that we have mixed materials. */
				have_mixed = true;
			}

			void MaterialList::AllocClean(int nZones)
			{
				matlist = new int[nZones];
			}

			void MaterialList::WriteMaterial(DBfile *db, const char *matvarname, const char *meshName, int nx, int ny, int nz)
			{
				int i, mdims[3] = { nx, ny, nz };

				/* Create a 1..nTotalMaterials material number array. */
				int *allmats = new int[matNames.size()];
				for (i = 0; i < matNames.size(); ++i)
					allmats[i] = i + 1;

				DBoptlist *optList = DBMakeOptlist(2);

				// Add material names.
				char **matnames = new char *[4];
				for (i = 0; i < matNames.size(); ++i)
					matnames[i] = (char *)matNames[i].c_str();
				DBAddOption(optList, DBOPT_MATNAMES, matnames);

				if (have_mixed)
				{
					DBPutMaterial(db, (char *)matvarname, (char *)meshName,
						(int) matNames.size(), allmats,
						matlist, mdims, 3, mix_next,
						mix_mat, mix_zone,
						(float*)mix_vf, GetMixedSize(),
						DB_DOUBLE, optList);
				}
				else
				{
					DBPutMaterial(db, (char *)matvarname, (char *)meshName,
						(int) matNames.size(), allmats,
						matlist, mdims, 3, NULL,
						NULL, NULL, NULL, 0,
						DB_INT, optList);
				}

				DBFreeOptlist(optList);
				delete[] matnames;
				delete[] allmats;
			}

			void MaterialList::Resize(int nMats)
			{
				if (_array_index + nMats >= _array_size)
				{
					int new_size = _array_size + _array_growth;

					if (_array_size == 0)
					{
						/* Reallocate arrays in large increments. */
						mix_zone = new int[new_size];
						mix_mat = new int[new_size];
						mix_vf = new double[new_size];
						mix_next = new int[new_size];
					}
					else
					{
						/* Reallocate arrays in large increments. */
						mix_zone = remake(mix_zone, _array_size, new_size);
						mix_mat = remake(mix_mat, _array_size, new_size);
						mix_vf = remake(mix_vf, _array_size, new_size);
						mix_next = remake(mix_next, _array_size, new_size);
					}

					_array_size = new_size;
				}
			}

		}
	}
}

