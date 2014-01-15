#pragma once

#include <string>
#include <vector>

struct DBfile;

namespace rtmath {
	namespace plugins {
		namespace silo {
			/**
			* Keeps track of mixed material information.
			*
			* Notes:
			*
			* Programmer: Brad Whitlock
			* Creation:   Fri Jun 21 13:53:35 PST 2002
			*
			* Modifications:
			*   Brad Whitlock, Wed Mar 17 16:43:34 PST 2004
			*   I fixed an off by one error.
			*
			**/
			class MaterialList
			{
			public:
				MaterialList();
				~MaterialList();
				void AddMaterial(const std::string &mat);
				void AddClean(int siloZone, int matNumber);
				void AddMixed(int siloZone, int *matNumbers, double *matVf, int nMats);
				void AllocClean(int nZones);

				inline int GetMixedSize() const { return _array_index - 1; }

				void WriteMaterial(DBfile *db, const char *matvarname, const char *meshName, int nx, int ny, int nz);
			private:
				void Resize(int nMats);

				int    have_mixed;
				int    *mix_zone;
				int    *mix_mat;
				double *mix_vf;
				int    *mix_next;
				int    *matlist;
				int    _array_size;
				int    _array_index;
				int    _array_growth;
				std::vector<std::string> matNames;
			};

		}
	}
}
