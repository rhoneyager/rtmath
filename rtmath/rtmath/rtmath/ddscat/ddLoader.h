#pragma once
#include "../defs.h"
/* ddLoader.h
 * These functions are used by atmos to map a pf entry in an atmosphere
 * to the matching scattering, extinction and emission matrices. There
 * are several possible loaders, involving directly loading files, consulting
 * maps, hashes, mie and rayleigh generation and database lookups.
 */

#include <map>
#include <string>
#include <vector>
#include <memory>
#include "../matrixop.h"
#include "../da/damatrix.h"
#include "ddOutput.h"

namespace rtmath {

	class DEPRECATED ddLoader
	{
		// This class is pure virtual...
	public:
		// id field gets parsed by the loaders for additional information.
		ddLoader();
		virtual ~ddLoader();
		std::shared_ptr<const damatrix> getP() const;
		std::shared_ptr<const damatrix> getK() const;
		std::shared_ptr<const damatrix> getEmV() const;
	protected:
		std::shared_ptr<const damatrix> _P, _K, _EmV;
	private:
		void _init();
	public:
		// Finds appropriate loader and returns a prepared derived class
		static std::unique_ptr<ddLoader> findLoader
			(const std::string &id, const std::string &prepend);
	};

	class DEPRECATED ddLoaderFile : public ddLoader
	{
	public:
		ddLoaderFile(const std::string &id, const std::string &subheader);
		virtual ~ddLoaderFile();
	private:
		std::shared_ptr<rtmath::ddscat::ddOutput> _ddset;
	};

	class DEPRECATED ddLoaderMie : public ddLoader
	{
	public:
		ddLoaderMie(const std::string &id, const std::string &subheader);
		virtual ~ddLoaderMie();
	};

}; // end namespace rtmath

