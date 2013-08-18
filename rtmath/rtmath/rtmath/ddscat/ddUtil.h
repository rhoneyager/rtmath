#pragma once
#include "../defs.h"
#include "../hash.h"

namespace boost { namespace filesystem { class path; } }

namespace rtmath
{
	namespace ddscat
	{
		/// Some auxiliary functions for ddscat
		namespace ddUtil
		{
			/**
			* \brief Function gets the version of ddscat based  
			* on the loaded environment module. 
			*
			* \returns true if a module has been found
			* \returns false if no ddscat module is loaded
			* \param out is the ddscat version identifier
			* \todo Make this function accessible elsewhere
			**/
			bool DLEXPORT_rtmath_ddscat getModuleLoadedDDSCAT(std::string &out);

			/**
			 * \brief Modifies the TARGET string of a ddscat output file 
			 * by appending some identifying information.
			 *
			 * It appends the environment-module ddscat id, the shape 
			 * file hash and a timestamp. If these features are already 
			 * detected, it does nothing.
			 **/
			void DLEXPORT_rtmath_ddscat tagTARGET(
				const boost::filesystem::path &pFile,
				HASH_t &hash, const std::string &ddver);

			/**
			 * \brief Modifies TARGET strings of all results in a ddscat 
			 * output directory.
			 *
			 * \see tagTARGET
			 **/
			void DLEXPORT_rtmath_ddscat tagTARGETs(
				const boost::filesystem::path &pBase);

			/**
			 * \brief Find appropriate files in a ddscat run directory
			 *
			 * Finds the par file and shape file for a run given a base path.
			 * The base path can be a file or a directory.
			 **/
			void DLEXPORT_rtmath_ddscat findDDFiles(
				const boost::filesystem::path &pBase,
				boost::filesystem::path &pPar,
				boost::filesystem::path &pShp);
		}
	}
}
