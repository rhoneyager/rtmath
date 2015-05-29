#pragma once
#include "../defs.h"
#include <Ryan_Debug/hash.h>

namespace boost { namespace filesystem { class path; } }

namespace rtmath
{
	namespace ddscat
	{
		/// Some auxiliary functions for ddscat
		namespace ddUtil
		{
			/// Get forced ddscat module
			bool DLEXPORT_rtmath_ddscat isDDSCATtagForced(std::string &out);

			/// Get forced ddscat module
			bool DLEXPORT_rtmath_ddscat isDDSCATtagForced();

			/// Get ddscat build from tagged string
			void DLEXPORT_rtmath_ddscat getDDSCATbuild(const std::string &in, std::string &out);

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
			void ERR_UNIMPLEMENTED DLEXPORT_rtmath_ddscat tagTARGET(
				const boost::filesystem::path &pFile,
				Ryan_Debug::hash::HASH_t &hash, const std::string &ddver);

			/**
			 * \brief Modifies TARGET strings of all results in a ddscat 
			 * output directory.
			 *
			 * \param forceDDVER will force the ddscat version identifier to a prepassed value
			 * \see tagTARGET
			 **/
			void ERR_UNIMPLEMENTED DLEXPORT_rtmath_ddscat tagTARGETs(
				const boost::filesystem::path &pBase,
				const std::string &forceDDVER = std::string());

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

			/**
			 * \brief Adds ddUtils options to a program
			 *
			 * \item cmdline provides options only allowed on the command line
			 * \item config provides options available on the command line and in a config file
			 * \item hidden provides options allowed anywhere, but are not displayed to the user
			 **/
			void DLEXPORT_rtmath_ddscat add_options(
				boost::program_options::options_description &cmdline,
				boost::program_options::options_description &config,
				boost::program_options::options_description &hidden);
			/// Processes static options defined in add_options
			/// \todo Add processor for non-static options
			void DLEXPORT_rtmath_ddscat process_static_options(
				boost::program_options::variables_map &vm);
		}
	}
}
