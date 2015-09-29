#ifndef SDBR_MAIN
#define SDBR_MAIN

#include "defs.h"

/// These functions have C-style linkage
#ifdef __cplusplus
extern "C" {
#endif

	/// This is an opaque pointer used throughout the c-style code.
	typedef void* SDBR_HANDLE

	/// Returns the length of the last error message
	int DLEXPORT_SDBR SDBR_err_len();
	/// \brief Copies the last error message to the specified character array.
	/// \param maxlen is the maximum number of characters to write (including the
	/// null character).
	/// \param buffer us the output array.
	/// \returns The number of characters written.
	int DLEXPORT_SDBR SDBR_err_msg(int maxlen, char* buffer);

	/** \brief Load the database
	 * \param dbfile is a null-terminated string that indicates
	 * the path to the scatdb_ag_ryan.csv file. If blank, then
	 * the file is automatically detected using the search paths according to the:
	 * 1) SCATDB_RYAN environment variable, 2) the current working directory,
	 * 3) the application binary directory, 4) the library directory,
	 * 5) the installation data directory (lib dir/../data?).
	 * \returns A handle to the opened database object.
	 **/
	SDBR_HANDLE DLEXPORT_SDBR SDBR_loadDB(const char* dbfile = 0);

/// Ensure that C-style linkage is respected
#ifdef __cplusplus
}
#endif

#endif

