#ifndef SDBR_MAIN
#define SDBR_MAIN

#include "defs.h"
#include <stdbool.h>
#include "data.h"

/// These functions have C-style linkage
#ifdef __cplusplus
extern "C" {
#endif

	/// This is an opaque pointer used throughout the c-style code.
	typedef void* SDBR_HANDLE;

	/// Deallocates a handle
	bool DLEXPORT_SDBR SDBR_free(SDBR_HANDLE);

	/// Returns the length of the last error message
	int DLEXPORT_SDBR SDBR_err_len();

	/// \brief Copies the last error message to the specified character array.
	/// \param maxlen is the maximum number of characters to write (including the
	/// null character).
	/// \param buffer is the output array.
	/// \returns The number of characters written.
	int DLEXPORT_SDBR SDBR_err_msg(int maxlen, char* buffer);

	/// Execute library load tasks
	bool DLEXPORT_SDBR SDBR_start(int argc, char** argv);

	/** \brief Load the database
	 * \param dbfile is a null-terminated string that indicates
	 * the path to the scatdb_ag_ryan.csv file. If NULL, then
	 * the file is automatically detected using the search paths according to the:
	 * 1) SCATDB_RYAN environment variable, 2) the current working directory,
	 * 3) the application binary directory, 4) the library directory,
	 * 5) the installation data directory (lib dir/../data?).
	 * \returns A handle to the opened database object.
	 **/
	SDBR_HANDLE DLEXPORT_SDBR SDBR_loadDB(const char* dbfile);

	/** \brief Find the database file
	 * \param maxlen is the maximum number of characters to write (including the
	 * null character).
	 * \param buffer is the output array.
	 * \returns The number of characters written. If zero, the search was unsuccessful.
	 **/
	int DLEXPORT_SDBR SDBR_findDB(int maxlen, char* buffer);

	/// Write the database
	/// \param handle is the database handle
	/// \param outfile is the null-terminated output filename
	/// \returns indicates success of operation. See error code if return is false.
	bool DLEXPORT_SDBR SDBR_writeDB(SDBR_HANDLE handle, const char* outfile);

	/// Get number of entries in database
	int DLEXPORT_SDBR SDBR_getNumEntries(SDBR_HANDLE handle);

	/// Get the 2-D array of floats

	/// Select a 1-D array of floats

	/// Get the 2-D array of ints

	/// Select a 1-D array of ints


/// Ensure that C-style linkage is respected
#ifdef __cplusplus
}
#endif

#endif

