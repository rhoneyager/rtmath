#include <stdio.h>

#include "../scatdb_ryan/scatdb_ryan.h"

void printError() {
	const int buflen = 1024;
	char buffer[buflen];
	SDBR_err_msg(buflen, buffer);
	printf("Error message:\n%s", buffer);
}

int main(int argc, char** argv) {
	printf("Calling SDBR_start\n");
	bool resb= SDBR_start(argc, argv);
	if (!resb) {
		printf("Library startup failed!\n");
		printError();
		return 1;
	}

	// Load the database
	printf("Loading the database\n");
	SDBR_HANDLE hdb = 0;
	hdb = SDBR_loadDB(0);
	if (!hdb) {
		printf("Cannot load database!\n");
		printError();
		return 2;
	}

	// Write the database
	printf("Writing the database to outdb.csv\n");
	resb = SDBR_writeDB(hdb, "outdb.csv");
	if (!resb) {
		printf("Cannot write database to outdb.csv!\n");
		printError();
		return 3;
	}

	// Close handles
	printf("Closing handles\n");
	SDBR_free(hdb);

	printf("Done\n");
	return 0;
}

