#include <memory>
#include <mutex>
#include <string>
#include <iostream>
#include <fstream>

#include "../scatdb_ryan/scatdb_ryan.hpp"

namespace {
	/// Always points to the first loaded database file.
	std::shared_ptr<const scatdb_ryan::db> loadedDB;
	std::mutex m_db;
}

namespace scatdb_ryan {
	db::db() {}
	db::~db() {}
	std::shared_ptr<const db> loadDB(const char* dbfile) {
		std::lock_guard<std::mutex> lock(m_db);
		if (!dbfile && loadedDB) return loadedDB;

		// Load the database
	}
}

