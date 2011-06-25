/* win-bonjour.h - provides Bonjour browsing and service registration support for rtmath-based software.
 * It exists as a dynamic-link library to allow for installs of the main software without the Bonjour components.
 * The service-browsing frature allows for fast network connections to be made.
 * The dll is imported in the same way that any other dll is explicitly-linked at run-time.
 * Its presence should be mentioned in the rtmath configuration for it to hook into the resolving functions.
 * The Linux equivalent uses Avahi.
 */

#pragma once

#include <string>
#include <set>
#include <map>

class host {
public:
	host(std::string &name);
	~host();
	std::string name;
	std::set<std::string> ipAddr;
	unsigned int port;
	group* parentGroup;
private:
};

class group {
public:
	group(std::string &name);
	~group();
	void getName(std::string &name) const;
	void setName(const std::string &name);
	std::set<host*> hosts;
private:
	std::string _name;
};

void registerService();
void unregisterService();

void setGroup(const std::string &groupname);
void getGroup(std::string &groupname);

void setName(const std::string &name);
void getName(std::string &name);
