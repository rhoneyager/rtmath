/// \brief Provides TSV file IO
#define _SCL_SECURE_NO_WARNINGS

#include <string>
#define IGNORE_MANIFESTS
#include "plugin-test.h"
#include "../../Ryan_Debug/logging.h"

D_Ryan_Debug_validator();

D_Ryan_Debug_start()
{
	using namespace Ryan_Debug::registry;
	static const Ryan_Debug::registry::DLLpreamble id(
		"Plugin-test",
		"A test plugin",
		PLUGINID);
	dllInitResult res = Ryan_Debug_registry_register_dll(id, (void*) dllStart);
	if (res != SUCCESS) return res;

	return SUCCESS;
}

/*
#define gcc_init(x) void __attribute__((constructor)) plugin_gcc_init() { x(); }
#define msvc_init(x) BOOL APIENTRY DllMain(HANDLE hModule, DWORD dwReason, LPVOID lpReserved) \
{ if (dwReason == DLL_PROCESS_ATTACH) x(); return true; }


#ifndef _MSC_FULL_VER
#define On_plugin_load(x) gcc_init(x);
#else
#define On_plugin_load(x) msvc_init(x);
#endif

void onLoad()
{
	int i = 0;
	i = 1;
}

On_plugin_load(onLoad);
*/
