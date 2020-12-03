#include <string.h>
#include <stdio.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/SetDirectory.h>
#include <sofa/core/ObjectFactory.h>

#define Q(x) #x
#define QUOTE(x) Q(x)

namespace sofa {

namespace constraintGeometry {

	//Here are just several convenient functions to help user to know what contains the plugin

	extern "C" {
        void initExternalModule();
        const char* getModuleName();
        const char* getModuleVersion();
        const char* getModuleLicense();
        const char* getModuleDescription();
        const char* getModuleComponentList();
	}
	
	void initExternalModule()
	{
		static bool first = true;
		if (first)
		{
            first = false;
#ifdef PLUGIN_DATA_DIR
            sofa::helper::system::DataRepository.addLastPath(std::string(QUOTE(PLUGIN_DATA_DIR)));
#endif
            sofa::helper::system::DataRepository.addLastPath(sofa::helper::system::SetDirectory::GetCurrentDir());
		}
	}

	const char* getModuleName()
	{
        return "ConstraintGeometry";
	}

	const char* getModuleVersion()
	{
#ifdef PLUGIN_GIT_INFO
        return QUOTE(PLUGIN_GIT_INFO);
#else
        return "??? to get the last git hash you must active the setupGit macro in CMakeLists";
#endif
	}

	const char* getModuleLicense()
	{
		return "LGPL";
	}

    const char* getModuleDescription()
    {
        return "Plugin to hendle constraints";
    }

	const char* getModuleComponentList()
	{
        return "";
	}

} 

} 
