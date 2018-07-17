/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Plugins                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include "constraintGeometry.h"
#include <sofa/helper/system/config.h>
#include <sofa/core/ObjectFactory.h>
#include <string>
#include <sofa/helper/system/FileRepository.h>
#include <stdio.h>

#define Q(x) #x
#define QUOTE(x) Q(x)

#ifndef PLUGIN_DATA_DIR
#define PLUGIN_DATA_DIR_ ""
#else
#define PLUGIN_DATA_DIR_ QUOTE(PLUGIN_DATA_DIR)
#endif

namespace sofa
{

namespace core
{

namespace behavior
{

	//Here are just several convenient functions to help user to know what contains the plugin

	extern "C" {
                SOFA_CONSTRAINTGEOMETRYPLUGIN_API void initExternalModule();
                SOFA_CONSTRAINTGEOMETRYPLUGIN_API const char* getModuleName();
                SOFA_CONSTRAINTGEOMETRYPLUGIN_API const char* getModuleVersion();
                SOFA_CONSTRAINTGEOMETRYPLUGIN_API const char* getModuleLicense();
                SOFA_CONSTRAINTGEOMETRYPLUGIN_API const char* getModuleDescription();
                SOFA_CONSTRAINTGEOMETRYPLUGIN_API const char* getModuleComponentList();
	}
	
	void initExternalModule()
	{
		static bool first = true;
		if (first)
		{
            first = false;
            sofa::helper::system::DataRepository.addLastPath(std::string(PLUGIN_DATA_DIR_));
            sofa::helper::system::DataRepository.addLastPath(std::string(PLUGIN_DATA_DIR_) + "/data");
		}
	}

	const char* getModuleName()
	{
                 return "CollisionAlgorithmPlugin";
	}

	const char* getModuleVersion()
	{
                return "0.0";
	}

	const char* getModuleLicense()
	{
		return "LGPL";
	}


	const char* getModuleDescription()
	{
		return "a simple example of a plugin component module";
	}

	const char* getModuleComponentList()
	{
                return "";
	}

}

} 

} 
