#ifndef _RegisterTypes_OsimDoubleLoopControllers_h_
#define _RegisterTypes_OsimDoubleLoopControllers_h_
#include "osimPluginDLL.h"


extern "C" {

OSIMPLUGIN_API void RegisterTypes_osimDoubleLoopControllers();

}

class dllObjectInstantiator
{
public:
        dllObjectInstantiator();
private:
        void registerDllClasses();
};


#endif // _RegisterTypes_OsimPlugin_h_


