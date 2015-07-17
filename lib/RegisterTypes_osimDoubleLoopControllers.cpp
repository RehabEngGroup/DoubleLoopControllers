
#include <string>
#include <iostream>
#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimDoubleLoopControllers.h"

#include "DoubleLoopPIController.h"

using namespace OpenSim;
using namespace std;

static dllObjectInstantiator dlc_instantiator;

//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the Plugin library.
 */
OSIMPLUGIN_API void RegisterTypes_osimDoubleLoopControllers()
{
    DoubleLoopPIController::registerType();
    //Object::RegisterType( DoubleLoopPIController() );
}

dllObjectInstantiator::dllObjectInstantiator()
{
        registerDllClasses();
}

void dllObjectInstantiator::registerDllClasses()
{
        RegisterTypes_osimDoubleLoopControllers();
}
