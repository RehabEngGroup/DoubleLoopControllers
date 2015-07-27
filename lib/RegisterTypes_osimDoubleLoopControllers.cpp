/* -------------------------------------------------------------------------- *
 *       DoubleLoopControllers: RegisterTypes_osimDoubleLoopControllers.cpp   *
 * -------------------------------------------------------------------------- *
 *                                                                            *
 * Copyright 2015 Luca Tagliapietra, Elena Ceseracciu, Monica Reggiani        *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License");            *
 * you may not use this file except in compliance with the License.           *
 * You may obtain a copy of the License at                                    *
 *                                                                            *
 *     http://www.apache.org/licenses/LICENSE-2.0                             *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * ---------------------------------------------------------------------------*/


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
