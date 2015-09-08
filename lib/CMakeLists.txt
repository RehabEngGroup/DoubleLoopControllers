# Copyright 2015 Luca Tagliapietra, Elena Ceseracciu, Monica Reggiani
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

include_directories(${OPENSIM_INCLUDE_DIRS} ${SIMTK_INCLUDE_DIRS})

set(SIMULATION_MANAGER_SOURCE_FILES SimulationManager.cpp ConfigurationInterpreter.cpp)
set(SIMULATION_MANAGER_HEADER_FILES SimulationManager.h ConfigurationInterpreter.h)
source_group("Headers" FILES ${SIMULATION_MANAGER_HEADER_FILES})
source_group("Sources" FILES ${SIMULATION_MANAGER_SOURCE_FILES})

set(DOUBLE_LOOP_SOURCE_FILES DoubleLoopPIController.cpp RegisterTypes_osimDoubleLoopControllers.cpp)
set(DOUBLE_LOOP_HEADER_FILES DoubleLoopPIController.h RegisterTypes_osimDoubleLoopControllers.h)
source_group("Headers" FILES ${DOUBLE_LOOP_HEADER_FILES})
source_group("Sources" FILES ${DOUBLE_LOOP_SOURCE_FILES})

add_library(SimulationManager STATIC ${SIMULATION_MANAGER_SOURCE_FILES} ${SIMULATION_MANAGER_HEADER_FILES})
add_library(DoubleLoopController SHARED ${DOUBLE_LOOP_SOURCE_FILES} ${DOUBLE_LOOP_HEADER_FILES})
target_link_libraries(SimulationManager ${OPENSIM_LIBRARIES} DoubleLoopController)
target_link_libraries(DoubleLoopController ${OPENSIM_LIBRARIES})

set_target_properties(DoubleLoopController PROPERTIES
                      DEFINE_SYMBOL OSIMPLUGIN_EXPORTS)

set(SimulationManager_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${OPENSIM_INCLUDE_DIRS} ${SIMTK_INCLUDE_DIRS} CACHE INTERNAL "Simulation manager include directories")

set(DoubleLoopController_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${OPENSIM_INCLUDE_DIRS} ${SIMTK_INCLUDE_DIRS} CACHE INTERNAL "Double Loop controllers include directories")

