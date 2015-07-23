include_directories(${OPENSIM_INCLUDE_DIRS} ${SIMTK_INCLUDE_DIRS})

# Identify the cpp file(s) that were to be built
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

