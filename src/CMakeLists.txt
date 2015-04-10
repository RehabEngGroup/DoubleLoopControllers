# Define project
project (DLcontrollerSim)

include_directories(${OPENSIM_INCLUDE_DIRS} ${SIMTK_INCLUDE_DIRS})
include_directories(${SimulationManager_INCLUDE_DIRS})
include_directories(${DoubleLoopController_INCLUDE_DIRS})

set(ankleAttachment_SIMULATOR_SOURCE_FILES simulate.cpp)

source_group("Sources" FILES ${ankleAttachment_SIMULATOR_SOURCE_FILES})

add_executable(ankleAttachmentSimulator ${ankleAttachment_SIMULATOR_SOURCE_FILES})
target_link_libraries(ankleAttachmentSimulator DoubleLoopController SimulationManager ${OPENSIM_LIBRARIES})